"""Offline recorded planner/station regressions; never starts ROS or motors."""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import re
import struct
import subprocess

import numpy as np
import rosbag2_py
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer, TransformException
from smart_delivery_core.station_geometry import StationGrid


def stamp(message):
    return message.header.stamp.sec*10**9+message.header.stamp.nanosec


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def messages(bag, names):
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),
                rosbag2_py.ConverterOptions('',''))
    classes={x.name:get_message(x.type) for x in reader.get_all_topics_and_types() if x.name in names}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(classes)))
    while reader.has_next():
        topic,data,ns=reader.read_next()
        yield topic,deserialize_message(data,classes[topic]),ns


def run_probe(executable, fixture):
    result=subprocess.run([str(executable),str(fixture)],capture_output=True,text=True,timeout=5.)
    if result.returncode:
        return dict(passed=False,returncode=result.returncode,error=result.stderr[:400])
    payload=json.loads(result.stdout)
    payload.pop('points',None)
    return payload


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag',type=Path)
    parser.add_argument('probe',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--baseline',type=Path)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    buffer=Buffer(cache_time=Duration(seconds=1500))
    for topic,message,_ in messages(args.bag,{'/tf','/tf_static'}):
        for transform in message.transforms:
            (buffer.set_transform_static if topic=='/tf_static' else buffer.set_transform)(transform,'recording')

    def pose_at(ns):
        try:
            tr=buffer.lookup_transform('map','base_footprint',Time(nanoseconds=int(ns))).transform
            return (tr.translation.x,tr.translation.y,yaw(tr.rotation))
        except TransformException:return None

    grid=static=None;grids=deque(maxlen=80);station={};planners=[];docking=[];seen=set()
    topics={'/global_costmap/costmap','/map','/rosout','/delivery/station_state'}
    for topic,message,ns in messages(args.bag,topics):
        if topic=='/global_costmap/costmap':
            grid=message;grids.append((stamp(message),message))
        elif topic=='/map':static=StationGrid(message)
        elif topic=='/delivery/station_state':
            station=json.loads(message.data)
            if station.get('phase')!='waiting_clearance' or station.get('target_name') not in ('右樓梯口','座位'):
                continue
            motion=station.get('motion_check',{})
            recorded_stamp=motion.get('costmap_stamp_ns')
            match=next((g for gs,g in grids if gs==recorded_stamp),None) if recorded_stamp else grid
            if match is None:continue
            key=(station.get('target_name'),stamp(match))
            if key in seen:continue
            seen.add(key)
            start=pose_at(station.get('inputs',{}).get('tf',{}).get('stamp_ns',ns))
            target=station.get('target')
            if start is None or target is None:continue
            sg=StationGrid(match)
            with sg.evaluation():candidate=sg.docking_candidate(start,target,static)
            if candidate:
                assert math.hypot(candidate[0]-target[0],candidate[1]-target[1])<=.0500001
                assert candidate[2]==target[2] and sg.segment(start,candidate)
                assert static is None or static.segment(start,candidate)
            docking.append(dict(at=ns/1e9,target=station['target_name'],start=start,requested=target,
                exact_motion_snapshot=recorded_stamp is not None,grid_stamp_ns=stamp(match),candidate=candidate))
        elif (topic=='/rosout' and grid is not None and message.name=='planner_server'
              and 'GuardedGridBased plugin failed to plan' in message.msg):
            match=re.search(r'from \(([^,]+), ([^)]+)\) to \(([^,]+), ([^)]+)\)',message.msg)
            start=pose_at(ns);target=station.get('stage') or station.get('target')
            if not match or start is None or target is None:continue
            values=np.asarray(grid.data,dtype=np.int16)
            # Exact lethal/unknown, quantized soft-cost inverse, not the internal
            # planner costmap. Both binaries receive the identical fixture.
            costs=np.where(values<0,255,np.where(values==100,254,np.where(values==99,253,
                np.where(values==0,0,1+np.rint((values-1)*251/97))))).astype(np.uint8)
            fixture=args.output/f'planner_{len(planners)}.bin'
            fixture.write_bytes(struct.pack('<ii9d',grid.info.width,grid.info.height,
                grid.info.resolution,grid.info.origin.position.x,grid.info.origin.position.y,*start,*target)
                +costs.tobytes())
            candidate=run_probe(args.probe,fixture)
            row=dict(at=ns/1e9,start=start,target=target,grid_stamp_ns=stamp(grid),candidate=candidate)
            if args.baseline:row['baseline']=run_probe(args.baseline,fixture)
            sg=StationGrid(grid)
            row['can_turn_at_start']=sg.turn_clear(start,target[2])
            row['fixed_heading_translation_clear']=sg.segment((start[0],start[1],target[2]),target)
            planners.append(row)
    assert len(planners)==5,len(planners)
    assert sum(p['candidate'].get('passed',False) for p in planners)>=4,planners
    for name in ('右樓梯口','座位'):
        assert any(p['target']==name and p['candidate'] is not None for p in docking),(name,docking)
    result=dict(planners=planners,docking=docking,quantized_soft_costs=True,
                full_body_and_padding_unchanged=True,robot_motion=False)
    (args.output/'results.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(planner_cases=len(planners),accepted=sum(p['candidate'].get('passed',False)
        for p in planners),docking_cases=len(docking),safe_candidates=sum(p['candidate'] is not None
        for p in docking)),ensure_ascii=False))


if __name__=='__main__':main()
