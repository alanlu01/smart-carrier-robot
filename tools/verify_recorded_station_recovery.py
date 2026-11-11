"""Read a bag sequentially for counterfactual gates; no ROS node or replay."""
import json
import math
import sys
import datetime
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rclpy.time import Time
from tf2_ros import Buffer,TransformException
from smart_delivery_core.station_geometry import StationGrid
from smart_delivery_core.recovery_scan_evidence import RecoveryScanEvidence
from smart_delivery_core.recovery_translation import load_self_mask,swept_scan_report
from smart_delivery_core.recovery_spin import RecoverySpin
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def main():
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=sys.argv[1],storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    names={'/map','/global_costmap/costmap','/tf','/tf_static','/delivery/station_state',
           '/scan','/scan_filtered','/localization/state','/odom','/chassis_cmd_vel'}
    types={t.name:get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in names}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    tf=Buffer();cache=RecoveryScanEvidence();static=grid=None;phase='';loc='';commanded=moving=False
    mask=load_self_mask(Path(get_package_share_directory('smart_delivery_core'))/'config/my_laser_filter.yaml',.22,.16)
    cases=[];evidence=[];seen=set();zone=datetime.timezone(datetime.timedelta(hours=8))
    clock=lambda t:datetime.datetime.fromtimestamp(t,zone).strftime('%H:%M:%S.%f')[:-3]
    yaw=lambda q:math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    def point(transform):return transform.translation.x,transform.translation.y,yaw(transform.rotation)
    while reader.has_next():
        topic,data,ns=reader.read_next();m=deserialize_message(data,types[topic]);t=ns/1e9
        if topic in ('/tf','/tf_static'):
            for tr in m.transforms:(tf.set_transform_static if topic=='/tf_static' else tf.set_transform)(tr,'recorded')
        elif topic=='/map':static=StationGrid(m)
        elif topic=='/global_costmap/costmap':grid=StationGrid(m)
        elif topic=='/odom':moving=math.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y)>.01 or abs(m.twist.twist.angular.z)>.02
        elif topic=='/chassis_cmd_vel':commanded=math.hypot(m.linear.x,m.linear.y)>.02 or abs(m.angular.z)>.03
        elif topic in ('/scan','/scan_filtered') and loc.startswith('RECOVERING') and not commanded and not moving:
            try:
                stamp=Time(nanoseconds=m.header.stamp.sec*10**9+m.header.stamp.nanosec)
                mount=tf.lookup_transform('base_footprint',m.header.frame_id,stamp).transform
                laser=tf.lookup_transform('odom',m.header.frame_id,stamp).transform
                base=tf.lookup_transform('odom','base_footprint',stamp).transform
                cache.observe(topic[1:],m,t,point(laser),point(base),point(mount),mask,mount.translation.z)
            except TransformException:pass
        elif topic=='/delivery/station_state':
            p=json.loads(m.data)
            if p['phase']=='waiting_data' and phase!='waiting_data':episode=clock(t)
            phase=p['phase']
            if phase=='waiting_clearance' and p['goals_submitted']==0 and episode not in seen and grid and static:
                seen.add(episode)
                pose=point(tf.lookup_transform('map','base_footprint',Time()).transform)
                target=tuple(p['target'])
                candidate=grid.staging(target,pose,final_clear=False,static_grid=static)
                cases.append(dict(at=clock(t),target=target,pose=pose,
                    old_staging=grid.staging(target,pose),new_preparation=candidate,
                    final_sweep_clear=grid.segment(candidate,target) if candidate else None))
        elif topic=='/localization/state':
            p=json.loads(m.data);loc=p['state']
            if (p.get('lateral_path_reports') and p.get('global_attempts')==2
                    and loc=='RECOVERING_GLOBAL' and p.get('recovery_step')=='global_settle'):
                key=clock(t)[:5]
                if key not in seen:
                    seen.add(key)
                    try:
                        pose=point(tf.lookup_transform('odom','base_footprint',Time()).transform)
                        reports={str(direction):{name:cache.translation_report(name,pose,t,t,direction,.3,.22,.16)
                            for name in ('scan','scan_filtered')} for direction in (1,-1)}
                        evidence.append(dict(at=clock(t),old=p['lateral_path_reports'],new=reports))
                    except TransformException:pass
    assert len(cases)>=3,cases
    assert all(c['old_staging'] is None and c['new_preparation'] and c['final_sweep_clear'] is False for c in cases),cases
    tails=[]
    for degrees in (3.1,5.1,8.8):
        turn=RecoverySpin(2*math.pi,0.,0.,1);turn.travel=turn.target-math.radians(degrees);turn.finishing=True
        native=math.sqrt(2*.8*turn.remaining)
        tails.append(dict(remaining_deg=degrees,old_command=native*.25,new_command=native*turn.speed_limit()))
    print(json.dumps(dict(remote_cases=cases,tail_envelopes=tails,lateral_evidence=evidence,
        replayed=False,nodes_created=0,hardware_commands=0),ensure_ascii=False))


if __name__=='__main__':main()
