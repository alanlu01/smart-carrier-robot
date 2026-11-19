"""Offline recorded costmaps only; no ROS node, service, replay or motor output."""
import json
import math
from pathlib import Path
import struct
import subprocess
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    bag, executable, output = map(Path, sys.argv[1:])
    output.mkdir(parents=True, exist_ok=True)
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),
                rosbag2_py.ConverterOptions('',''))
    classes={x.name:get_message(x.type) for x in reader.get_all_topics_and_types()
             if x.name in {'/global_costmap/costmap','/plan'}}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(classes)))
    targets=(1795064080.828,1795064112.443,1795064186.698)
    grid=None;results=[]
    while reader.has_next():
        topic,data,stamp=reader.read_next();at=stamp/1e9
        if at>max(targets)+.1:break
        message=deserialize_message(data,classes[topic])
        if topic=='/global_costmap/costmap':
            grid=message
        elif grid is not None and any(abs(at-t)<.03 for t in targets):
            assert grid.info.origin.orientation.w==1.
            points=message.poses
            start,end=points[0].pose,points[-1].pose
            yaw=lambda q:math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            values=np.asarray(grid.data,dtype=np.int16)
            # Invert OccupancyGrid's quantized soft-cost LUT; hard/unknown
            # values are exact. This is not PlannerServer's internal snapshot.
            costs=np.where(values<0,255,np.where(values==100,254,
                np.where(values==99,253,np.where(values==0,0,
                1+np.rint((values-1)*251/97))))).astype(np.uint8)
            fixture=output/f'costmap_{len(results)}.bin'
            fixture.write_bytes(struct.pack('<ii9d',grid.info.width,grid.info.height,
                grid.info.resolution,grid.info.origin.position.x,grid.info.origin.position.y,
                start.position.x,start.position.y,0.,end.position.x,end.position.y,yaw(end.orientation))
                +costs.tobytes())
            completed=subprocess.run([str(executable),str(fixture)],text=True,capture_output=True,timeout=5.)
            assert completed.returncode==0,(at,completed.stderr)
            result=json.loads(completed.stdout);assert result['passed'] and result['retained']
            result.update(recorded_at=at,fixture=str(fixture),quantized_soft_costs=True)
            results.append(result)
    assert len(results)==3,len(results)
    (output/'results.json').write_text(json.dumps(results,indent=2))
    print(json.dumps([dict(recorded_at=r['recorded_at'],length_m=r['length_m'],
        retained=r['retained'],milliseconds=r['milliseconds']) for r in results]))


if __name__=='__main__':
    main()
