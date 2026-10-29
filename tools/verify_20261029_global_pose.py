"""Offline regression on the retained 10/29 bag; NO ROS init, replay or writes."""
import collections
import datetime
import json
import math
import statistics
import sys
import time
import rosbag2_py
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer,TransformException
from smart_delivery_core.global_pose_gate import ray_consistency

zone=datetime.timezone(datetime.timedelta(hours=8))
reader=rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=sys.argv[1],storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
names=('/tf','/tf_static','/map','/scan_filtered','/localization/state')
types={entry.name:get_message(entry.type) for entry in reader.get_all_topics_and_types() if entry.name in names}
reader.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
buffer=Buffer(cache_time=Duration(seconds=20.))
scan=grid=None
previous=None
results=collections.defaultdict(list)
skips=collections.Counter()
durations=[]
while reader.has_next():
    topic,data,ns=reader.read_next()
    message=deserialize_message(data,types[topic])
    if topic in ('/tf','/tf_static'):
        for transform in message.transforms:
            (buffer.set_transform_static if topic=='/tf_static' else buffer.set_transform)(transform,'recorded')
    elif topic=='/map':grid=message
    elif topic=='/scan_filtered':scan=message
    elif topic=='/localization/state' and grid is not None and scan is not None:
        clock=datetime.datetime.fromtimestamp(ns/1e9,zone).strftime('%H:%M:%S')
        group=('wrong_candidate' if '13:44:40'<=clock<='13:47:12' else
               'reference' if ('13:28:20'<=clock<='13:29:00' or '13:33:45'<=clock<='13:34:05'
                               or '13:49:40'<=clock<='13:50:00') else None)
        if group is None or clock==previous:continue
        previous=clock
        try:
            transform=buffer.lookup_transform('map',scan.header.frame_id,Time.from_msg(scan.header.stamp))
        except TransformException:
            skips[group]+=1
            continue
        t=transform.transform.translation;q=transform.transform.rotation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        started=time.perf_counter()
        report=ray_consistency(grid,scan,(t.x,t.y,yaw))
        durations.append(time.perf_counter()-started)
        results[group].append([clock,report])
summary={key:dict(samples=len(rows),verified=sum(row[1]['verified'] for row in rows),
                 conflict_min=min(row[1].get('conflict_fraction',1.) for row in rows),
                 conflict_max=max(row[1].get('conflict_fraction',1.) for row in rows))
         for key,rows in results.items()}
print(json.dumps(dict(summary=summary,tf_skips=skips,mean_sec=statistics.mean(durations),
                     max_sec=max(durations),ros_nodes=0,motor_commands=0),ensure_ascii=False))
assert summary['wrong_candidate']['samples']>=100 and summary['wrong_candidate']['verified']==0
assert summary['reference']['samples']>=50
assert summary['reference']['verified']/summary['reference']['samples']>=.90
