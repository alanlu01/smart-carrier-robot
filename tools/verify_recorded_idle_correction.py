"""Read a parked bag window offline; never create a node, replay, or publish."""
import argparse
import datetime
import json
import math
from pathlib import Path
import types


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir',type=Path)
    parser.add_argument('--start',default='19:48:12.700')
    parser.add_argument('--end',default='19:49:37.300')
    parser.add_argument('--baseline-file',type=Path)
    args=parser.parse_args()
    import rosbag2_py
    from rclpy.duration import Duration
    from rclpy.serialization import deserialize_message
    from rclpy.time import Time
    from rosidl_runtime_py.utilities import get_message
    from tf2_ros import Buffer,TransformException
    from smart_delivery_core.standby_idle_clock import StandbyIdleClock
    clock=StandbyIdleClock();clock.mark_parked(dict(name='CYCU EE',x=2.,y=5.18,yaw=0.))
    old=None
    if args.baseline_file:
        mod=types.ModuleType('baseline_idle_clock')
        exec(compile(args.baseline_file.read_text(),str(args.baseline_file),'exec'),mod.__dict__)
        old=mod.StandbyIdleClock();old.mark_parked(clock.point)
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.run_dir/'rosbag'),storage_id='mcap'),
                rosbag2_py.ConverterOptions('',''))
    names={'/tf','/tf_static','/odom','/localization/state','/smart_carrier/optional_idle_state'}
    classes={t.name:get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in names}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(classes)))
    buffer=Buffer(cache_time=Duration(seconds=30));odom=None;quality={};samples=0
    first_odom=None;max_motion=0.;max_yaw=0.;old_resets=0;old_reason=None;last=None
    tz=datetime.timezone(datetime.timedelta(hours=8))
    def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    while reader.has_next():
        topic,data,ns=reader.read_next();now=ns/1e9
        label=datetime.datetime.fromtimestamp(now,tz).strftime('%H:%M:%S.%f')[:-3]
        if label>args.end:break
        m=deserialize_message(data,classes[topic])
        if topic in ('/tf','/tf_static'):
            for tr in m.transforms:
                if topic=='/tf_static':buffer.set_transform_static(tr,'offline')
                else:buffer.set_transform(tr,'offline')
        elif topic=='/odom':odom=m
        elif topic=='/localization/state':quality=json.loads(m.data)
        elif topic=='/smart_carrier/optional_idle_state' and label>=args.start and odom is not None:
            p=json.loads(m.data)
            try:
                tr=buffer.lookup_transform('map','base_footprint',Time())
                v=tr.transform
            except TransformException:continue
            stamp=tr.header.stamp.sec+tr.header.stamp.nanosec/1e9
            pos=dict(x=v.translation.x,y=v.translation.y,yaw=yaw(v.rotation))
            op=odom.pose.pose;physical=(op.position.x,op.position.y,yaw(op.orientation))
            if first_odom is None:first_odom=physical
            max_motion=max(max_motion,math.hypot(physical[0]-first_odom[0],physical[1]-first_odom[1]))
            max_yaw=max(max_yaw,abs(pos['yaw']))
            trusted=(quality.get('state')=='LOCALIZED' and quality.get('dispatch_allowed') is True
                and quality.get('xy_std',math.inf)<=.20 and quality.get('yaw_std',math.inf)<=math.radians(8.)
                and quality.get('map_match_score',0.)>=.60 and not quality.get('mild_map_observation'))
            args_update=dict(parked=True,safe=p.get('safety_check',{}).get('safe') is True
                and -.1<=now-stamp<=.30,stationary=True,pose=pos,odom=physical,
                trusted_map_correction=trusted)
            clock.update(now,**args_update)
            if old is not None:
                old.update(now,**args_update)
                if old.reason=='map_pose_deviation_persistent' and old_reason!=old.reason:old_resets+=1
                old_reason=old.reason
            samples+=1;last=(now,args_update)
    assert samples>=100 and max_motion<.001 and math.radians(12.)<max_yaw<math.radians(20.)
    recorded=clock.snapshot(last[0])
    assert recorded['deviation_resets']==1 and not recorded['correction_disqualified'],recorded
    # Separate synthetic continuation: retained parking becomes high-quality,
    # remains physically stopped, and supplies continuous fresh evidence.
    for i in range(1,10):
        future=dict(last[1],safe=True,trusted_map_correction=True)
        clock.update(last[0]+i*.5,**future)
    assert clock.correction_used and clock.elapsed<=4.5
    print(json.dumps(dict(passed=True,offline=True,samples=samples,max_odom_motion_m=max_motion,
        max_map_yaw_deg=math.degrees(max_yaw),baseline_reset_transitions=old_resets,
        recorded_window=recorded,synthetic_trusted_continuation=clock.snapshot(last[0]+4.5)),ensure_ascii=False))


if __name__=='__main__':main()
