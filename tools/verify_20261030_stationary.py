"""Re-evaluate the recorded stationary mismatch; no ROS init or bag playback."""
import datetime,json,sys
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from smart_delivery_core.stationary_map_guard import StationaryMapGuard


def main():
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(Path(sys.argv[1])),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/localization/state']))
    guard=StationaryMapGuard();start=confirmed=None;observations=0
    zone=datetime.timezone(datetime.timedelta(hours=8))
    while reader.has_next():
        _,data,ns=reader.read_next();at=ns/1e9
        clock=datetime.datetime.fromtimestamp(at,zone).strftime('%H:%M:%S')
        if not '11:18:15'<=clock<='11:19:24':continue
        p=json.loads(deserialize_message(data,String).data)
        age=p['map_match_age_sec'];score=p['map_match_score']
        if age is None or score is None:continue
        ready=(p['xy_std']<=.25 and p['yaw_std']<=.17 and not p['verification_blockers']
            and p['map_match_sector_count']>=4 and p['map_match_near_fraction']<=.6
            and p['raw_scan_age_sec']<=.30 and p['scan_stamp_age_sec']<=.30
            and p['odom_age_sec']<=.25)
        qualified=ready and (score<.30 or guard.active)
        result=guard.observe(at,at-age,qualified,recovered=ready and score>=.35)
        if guard.active and start is None:start=at
        observations+=1
        if result:
            confirmed=at;break
    assert start and confirmed and confirmed-start<=20.,(start,confirmed)
    print(json.dumps(dict(first_suspicion=datetime.datetime.fromtimestamp(start,zone).isoformat(),
        confirmed=datetime.datetime.fromtimestamp(confirmed,zone).isoformat(),
        elapsed_sec=round(confirmed-start,3),independent_samples=guard.count,
        evaluated_states=observations,mode='offline_counterfactual_not_field_timing')))


if __name__=='__main__':main()
