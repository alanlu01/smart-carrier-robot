"""Compare crowd scoring on MCAP offline; never create a node or replay topics."""

import argparse
import datetime
import json
import math
from pathlib import Path
import statistics


def main():
    """Print evidence from recorded state windows, not ground-truth people counts."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path)
    args = parser.parse_args()
    import rosbag2_py
    import yaml
    from rclpy.duration import Duration
    from rclpy.serialization import deserialize_message
    from rclpy.time import Time
    from rosidl_runtime_py.utilities import get_message
    from tf2_ros import Buffer, TransformException
    from smart_delivery_core.optional_idle import DirectionalPeople

    config = yaml.safe_load((args.run_dir/'config_snapshot/optional_idle_features.yaml').read_text())['people']
    buffer = Buffer(cache_time=Duration(seconds=30))
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.run_dir/'rosbag'), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    names = {'/tf', '/tf_static', '/vision/semantic_info', '/smart_carrier/optional_idle_state'}
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in names}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    people = None
    started = None
    previous_state = None
    tz = datetime.timezone(datetime.timedelta(hours=8))

    def label(ns):
        return datetime.datetime.fromtimestamp(ns/1e9, tz).strftime('%H:%M:%S.%f')[:-3]

    while reader.has_next():
        name, data, received = reader.read_next()
        message = deserialize_message(data, types[name])
        if name in ('/tf', '/tf_static'):
            for transform in message.transforms:
                if name == '/tf_static':
                    buffer.set_transform_static(transform, 'offline')
                else:
                    buffer.set_transform(transform, 'offline')
        elif name == '/smart_carrier/optional_idle_state':
            state = json.loads(message.data)
            if state['state'] == 'observing' and people is None:
                people, started = DirectionalPeople(config), received
            elif people is not None and state['state'] not in ('observing', 'waiting_observation_retry'):
                medians = [statistics.median(v) if len(v) >= config['minimum_frames_per_bin'] else 0.
                           for v in people.frames]
                print(json.dumps({'start': label(started), 'end': label(received),
                    'completed_on_robot': state.get('analysis_completed'),
                    'recorded_reason': state['reason'], 'previous_positive_median_bins': sum(v > 0 for v in medians),
                    'new_scoring': people.summary(config['observation_pose'])}, ensure_ascii=False), flush=True)
                people = None
            previous_state = state['state']
        elif people is not None and previous_state == 'observing':
            payload = json.loads(message.data)
            stamp = payload['stamp']['sec']*1_000_000_000+payload['stamp']['nanosec']
            if not -.1 <= (received-stamp)/1e9 <= .5:
                continue
            try:
                transform = buffer.lookup_transform('map', 'base_footprint', Time(nanoseconds=stamp))
            except TransformException:
                continue
            q = transform.transform.rotation
            heading = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            people.observe(payload['detections'], heading, stamp)


if __name__ == '__main__':
    main()
