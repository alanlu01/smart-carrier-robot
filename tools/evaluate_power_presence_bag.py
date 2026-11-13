"""Offline power-status re-evaluation; never creates a node or replays a bag.

Recorded currents are rounded averages, not raw INA registers. Reported status
is a comparison, not a ground-truth label. Use independently confirmed physical
operations when judging accuracy.
"""
import argparse
import datetime
import json
from pathlib import Path

from power_monitor.power_status import SlotStateTracker


def evaluate(rows):
    baseline = [SlotStateTracker(confirm_samples=3) for _ in range(3)]
    candidate = [SlotStateTracker(confirm_samples=3, weak_presence_min_a=.010)
                 for _ in range(3)]
    last = [None] * 3
    transitions = []
    count = 0
    for stamp, payload in rows:
        count += 1
        for index in range(3):
            slot = payload[f'ch{index+1}']
            valid = (slot.get('sensor_ok', True) and slot.get('enabled', True)
                     and payload.get('slot_supply', {}).get('healthy', True))
            args = (slot.get('current'), slot.get('voltage'))
            old = baseline[index].update(*args, sensor_ok=valid, now=stamp)
            new = candidate[index].update(*args, sensor_ok=valid, now=stamp)
            key = old, new
            if key != last[index]:
                transitions.append(dict(stamp=stamp, channel=index+1, old=old, new=new,
                    current=slot.get('current'), voltage=slot.get('voltage'),
                    evidence=candidate[index].evidence()))
                last[index] = key
    return dict(samples=count, transitions=transitions,
        caveat='Rounded recorded averages; comparison only, not ground-truth accuracy')


def read_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String
    import yaml
    metadata = yaml.safe_load((path/'metadata.yaml').read_text())
    storage = metadata['rosbag2_bagfile_information']['storage_identifier']
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=storage),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/power_status']))
    while reader.has_next():
        _, data, stamp = reader.read_next()
        yield stamp/1e9, json.loads(deserialize_message(data, String).data)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report = evaluate(read_bag(args.bag))
    report['generated_utc'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
    content = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        args.output.write_text(content, encoding='utf8')
    else:
        print(content)


if __name__ == '__main__':
    main()
