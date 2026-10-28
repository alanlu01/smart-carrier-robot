"""Isolated real manager construction; no executor spin, goals or motor commands."""

import json
import os
from pathlib import Path
from tempfile import TemporaryDirectory

assert os.environ.get('ROS_DOMAIN_ID') == '231'
assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') == 'LOCALHOST'
assert not os.environ.get('ROS_STATIC_PEERS')

import rclpy
from smart_delivery_core.localization_manager import LocalizationManager
from smart_delivery_core.localization_persistence import load_cache


def main():
    with TemporaryDirectory(prefix='codex_pose_cache_smoke_') as directory:
        cache = Path(directory)/'pose.json'
        rclpy.init(args=['--ros-args', '-p', f'pose_cache_path:={cache}',
                        '-p', 'auto_initialize:=false', '-p', 'auto_motion_recovery:=false'])
        node = None
        try:
            node = LocalizationManager()
            marker = load_cache(cache)
            assert marker['pose'] is None and marker['boot_id'] == node.boot_id
            assert marker['map_identity'] == node.map_identity
            node._publish_state()
            assert not node.spin_in_progress and node.lateral_phase is None
            assert node.pose_cache_writer.snapshot()['submitted'] == 0
            print(json.dumps({'real_manager_startup':True, 'boot_marker_preserved':True,
                              'state_serialization':True, 'motor_commands':0, 'goals':0}))
        finally:
            if node is not None:
                assert node.pose_cache_writer.close()
                node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
