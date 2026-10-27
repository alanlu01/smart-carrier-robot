"""Localhost/domain 231 probe. Synthetic callbacks/status only; NO motion/actions."""

import json
import os
import time
from pathlib import Path

assert os.environ.get('ROS_DOMAIN_ID') == '231'
assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') == 'LOCALHOST'
assert not os.environ.get('ROS_STATIC_PEERS')

import rclpy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformException
from smart_delivery_core.localization_manager import LocalizationManager
from smart_delivery_core.optional_idle import load_idle_config
from smart_delivery_core.optional_idle_runtime import OptionalIdleRuntime, STATE_QOS
from smart_delivery_core.pipeline_health import DurationMetrics, ReceiptMetrics, ScanPairMetrics


class NoTransforms:
    def lookup_transform(self, *args, **kwargs):
        raise TransformException('synthetic missing TF')


def main():
    rclpy.init()
    probe = rclpy.create_node('codex_recovery_console_probe')
    # Do NOT construct LocalizationManager: its real startup/cache/actions
    # are intentionally out of scope. Bind only its raw-scan receive method.
    receiver = object.__new__(LocalizationManager)
    receiver.duration_metrics = DurationMetrics()
    receiver.receipt_metrics,receiver.scan_pair_metrics = ReceiptMetrics(),ScanPairMetrics()
    receiver.get_clock = probe.get_clock
    probe.create_subscription(LaserScan,'/codex_test/raw_scan',receiver._raw_scan_callback,10)
    publisher = probe.create_publisher(LaserScan,'/codex_test/raw_scan',10)
    root = Path(__file__).resolve().parents[1]/'packages/smart_delivery_core'
    config = load_idle_config(root/'config/optional_idle_features.yaml')
    assert not config['home']['enabled']
    runtime = OptionalIdleRuntime(probe,NoTransforms(),config,root,lambda:True,
        lambda *_:(_ for _ in ()).throw(AssertionError('motion lease used')),
        lambda:(_ for _ in ()).throw(AssertionError('motion guard used')))
    statuses=[]
    probe.create_subscription(String,'/smart_carrier/optional_idle_state',
                              lambda message:statuses.append(json.loads(message.data)),STATE_QOS)
    try:
        end=time.monotonic()+3.
        while publisher.get_subscription_count()<1 and time.monotonic()<end:
            rclpy.spin_once(probe,timeout_sec=.02)
        for _ in range(20):
            message=LaserScan();message.header.stamp=probe.get_clock().now().to_msg()
            message.header.frame_id='synthetic_laser';publisher.publish(message)
            until=time.monotonic()+.05
            while time.monotonic()<until:rclpy.spin_once(probe,timeout_sec=.005)
        assert not runtime._safe()  # Populate detailed diagnosis; never poll/start an action.
        runtime._heartbeat()
        until=time.monotonic()+.3
        while time.monotonic()<until:rclpy.spin_once(probe,timeout_sec=.01)
        count=receiver.receipt_metrics.snapshot(time.monotonic())['scan']['count']
        assert count>=15 and statuses
        assert statuses[-1]['last_unsafe_check']['failed_checks']
        assert statuses[-1]['state']=='idle' and 'wall_durations' in statuses[-1]
        assert receiver.duration_metrics.snapshot(time.monotonic())['raw_scan_callback']['count']==count
        print(json.dumps({'domain':231,'raw_callbacks':count,'status_messages':len(statuses),
            'structured_diagnosis':True,'motor_commands':0,'goals':0}))
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':main()
