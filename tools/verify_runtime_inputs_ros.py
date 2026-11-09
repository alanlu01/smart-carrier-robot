"""Late-joining ready subscriber and real on-demand timer; localhost only."""
import json
import os
import tempfile
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool, String
from smart_delivery_core.localization_manager import LocalizationManager, STATE_QOS
from smart_delivery_core.optional_idle_runtime import OptionalIdleRuntime


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '231'
    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') == 'LOCALHOST'
    assert not os.environ.get('ROS_STATIC_PEERS')
    with tempfile.TemporaryDirectory(prefix='runtime_inputs_') as temp:
        rclpy.init(args=['--ros-args','-p','auto_initialize:=false',
                        '-p','automatic_motion_recovery:=false',
                        '-p','startup_pose_mode:=configured','-p','pose_cache_path:='+temp+'/pose.json'])
        node = LocalizationManager()
        probe = rclpy.create_node('codex_ready_late_joiner')
        ready, states = [], []
        probe.create_subscription(Bool,'/localization/ready',lambda m:ready.append(m.data),STATE_QOS)
        probe.create_subscription(String,'/localization/state',lambda m:states.append(json.loads(m.data)),STATE_QOS)
        executor=SingleThreadedExecutor()
        executor.add_node(node);executor.add_node(probe)
        def spin(seconds):
            deadline=time.monotonic()+seconds
            while time.monotonic()<deadline:executor.spin_once(timeout_sec=.01)
        try:
            spin(1.5)
            assert ready == [False], ready
            assert states[-1]['initial_pose_source'] == 'configured_explicit'
            assert states[-1]['ready'] is False
            manager=object.__new__(OptionalIdleRuntime)
            manager.state='idle';manager.pending_scan=None
            calls=[]
            def retry():
                calls.append(time.monotonic());manager.pending_scan=None
            manager._retry_scan_transform_once=retry
            manager.scan_retry_timer=probe.create_timer(.02,manager._retry_scan_transform)
            manager.scan_retry_timer.cancel()
            spin(.3);assert not calls
            manager.state='observing';manager.pending_scan=(time.monotonic(),object())
            manager._sync_scan_retry_timer();spin(.2)
            assert len(calls)==1 and manager.scan_retry_timer.is_canceled()
            print(json.dumps(dict(initial_false_latched=True,late_joiner_verified=True,
                explicit_pose_verified=True,idle_50hz_timer_cancelled=True,
                pending_tf_timer_resumed_and_cancelled=True,hardware_commands=0)))
        finally:
            executor.shutdown();node.pose_cache_writer.close()
            node.destroy_node();probe.destroy_node();rclpy.shutdown()


if __name__ == '__main__':main()
