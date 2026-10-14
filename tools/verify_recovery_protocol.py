"""Exercise Jazzy cancel RPCs in an isolated domain with synthetic servers.

No production node constructors, robot topics, goals, or hardware interfaces.
Run with ROS_DOMAIN_ID=231 and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST.
"""
import json
import os
import time
from types import SimpleNamespace

assert os.environ.get('ROS_DOMAIN_ID') == '231'
assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') == 'LOCALHOST'

import rclpy
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from hailo_vision.fusion_node import SensorFusionNode
from hailo_vision.recovery_lease import RecoveryLease
from smart_delivery_core.wheel_feedback import WheelStopEvidence


def main():
    rclpy.init()
    node = rclpy.create_node('codex_recovery_protocol_probe')
    servers = []
    guard = object.__new__(SensorFusionNode)
    started = time.monotonic()
    guard.recovery_lease = RecoveryLease()
    guard.recovery_lease.receive({'token': 'test', 'owner': 'old', 'active': True,
                                  'permit': True}, started-1.)
    guard.recovery_stop = WheelStopEvidence(started)
    guard.recovery_odom_at = guard.recovery_odom_stamp_ns = None
    guard.recovery_odom_stopped = False
    guard.recovery_last_nonzero_cmd_at = started-1.
    guard.recovery_handoff = False
    guard.recovery_last_release = {}
    guard.recovery_action_status = {}
    guard.recovery_cancel_pending = {}
    guard.recovery_cancel_confirmed = {}
    guard.recovery_cancel_probe_at = 0.
    guard.recovery_cancel_clients = {}
    guard.get_clock = node.get_clock
    guard.get_publishers_info_by_topic = lambda topic: node.get_publishers_info_by_topic(
        '/codex_protocol'+topic)
    commands, messages = [], []
    guard.pub_safe_cmd = SimpleNamespace(publish=commands.append)
    guard.recovery_guard_pub = SimpleNamespace(publish=messages.append)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    try:
        for name in ('navigate_to_pose', 'navigate_through_poses', 'spin'):
            action = '/codex_protocol/'+name
            servers.append(ActionServer(node, NavigateToPose, action,
                                        lambda _: NavigateToPose.Result()))
            guard.recovery_cancel_clients[name] = node.create_client(CancelGoal,
                                                                     action+'/_action/cancel_goal')
            node.create_subscription(GoalStatusArray, action+'/_action/status',
                                     guard.recovery_status_callback(name), qos)
        for client in guard.recovery_cancel_clients.values():
            assert client.wait_for_service(timeout_sec=2.)
        guard.recovery_lease_callback(SimpleNamespace(data=json.dumps({
            'token': 'test', 'owner': 'new', 'active': True, 'permit': False, 'handoff': True})))
        assert guard.recovery_lease.blocked(time.monotonic())
        group_at = 0.
        deadline = time.monotonic()+5.
        while time.monotonic() < deadline:
            now = time.monotonic()
            odom = Odometry()
            odom.header.stamp = node.get_clock().now().to_msg()
            guard.recovery_odom_callback(odom)
            if now-group_at >= .5:
                group_at = now
                for letter in 'ABCD':
                    guard.recovery_feedback_callback(SimpleNamespace(data=letter+'_RT: 0'))
            guard.recovery_handoff_probe(now)
            rclpy.spin_once(node, timeout_sec=.02)
            if not guard.recovery_disarm_blockers(time.monotonic()):
                break
        assert not guard.recovery_disarm_blockers(time.monotonic()), messages[-1].data
        guard.recovery_lease_callback(SimpleNamespace(data=json.dumps({
            'token': 'test', 'owner': 'new', 'active': False})))
        assert not guard.recovery_lease.enabled
        assert commands and all(c.linear.x == c.linear.y == c.angular.z == 0 for c in commands)
        print(json.dumps({'isolated_domain': 231, 'synthetic_cancel_servers': 3,
                          'handoff_released': True, 'production_goals_sent': 0,
                          'nonzero_commands': 0}))
    finally:
        for server in servers:
            server.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
