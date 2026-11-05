"""Exercise single-thread delivery loops and the real 1 s guard on localhost.

The separate fixture process provides fake actions and high-rate sensor inputs.
No velocity publisher, hardware, production domain or real Nav2 is used.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from smart_delivery_core.delivery_goal_guard import DeliveryGoalGuard, LEASE_QOS
from smart_delivery_core.delivery_inputs import shared_inputs, message_json
from smart_delivery_core.delivery_navigator import DeliveryNavigator
from smart_delivery_core.smart_delivery import go_to_standby
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def fixture():
    """Keep server execution independent of the client's single executor."""
    from rclpy.action import ActionServer, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init()
    server = rclpy.create_node('codex_delivery_fixture')
    guard = DeliveryGoalGuard()
    model = dict(goals=0, cancels=0, feedback=0)

    def execute(handle):
        model['goals'] += 1
        duration = handle.request.pose.pose.position.x
        start = time.monotonic()
        while rclpy.ok() and time.monotonic()-start < duration:
            if handle.is_cancel_requested:
                model['cancels'] += 1
                handle.canceled()
                return NavigateToPose.Result()
            handle.publish_feedback(NavigateToPose.Feedback())
            model['feedback'] += 1
            time.sleep(.05)
        handle.succeed()
        return NavigateToPose.Result()

    action = ActionServer(server, NavigateToPose, '/navigate_to_pose', execute,
                          cancel_callback=lambda _: CancelResponse.ACCEPT,
                          callback_group=ReentrantCallbackGroup())
    publications = []
    for kind, topic, hz, message in (
        (LaserScan, '/scan_filtered', 12., LaserScan(ranges=[2.]*720)),
        (Odometry, '/odom', 20., Odometry()),
        (String, '/vision/semantic_info', 15., String(data=json.dumps({'people': [], 'padding': 'x'*1024}))),
        (String, '/localization/state', 1., String(data='{"ready":true}')),
        (TFMessage, '/tf', 30., TFMessage()),
    ):
        publisher = server.create_publisher(kind, topic, 10)
        timer = server.create_timer(1./hz, lambda p=publisher, m=message: p.publish(m))
        publications.append((publisher, timer))
    stats = server.create_publisher(String, '/codex/delivery_fixture', 10)
    server.create_timer(.2, lambda: stats.publish(String(data=json.dumps(model))))
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    executor.add_node(guard)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        action.destroy()
        guard.destroy_node()
        server.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '231'
    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') == 'LOCALHOST'
    assert not os.environ.get('ROS_STATIC_PEERS')
    if '--fixture' in sys.argv:
        fixture()
        return
    fixture_log = Path(os.environ.get('ROS_LOG_DIR', '/tmp'))/'delivery_fixture.log'
    fixture_log.parent.mkdir(parents=True, exist_ok=True)
    output = fixture_log.open('w')
    process = subprocess.Popen([sys.executable, '-B', __file__, '--fixture'], stdout=output, stderr=output)
    rclpy.init()
    node = DeliveryNavigator()
    assert DeliveryNavigator.isTaskComplete is BasicNavigator.isTaskComplete
    assert rclpy.get_global_executor().__class__.__name__ == 'SingleThreadedExecutor'
    counts = dict(scan=0, odom=0, semantic=0, localization=0, tf=0, timer=0)
    stats = {}
    inputs = shared_inputs(node)
    def observe(key, message):
        counts[key] += 1
        if key in ('semantic', 'localization'):
            message_json(message)
    for kind, topic, key in ((LaserScan, '/scan_filtered', 'scan'), (Odometry, '/odom', 'odom'),
                              (String, '/vision/semantic_info', 'semantic'),
                              (String, '/localization/state', 'localization'), (TFMessage, '/tf', 'tf')):
        inputs.subscribe(kind, topic, lambda m, k=key: observe(k, m), 10)
    node.create_timer(.02, lambda: counts.__setitem__('timer', counts['timer']+1))
    node.create_subscription(String, '/codex/delivery_fixture', lambda m: stats.update(json.loads(m.data)), 10)
    lease_pub = node.create_publisher(Bool, '/smart_carrier/delivery_navigation_active', LEASE_QOS)
    lease = dict(active=False, suspended=False, last=None, gaps=[])
    def heartbeat():
        if lease['active'] and not lease['suspended']:
            now = time.monotonic()
            if lease['last'] is not None:
                lease['gaps'].append(now-lease['last'])
            lease['last'] = now
            lease_pub.publish(Bool(data=True))
    node.create_timer(.2, heartbeat)
    def active(value):
        lease['active'], lease['last'] = value, None
        lease_pub.publish(Bool(data=value))
    def spin_for(seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
    def goal(duration):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = duration
        pose.pose.orientation.w = 1.
        return pose
    def service_loop(duration, lose_lease=False):
        active(True)
        node.goToPose(goal(duration))
        started = time.monotonic()
        while not node.isTaskComplete():
            # Same waits as the production service/standby navigation loops.
            time.sleep(.1)
            rclpy.spin_once(node, timeout_sec=.05)
            if lose_lease and time.monotonic()-started > 1.:
                lease['suspended'] = True
            assert time.monotonic()-started < duration+8., 'navigation failed to terminate'
        result = node.getResult()
        active(False)
        return result
    try:
        assert node.nav_to_pose_client.wait_for_server(timeout_sec=15.)
        spin_for(1.)
        started = time.monotonic()
        assert service_loop(30.) == TaskResult.SUCCEEDED
        service_seconds = time.monotonic()-started
        point = dict(name='fixture', x=30., y=0., yaw=0.)
        outcomes = []
        started = time.monotonic()
        result = go_to_standby(node, {'x': 0., 'y': 0.}, set_navigation_active=active,
                               standby_point=point, on_outcome=lambda p, s: outcomes.append(s))
        standby_seconds = time.monotonic()-started
        assert result == {'x': 30., 'y': 0.} and outcomes == ['approaching', 'arrived']
        spin_for(.4)
        assert stats.get('goals') == 2 and stats.get('cancels') == 0, stats
        max_gap = max(lease['gaps'])
        assert max_gap < .75, max_gap
        assert counts['odom'] > 900 and counts['scan'] > 500 and counts['semantic'] > 700, counts
        # Production sleeps .1 s per cycle: ROS timers coalesce missed periods
        # rather than queuing every 50 Hz tick. Require sustained processing,
        # not the nominal 3000 ticks during the two 30 s actions.
        assert counts['timer'] > 1500, counts
        started = time.monotonic()
        outcomes.clear()
        result = go_to_standby(node, {'x': 0., 'y': 0.}, set_navigation_active=active,
                               standby_point=point, new_task_waiting=lambda: time.monotonic()-started > 2.,
                               on_outcome=lambda p, s: outcomes.append(s))
        assert result is None and outcomes[-1] == 'interrupted_by_order'
        spin_for(.5)
        assert stats['cancels'] == 1, stats
        # Negative control: lost heartbeats must still cancel the owned goal.
        assert service_loop(30., lose_lease=True) == TaskResult.CANCELED
        spin_for(.5)
        assert stats['cancels'] == 2, stats
        print(json.dumps(dict(single_threaded_client=True, service_seconds=round(service_seconds, 3),
            standby_seconds=round(standby_seconds, 3), max_active_lease_gap_sec=round(max_gap, 4),
            input_callbacks=counts, healthy_goals=2, unintended_cancels=0,
            priority_order_cancel_verified=True, one_second_guard_expiry_verified=True,
            hardware_commands=0)))
    finally:
        active(False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=5.)
        output.close()


if __name__ == '__main__':
    main()
