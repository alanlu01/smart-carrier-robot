"""Execute optional state transitions with fake actions, never ROS nodes/motion."""

import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace

from geometry_msgs.msg import TransformStamped
from nav2_simple_commander.robot_navigator import TaskResult
from nav_msgs.msg import Odometry
import pytest
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from smart_delivery_core.optional_idle_runtime import OptionalIdleRuntime
from smart_delivery_core.optional_idle import load_idle_config

ROOT = Path(__file__).parents[1]


class Future:
    def __init__(self, value, complete=True):
        self.value, self.complete = value, complete

    def done(self):
        return self.complete

    def result(self):
        return self.value


class Navigator:
    def __init__(self):
        self.sent, self.subscriptions, self.leases = [], [], []
        self.cancel_calls = 0
        self.complete = False
        self.result = TaskResult.SUCCEEDED
        self.handle = SimpleNamespace(accepted=True, get_result_async=lambda: Future(None, False),
                                      cancel_goal_async=self.cancel)
        self.request = Future(self.handle)
        self.nav_to_pose_client = SimpleNamespace(server_is_ready=lambda: True,
                                                  send_goal_async=self.send)
        self._feedbackCallback = lambda _: None

    def send(self, goal, feedback):
        self.sent.append(goal)
        return self.request

    def cancel(self):
        self.cancel_calls += 1
        return Future(None)

    def create_publisher(self, *args):
        return SimpleNamespace(publish=lambda message: None)

    def create_subscription(self, msgtype, topic, callback, qos):
        self.subscriptions.append(topic)

    def create_service(self, *args):
        pass

    def create_timer(self, *args):
        pass

    def get_clock(self):
        return SimpleNamespace(now=lambda: Time(nanoseconds=100_000_000_000))

    def get_logger(self):
        return SimpleNamespace(warning=lambda _: None)

    def isTaskComplete(self):
        return self.complete

    def getResult(self):
        return self.result


@pytest.fixture
def runtime(monkeypatch, tmp_path):
    from smart_delivery_core import optional_idle_runtime as module
    config = load_idle_config(ROOT / 'config/optional_idle_features.yaml')
    config['home'].update(enabled=True, pose={'name': 'HOME', 'x': 1., 'y': 2., 'yaw': 0.},
                          low_voltage_v=10., battery_source_verified=True)
    config['people'].update(enabled=True,
        observation_pose={'name': 'OBS', 'x': 0., 'y': 0., 'yaw': 0.},
        standby_points=[{'name': 'A', 'x': 1., 'y': 0., 'yaw': 0.},
                        {'name': 'B', 'x': -1., 'y': 0., 'yaw': 0.}])
    moment = [100.]
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=lambda: moment[0]))
    monkeypatch.setattr(module.Path, 'home', lambda: tmp_path)
    navigator = Navigator()
    transform = TransformStamped()
    transform.header.stamp.sec = 100
    transform.transform.rotation.w = 1.
    buffer = SimpleNamespace(lookup_transform=lambda *args: transform)
    manager = OptionalIdleRuntime(navigator, buffer, config, ROOT, lambda: True,
                                  navigator.leases.append, lambda: navigator.leases.append('suspended'))
    manager.map_matches = True
    manager.home.healthy, manager.home.last_receipt, manager.home.last_voltage = True, 100., 12.
    odom = Odometry()
    odom.header.stamp.sec = 100
    manager._odom(odom)
    return manager, navigator, moment


def test_wrong_map_or_absent_server_never_sends_an_optional_goal(runtime):
    manager, navigator, _ = runtime
    manager.map_matches = False
    assert not manager._start(manager.config['home']['pose'], 'going_home', 'home.xml', 10.)
    assert not navigator.sent
    manager.map_matches = True
    navigator.nav_to_pose_client.server_is_ready = lambda: False
    assert not manager._start(manager.config['home']['pose'], 'going_home', 'home.xml', 10.)
    assert not navigator.sent


def test_people_goal_is_preempted_by_new_work_and_must_stop_before_service(runtime):
    manager, navigator, moment = runtime
    manager._safe = lambda: True
    manager._stationary = lambda: True
    assert manager._start(manager.config['people']['observation_pose'], 'going_observation', '', 10.)
    assert manager.poll(True)
    assert navigator.cancel_calls == 1
    navigator.complete, navigator.result = True, TaskResult.CANCELED
    assert manager.poll(True)
    assert not manager.motion and manager.settling_after_cancel
    assert manager.poll(True)
    moment[0] += 1.1
    assert not manager.poll(True)
    assert not manager.settling_after_cancel


def test_home_finishes_instead_of_being_preempted_by_new_unstarted_orders(runtime):
    manager, navigator, _ = runtime
    manager._safe = lambda: True
    manager.home.latched = True
    assert manager._start(manager.config['home']['pose'], 'going_home', 'home.xml', 10.)
    assert manager.poll(True)
    assert navigator.cancel_calls == 0


@pytest.mark.parametrize('state', [
    'waiting_observation_stop', 'people_destination_selected', 'people_arrival_check'])
def test_new_service_discards_an_observation_between_actions(runtime, state):
    manager, navigator, _ = runtime
    manager.state = state
    assert not manager.poll(True)
    assert manager.state == 'idle'
    assert manager.reason == 'observation_preempted_between_actions'
    assert not manager.idle()
    assert not navigator.sent


def test_goal_transport_error_leaves_stop_guard_armed(runtime):
    manager, navigator, _ = runtime
    manager._safe = lambda: True
    def transport_error(*args):
        raise RuntimeError('uncertain delivery')
    navigator.nav_to_pose_client.send_goal_async = transport_error
    with pytest.raises(RuntimeError):
        manager._start(manager.config['home']['pose'], 'going_home', 'home.xml', 10.)
    assert navigator.leases == [True, 'suspended']


def test_cancellation_or_goal_acceptance_timeout_does_not_allow_another_goal(runtime):
    manager, navigator, moment = runtime
    manager._safe = lambda: True
    navigator.request.complete = False
    manager._start(manager.config['people']['observation_pose'], 'going_observation', '', 2.)
    moment[0] += 3.
    assert manager.poll(False)
    moment[0] += 6.
    assert manager.poll(False)
    assert manager.motion and navigator.leases[-1] == 'suspended'
    assert len(navigator.sent) == 1


def test_home_nav2_success_is_not_arrival_when_precise_pose_is_wrong(runtime):
    manager, navigator, moment = runtime
    manager._safe = lambda: True
    manager._stationary = lambda: True
    manager.home.latched = True
    manager._start(manager.config['home']['pose'], 'going_home', 'home.xml', 10.)
    navigator.complete = True
    assert not manager.poll(False)
    assert manager.state == 'verifying_home'
    assert manager.idle() and manager.state != 'home_arrived'
    moment[0] += 6.
    manager.idle()
    assert manager.state == 'home_failed'
    assert not manager.accepting_tasks


def test_home_arrival_requires_precise_pose_and_stopped_duration(runtime):
    manager, _, moment = runtime
    manager.home.latched = True
    manager.state, manager.deadline = 'verifying_home', 105.
    manager._safe = lambda: True
    manager._stationary = lambda: True
    manager._pose = lambda *args: manager.config['home']['pose']
    manager.idle()
    assert manager.state == 'verifying_home'
    moment[0] += 1.1
    manager.idle()
    assert manager.state == 'home_arrived' and not manager.accepting_tasks


def test_latched_home_priority_wins_over_people_observation(runtime):
    manager, navigator, _ = runtime
    manager._safe = lambda: True
    manager.home.latched = True
    manager.idle()
    assert manager.state == 'going_home'
    assert navigator.sent[-1].behavior_tree.endswith('smart_home_navigate.xml')


def test_scan_clearance_rejects_nearby_objects_unknown_scan_and_stale_data(runtime):
    manager, _, _ = runtime
    manager._safe = lambda: True
    manager._stationary = lambda: True
    manager._stamp_fresh = lambda *args: True
    scan = LaserScan()
    scan.header.frame_id, scan.header.stamp.sec = 'inverted_lidar', 100
    scan.range_min, scan.range_max = .05, 10.
    scan.angle_min, scan.angle_increment = -math.pi, math.pi / 180
    scan.ranges = [2.] * 360
    manager.last_scan = (100., scan)
    assert manager._clear_to_spin()
    scan.ranges[150] = .2
    assert not manager._clear_to_spin()
    scan.ranges = [float('nan')] * 360
    assert not manager._clear_to_spin()
    scan.ranges = [2.] * 360
    manager._stamp_fresh = lambda *args: False
    assert not manager._clear_to_spin()


def test_failed_observation_does_not_invent_a_people_destination(runtime):
    manager, navigator, _ = runtime
    manager._safe = lambda: True
    manager.state, manager.motion, manager.deadline = 'observing', True, 200.
    manager._clear_to_spin = lambda *args: True
    manager.last_scan = (100., LaserScan())
    manager._stamp_fresh = lambda *args: True
    manager.spin_travel = 1.0
    navigator.complete = True
    manager.poll(False)
    assert manager.state == 'idle'
    assert manager.reason == 'insufficient_rotation_coverage'
    assert not navigator.sent


def test_old_home_latch_survives_delivery_restart(runtime):
    manager, navigator, _ = runtime
    manager._save_latch(True)
    restarted = OptionalIdleRuntime(navigator, manager.buffer, manager.config, ROOT,
                                     lambda: True, lambda _: None, lambda: None)
    assert restarted.home_requested and not restarted.accepting_tasks


def test_only_unstarted_orders_are_released_not_paused_current_task():
    source = ROOT / 'smart_delivery_core/smart_delivery.py'
    main = next(node for node in ast.parse(source.read_text()).body
                if isinstance(node, ast.FunctionDef) and node.name == 'main')
    loop = next(node for node in main.body if isinstance(node, ast.While)
                and ast.unparse(node.test) == 'rclpy.ok()')
    branch = next(node for node in loop.body if isinstance(node, ast.If)
                  and ast.unparse(node.test) == 'optional_runtime and optional_runtime.home_requested')
    wrapper = ast.fix_missing_locations(ast.Module(body=[ast.For(
        target=ast.Name(id='_once', ctx=ast.Store()), iter=ast.List(elts=[ast.Constant(1)], ctx=ast.Load()),
        body=[branch], orelse=[])], type_ignores=[]))
    released = []
    paused = {'task_id': 'current', '_resume_state': 'waiting_localization'}
    namespace = dict(optional_runtime=SimpleNamespace(home_requested=True),
                     active_task=paused, pending_orders=[paused, {'task_id': 'later1'}, {'task_id': 'later2'}],
                     DeliveryStateMachine=lambda _: None,
                     queue_result=lambda order, *args: released.append(order['task_id']))
    for _ in range(3):
        exec(compile(wrapper, str(source), 'exec'), namespace)
        namespace['active_task'] = None  # durable ACK of the released task
    assert released == ['later1', 'later2']
    assert namespace['pending_orders'] == [paused]
