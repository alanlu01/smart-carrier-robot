"""Call the actual fusion watchdog without spinning ROS or moving hardware."""

from types import SimpleNamespace
from geometry_msgs.msg import Twist

from hailo_vision import fusion_node as module
from hailo_vision.pipeline_metrics import PipelineMetrics
from hailo_vision.recovery_lease import LocalizationSpeedLimit, RecoveryLease
from smart_delivery_core.wheel_feedback import WheelStopEvidence


def fusion(monkeypatch, age):
    """Build a non-ROS harness with the same safety thresholds as production."""
    monkeypatch.setattr(module.time, 'monotonic', lambda: 103.0)
    node = object.__new__(module.SensorFusionNode)
    node.pipeline_metrics = PipelineMetrics(100.0)
    node.last_pipeline_report_at = 100.0
    node.semantic_sequence = 10
    node.recovery_lease = RecoveryLease()
    node.recovery_stop = WheelStopEvidence(100.0)
    node.recovery_odom_at = node.recovery_odom_stamp_ns = None
    node.recovery_odom_stopped = False
    node.recovery_last_nonzero_cmd_at = 100.0
    node.recovery_handoff = False
    node.recovery_last_release = {}
    node.recovery_action_status = {}
    node.recovery_cancel_pending = {}
    node.recovery_cancel_confirmed = {}
    node.recovery_cancel_probe_at = 0.
    node.recovery_cancel_clients = {}
    node.localization_speed_limit = LocalizationSpeedLimit()
    node.speed_multiplier = 1.0
    node.semantic_health_state = 'healthy'
    node.latest_semantic_received_at = 100.0
    node.semantic_timeout_sec = 0.8
    node.semantic_recovery_timeout_sec = 0.4
    node.semantic_hard_stop_timeout_sec = 2.0
    node.semantic_stale_speed_multiplier = 0.5
    node.semantic_age = lambda _now=None: age
    node.get_logger = lambda: SimpleNamespace(
        info=lambda *_: None, warning=lambda *_: None, error=lambda *_: None
    )
    node.health_messages, node.commands = [], []
    node.pipeline_pub = SimpleNamespace(publish=node.health_messages.append)
    node.pub_safe_cmd = SimpleNamespace(publish=node.commands.append)
    node.recovery_guard_pub = SimpleNamespace(publish=lambda _: None)
    return node


def test_diagnostics_preserve_hard_stop_on_stale_semantics(monkeypatch):
    node = fusion(monkeypatch, age=2.1)
    node.semantic_watchdog_callback()
    assert len(node.health_messages) == 1
    assert node.semantic_health_state == 'stopped'
    assert len(node.commands) == 1
    assert node.commands[0].linear.x == 0.0
    assert node.semantic_speed_limit() == 0.0


def test_localization_soft_limit_combines_by_minimum_and_expiry_stops(monkeypatch):
    node=fusion(monkeypatch,age=.1)
    node.localization_speed_limit.observe(.5,103.)
    command=Twist()
    command.linear.x=.5
    node.cmd_vel_callback(command)
    assert node.commands[-1].linear.x==.25
    node.speed_multiplier=.5
    node.cmd_vel_callback(command)
    assert node.commands[-1].linear.x==.25  # no accidental quarter-speed multiplication
    node.localization_speed_limit.received_at=101.
    node.cmd_vel_callback(command)
    assert node.commands[-1].linear.x==0.


def test_known_localization_producer_loss_flushes_zero_even_without_new_commands(monkeypatch):
    node=fusion(monkeypatch,age=.1)
    node.localization_speed_limit.observe(1.,101.)
    node.semantic_watchdog_callback()
    assert node.commands[-1].linear.x==0.


def test_diagnostics_preserve_soft_timeout_and_healthy_recovery(monkeypatch):
    node = fusion(monkeypatch, age=0.9)
    node.semantic_watchdog_callback()
    assert node.semantic_health_state == 'stale'
    assert node.semantic_speed_limit() == 0.5
    assert not node.commands
    node.semantic_age = lambda _now=None: 0.2
    node.semantic_watchdog_callback()
    assert node.semantic_health_state == 'healthy'
    assert node.semantic_speed_limit() == 1.0


def test_recovery_command_preserves_semantic_reduction_and_forbids_other_motion(monkeypatch):
    node = fusion(monkeypatch, age=0.1)
    node.speed_multiplier = 0.5
    node.recovery_lease.receive({'token': 'a', 'active': True, 'permit': True}, 103.0)
    command = Twist()
    command.linear.x, command.linear.y, command.angular.z = 1.0, 1.0, 1.0
    node.cmd_vel_callback(command)
    assert node.commands[-1].linear.x == node.commands[-1].angular.z == 0.0
    assert node.commands[-1].linear.y == 0.125
    node.speed_multiplier = 0.0
    node.cmd_vel_callback(command)
    assert node.commands[-1].linear.y == 0.0


def test_watchdog_stops_expired_lease_even_if_semantic_state_did_not_change(monkeypatch):
    node = fusion(monkeypatch, age=0.1)
    node.recovery_lease.receive({'token': 'a', 'active': True, 'permit': True}, 102.7)
    node.semantic_watchdog_callback()
    assert node.commands[-1].linear.y == 0.0
    assert node.recovery_lease.blocked(103.0)
    command = Twist()
    command.linear.y = 0.25
    node.cmd_vel_callback(command)
    assert node.commands[-1].linear.y == 0.0


def stop_harness(monkeypatch):
    from nav_msgs.msg import Odometry
    node = fusion(monkeypatch, age=.1)
    clock = SimpleNamespace(now=103.)
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock.now)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(clock.now*1e9)))
    node.get_publishers_info_by_topic = lambda _: [SimpleNamespace(endpoint_gid=(1,))]

    def physical_stop(at):
        clock.now = at
        odom = Odometry()
        odom.header.stamp.sec = int(at)
        odom.header.stamp.nanosec = round((at-int(at))*1e9)
        node.recovery_odom_callback(odom)
        for letter in 'ABCD':
            node.recovery_feedback_callback(SimpleNamespace(data=letter+'_RT: 0'))

    return node, clock, physical_stop


def lease_packet(node, **payload):
    import json
    node.recovery_lease_callback(SimpleNamespace(data=json.dumps(payload)))


def test_fusion_requires_independent_real_stop_before_matching_release(monkeypatch):
    node, clock, stopped = stop_harness(monkeypatch)
    lease_packet(node, token='a', owner='m', active=True, permit=False)
    lease_packet(node, token='a', owner='m', active=False)
    assert node.recovery_lease.enabled
    for at in (103.5, 104., 104.5):
        stopped(at)
    lease_packet(node, token='a', owner='m', active=False)
    assert not node.recovery_lease.enabled
    assert node.recovery_last_release == {'released_token': 'a', 'released_owner': 'm'}


def test_handoff_needs_idle_actions_from_current_server_generation(monkeypatch):
    node, clock, stopped = stop_harness(monkeypatch)
    lease_packet(node, token='a', owner='old', active=True, permit=True)
    clock.now += .4
    lease_packet(node, token='a', owner='new', active=True, permit=False, handoff=True)
    assert node.recovery_lease.owner == 'new'
    assert node.recovery_lease.blocked(clock.now)
    for at in (104., 104.5, 105.):
        stopped(at)
    assert any('not_confirmed_idle' in x for x in node.recovery_disarm_blockers(105.))
    for name in ('navigate_to_pose', 'navigate_through_poses', 'spin'):
        node.recovery_status_callback(name)(SimpleNamespace(status_list=[]),
                                            {'source_timestamp': 1})
        node.recovery_cancel_confirmed[name] = ((1,), 105.)
    assert not node.recovery_disarm_blockers(105.)
    node.recovery_action_status['spin'] = (True, (1,))
    lease_packet(node, token='a', owner='new', active=False)
    assert node.recovery_lease.enabled
    node.recovery_action_status['spin'] = (False, (1,))
    node.recovery_cancel_confirmed['spin'] = ((2,), 105.)
    assert 'spin_not_confirmed_idle' in node.recovery_disarm_blockers(105.)
    node.recovery_action_status['spin'] = (False, (1,))
    node.recovery_cancel_confirmed['spin'] = ((1,), 105.)
    lease_packet(node, token='a', owner='new', active=False)
    assert not node.recovery_lease.enabled


def test_stale_or_moving_odom_requires_new_stop_evidence(monkeypatch):
    node, clock, stopped = stop_harness(monkeypatch)
    lease_packet(node, token='a', owner='m', active=True, permit=False)
    for at in (103.5, 104., 104.5):
        stopped(at)
    clock.now = 104.8
    lease_packet(node, token='a', owner='m', active=False)
    assert node.recovery_lease.enabled and node.recovery_stop.zero_groups == 0
    stopped(104.9)
    assert node.recovery_disarm_blockers(104.9)


def test_live_upstream_command_prevents_disarm_even_with_zero_wheel_reports(monkeypatch):
    node, clock, stopped = stop_harness(monkeypatch)
    lease_packet(node, token='a', owner='m', active=True, permit=False)
    for at in (103.5, 104., 104.5):
        stopped(at)
    command = Twist()
    command.linear.x = .2
    node.cmd_vel_callback(command)
    lease_packet(node, token='a', owner='m', active=False)
    assert node.recovery_lease.enabled
    assert node.commands[-1].linear.x == 0.


def test_diagnostics_show_lease_stop_even_when_semantics_and_localization_healthy(monkeypatch):
    import json
    node = fusion(monkeypatch, age=.1)
    node.recovery_lease.receive({'token': 'a', 'active': True, 'permit': False}, 103.)
    node.semantic_watchdog_callback()
    payload = json.loads(node.health_messages[-1].data)
    assert payload['recovery_guard_blocked']
    assert payload['effective_speed_multiplier'] == 0.
    assert node.recovery_lease.blocked(103.0)
    command = Twist()
    command.linear.y = 0.25
    node.cmd_vel_callback(command)
    assert node.commands[-1].linear.y == 0.0


def test_unused_action_without_status_requires_fresh_successful_cancel_probe(monkeypatch):
    from rclpy.task import Future
    node, clock, stopped = stop_harness(monkeypatch)
    node.recovery_handoff = True
    node.recovery_lease.owner = 'new'
    def call(_request):
        future = Future()
        future.set_result(SimpleNamespace(return_code=0, goals_canceling=[]))
        return future
    node.recovery_cancel_clients = {
        name: SimpleNamespace(service_is_ready=lambda: True, call_async=call)
        for name in ('navigate_to_pose', 'navigate_through_poses', 'spin')}
    node.recovery_handoff_probe(clock.now)
    assert len(node.recovery_cancel_confirmed) == 3
    assert node.recovery_action_status == {}  # genuinely unused action servers
    for at in (103., 103.5, 104.):
        stopped(at)
    assert not node.recovery_disarm_blockers(clock.now)
    clock.now = 105.1
    assert any('not_confirmed_idle' in x for x in node.recovery_disarm_blockers(clock.now))


def test_canceling_goal_or_cancel_error_cannot_confirm_idle(monkeypatch):
    from rclpy.task import Future
    node, clock, _ = stop_harness(monkeypatch)
    node.recovery_handoff = True
    node.recovery_lease.owner = 'new'
    for response in (SimpleNamespace(return_code=0, goals_canceling=[object()]),
                     SimpleNamespace(return_code=1, goals_canceling=[])):
        future = Future()
        future.set_result(response)
        node.recovery_cancel_pending['spin'] = (future, clock.now)
        node.recovery_cancel_callback('spin', (1,), 'new')(future)
        assert 'spin' not in node.recovery_cancel_confirmed


def test_status_callback_uses_actual_jazzy_dict_and_rejects_reordered_messages(monkeypatch):
    node, _, _ = stop_harness(monkeypatch)
    callback = node.recovery_status_callback('spin')
    callback(SimpleNamespace(status_list=[SimpleNamespace(status=3)]), {'source_timestamp': 20})
    callback(SimpleNamespace(status_list=[]), {'source_timestamp': 10})
    assert node.recovery_action_status['spin'][0]
    callback(SimpleNamespace(status_list=[]), {'source_timestamp': 30})
    assert not node.recovery_action_status['spin'][0]
