"""Call the actual fusion watchdog without spinning ROS or moving hardware."""

from types import SimpleNamespace
from geometry_msgs.msg import Twist

from hailo_vision import fusion_node as module
from hailo_vision.pipeline_metrics import PipelineMetrics
from hailo_vision.recovery_lease import RecoveryLease


def fusion(monkeypatch, age):
    """Build a non-ROS harness with the same safety thresholds as production."""
    monkeypatch.setattr(module.time, 'monotonic', lambda: 103.0)
    node = object.__new__(module.SensorFusionNode)
    node.pipeline_metrics = PipelineMetrics(100.0)
    node.last_pipeline_report_at = 100.0
    node.semantic_sequence = 10
    node.recovery_lease = RecoveryLease()
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
