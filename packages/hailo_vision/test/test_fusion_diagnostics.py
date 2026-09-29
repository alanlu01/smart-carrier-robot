"""Call the actual fusion watchdog without spinning ROS or moving hardware."""

from types import SimpleNamespace

from hailo_vision import fusion_node as module
from hailo_vision.pipeline_metrics import PipelineMetrics


def fusion(monkeypatch, age):
    """Build a non-ROS harness with the same safety thresholds as production."""
    monkeypatch.setattr(module.time, 'monotonic', lambda: 103.0)
    node = object.__new__(module.SensorFusionNode)
    node.pipeline_metrics = PipelineMetrics(100.0)
    node.last_pipeline_report_at = 100.0
    node.semantic_sequence = 10
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
