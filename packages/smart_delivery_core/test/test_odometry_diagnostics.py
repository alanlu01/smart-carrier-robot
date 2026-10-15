"""Exercise production odometry with fake publishers; no nodes or motor commands."""

from types import SimpleNamespace

from builtin_interfaces.msg import Time
import pytest

from smart_delivery_core import mecanum_odom_real as module
from smart_delivery_core.pipeline_health import DurationMetrics, ReceiptMetrics


@pytest.mark.parametrize('stale', [False, True])
def test_integration_has_no_public_tf_writer_and_preserves_watchdog(monkeypatch, stale):
    clock = [100.0]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    node = object.__new__(module.MecanumOdomReal)
    node.duration_metrics, node.receipt_metrics = DurationMetrics(), ReceiptMetrics()
    node.last_time, node.max_integration_dt_sec = 99.95, .20
    node.feedback_assembler = SimpleNamespace(is_stale=lambda *_: stale, sample_age=lambda _: 1.0)
    node.feedback_state = 'healthy'
    node.local_write_errors = 0
    node.wheel_state_timeout_sec = .80
    node.current_speeds = dict.fromkeys(('FL', 'FR', 'RL', 'RR'), 1.0)
    node.wheel_radius, node.geometry_factor, node.yaw_ratio = .08, .207, 1.0
    node.x = node.y = node.th = 0.0
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        to_msg=lambda: Time(sec=int(clock[0]))))
    node.get_logger = lambda: SimpleNamespace(warning=lambda *_: None)
    odometry = []
    node.odom_pub = SimpleNamespace(publish=odometry.append)
    node.update_odometry()
    assert node.x == pytest.approx(0.0 if stale else .004)
    assert node.y == node.th == 0.0
    assert odometry[0].header.stamp.sec == 100
    assert node.feedback_state == ('stale' if stale else 'healthy')
    result = node.duration_metrics.snapshot(clock[0])
    assert 'tf_publish' not in result and 'odom_publish' not in result
    assert result['odom_callback']['max_sec'] == 0.0
    assert result['local_odom_publish']['max_sec'] == 0.0


def test_health_recomputes_age_instead_of_repeating_last_healthy_callback(monkeypatch):
    import json
    monkeypatch.setattr(module.time,'monotonic',lambda:100.)
    node=object.__new__(module.MecanumOdomReal)
    node.feedback_assembler=SimpleNamespace(sample_age=lambda _:5.,complete_sample_count=10)
    node.feedback_state='healthy';node.wheel_state_timeout_sec=.8
    node.local_write_errors=0
    node.receipt_metrics=ReceiptMetrics();node.duration_metrics=DurationMetrics()
    node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=100000000000))
    msgs=[];node.feedback_health_pub=SimpleNamespace(publish=msgs.append)
    node.publish_feedback_health()
    assert json.loads(msgs[-1].data)['state']=='stale'
