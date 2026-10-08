"""Check passive diagnostics do not blur receive gaps and source latency."""

import pytest

from smart_delivery_core import pipeline_health
from smart_delivery_core.pipeline_health import DurationMetrics, ReceiptMetrics, ScanPairMetrics


def test_receive_peak_retains_its_age_and_source_delay():
    metrics = ReceiptMetrics()
    metrics.observe('scan', 100.0, 0.05)
    metrics.observe('scan', 103.0, 2.9)
    metrics.observe('scan', 103.1, 0.04)
    snapshot = metrics.snapshot(105.0)['scan']
    assert snapshot['count'] == 3
    assert snapshot['max_gap_sec'] == 3.0
    assert snapshot['peak_age_sec'] == 2.0
    assert snapshot['source_age_at_receive_sec'] == 0.04
    assert snapshot['receive_age_sec'] == 1.9


def test_duration_separates_blocked_publish_from_callback_receipt_gap(monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(pipeline_health.time, 'monotonic', lambda: clock[0])
    metrics = DurationMetrics()
    with metrics.measure('odom_callback'):
        with metrics.measure('tf_publish'):
            clock[0] += 2.0
        with metrics.measure('odom_publish'):
            clock[0] += .001
    result = metrics.snapshot(105.0)
    assert result['tf_publish']['max_sec'] == 2.0
    assert result['tf_publish']['over_50ms_count'] == 1
    assert result['odom_publish']['max_sec'] == .001
    assert result['odom_callback']['max_sec'] == 2.001
    assert result['tf_publish']['peak_age_sec'] == 3.0


def test_failed_operation_is_measured_and_exception_is_not_swallowed():
    metrics = DurationMetrics()
    with pytest.raises(RuntimeError):
        with metrics.measure('publish'):
            raise RuntimeError('test failure')
    assert metrics.operations['publish']['count'] == 1


def test_scan_pair_skew_handles_both_callback_orders_and_bounds_memory():
    metrics = ScanPairMetrics(capacity=2)
    metrics.observe('scan', 1, 100.0)
    metrics.observe('scan_filtered', 1, 102.0)
    assert metrics.snapshot(103.0)['last_filtered_minus_raw_callback_sec'] == 2.0
    metrics.observe('scan_filtered', 2, 103.0)
    metrics.observe('scan', 2, 103.01)
    result = metrics.snapshot(104.0)
    assert result['last_filtered_minus_raw_callback_sec'] == -.01
    assert result['max_absolute_callback_skew_sec'] == 2.0
    assert result['matched_count'] == 2
    for stamp in range(3, 20):
        metrics.observe('scan', stamp, 104.0)
    assert len(metrics.pending) == 2
    assert metrics.evicted == 15
    metrics.observe('scan', 0, 105.0)
    assert len(metrics.pending) == 2
