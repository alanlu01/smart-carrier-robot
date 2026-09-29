"""Check passive diagnostics do not blur receive gaps and source latency."""

from smart_delivery_core.pipeline_health import ReceiptMetrics


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
