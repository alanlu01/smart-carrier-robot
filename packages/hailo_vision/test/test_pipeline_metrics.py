"""Exercise timing diagnostics without importing Hailo or changing safety."""

from hailo_vision.pipeline_metrics import PipelineMetrics


def test_interval_reset_keeps_receive_edges_to_find_next_outage():
    metrics = PipelineMetrics(100.0)
    metrics.observe('image', 100.0)
    metrics.observe('image', 100.1)
    metrics.measure('inference_sec', 0.04)
    first = metrics.snapshot(101.0)
    assert first['counts']['image'] == 2
    assert first['max_sec']['inference_sec'] == 0.04
    metrics.observe('image', 104.1)
    second = metrics.snapshot(105.0)
    assert second['counts']['image'] == 1
    assert second['max_sec']['image_gap_sec'] == 4.0
    assert 'inference_sec' not in second['max_sec']
    assert second['receive_age_sec']['image'] == 0.9
