"""Passive timing and covariance diagnostics must never grant motion permission."""

import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from smart_delivery_core.localization_health import PoseQuality
from smart_delivery_core.localization_quality_history import LocalizationQualityHistory
from smart_delivery_core.scan_pipeline_metrics import ScanPipelineMetrics


def test_covariance_history_uses_independent_stamps_and_preserves_critical_truth():
    history = LocalizationQualityHistory()
    healthy = PoseQuality(.15, .05, True, False)
    critical = PoseQuality(.76, .12, False, True)
    assert history.observe(1, 100., (0., 0., 0.), healthy, .8)
    assert history.observe(2, 100.2, (0., 0., 0.), critical, .8)
    assert history.snapshot(100.2)['pattern'] == 'critical'
    assert not history.observe(2, 100.3, (0., 0., 0.), healthy, .8)
    history.observe(3, 100.5, (0., 0., 0.), healthy, .8)
    report = history.snapshot(100.5)
    assert report['last_burst']['pattern'] == 'short_critical_burst'
    assert report['critical_samples_3s'] == 1 and report['diagnostic_only']
    assert history.snapshot(103.)['pattern'] == 'awaiting_update'
    assert history.snapshot(109.)['pattern'] == 'stale'


def test_persistent_uncertainty_and_pose_jump_are_not_labelled_single_spike():
    history = LocalizationQualityHistory()
    critical = PoseQuality(.76, .12, False, True)
    for i in range(10):
        history.observe(i+1, 100.+i*.3, (0., 0., 0.), critical, .2)
    history.observe(11, 103., (1., 0., .4), PoseQuality(.2, .05, True, False), .8)
    report = history.snapshot(103.)
    assert report['last_burst']['pattern'] == 'persistent_critical_burst'
    assert report['last_pose_step_m'] == 1. and report['last_pose_step_rad'] == .4


def test_diagnostics_memory_is_bounded_and_invalid_covariance_is_rejected():
    history = LocalizationQualityHistory()
    quality = PoseQuality(.2, .1, True, False)
    for i in range(100):
        history.observe(i+1, i*.1, (0.,0.,0.), quality)
    assert len(history.samples) == 32
    assert not history.observe(101, 11., (float('nan'),0.,0.), quality)


def test_missing_amcl_updates_are_not_invented_as_persistent_critical_evidence():
    history = LocalizationQualityHistory()
    history.observe(1, 100., (0.,0.,0.), PoseQuality(.76,.1,False,True))
    history.observe(2, 103., (0.,0.,0.), PoseQuality(.2,.1,True,False))
    assert history.snapshot(103.)['last_burst']['pattern'] == 'insufficient_temporal_coverage'


def test_scan_pair_separates_writer_path_transport_and_executor_delay():
    metrics = ScanPipelineMetrics()
    metrics.observe('raw', 100_000_000_000, 100., 100_100_000_000,
                    100_080_000_000, 100_090_000_000)
    metrics.observe('filtered', 100_000_000_000, 102., 102_100_000_000,
                    100_090_000_000, 100_100_000_000)
    report = metrics.snapshot(102.)
    assert report['timings']['pair.callback_skew_sec']['last_sec'] == 2.
    assert report['timings']['pair.writer_pipeline_sec']['last_sec'] == .01
    assert report['last']['filtered']['rmw_to_callback_sec'] == 2.
    assert report['last']['filtered']['writer_to_rmw_sec'] == .01


def test_delayed_writer_does_not_become_two_seconds_of_observer_queue_time():
    metrics = ScanPipelineMetrics()
    metrics.observe('raw', 100_000_000_000, 100., 100_100_000_000, 100_080_000_000)
    metrics.observe('filtered', 100_000_000_000, 102., 102_100_000_000, 102_090_000_000)
    report = metrics.snapshot(102.)
    assert report['timings']['pair.writer_pipeline_sec']['last_sec'] == 2.01
    assert report['last']['filtered']['writer_to_callback_sec'] == .01
    assert report['last']['filtered']['rmw_to_callback_sec'] is None


def test_missing_or_wrong_clock_domain_dds_timestamps_are_explicitly_unavailable():
    metrics = ScanPipelineMetrics()
    metrics.observe('raw', 100_000_000_000, 100., 100_100_000_000)
    metrics.observe('filtered', 100_000_000_000, 100.01, 100_110_000_000, 1)
    report = metrics.snapshot(100.01)
    assert report['last']['raw']['dds_source_timestamp_available'] is False
    assert 'pair.writer_pipeline_sec' not in report['timings']
    assert report['matched_pairs'] == 1


def test_queued_old_scan_is_not_healthy_just_because_its_callback_is_new():
    metrics = ScanPipelineMetrics()
    metrics.observe('raw', 102_000_000_000, 102., 102_100_000_000)
    metrics.observe('filtered', 100_000_000_000, 102.01, 102_110_000_000)
    assert metrics.snapshot(102.01)['state'] == 'filtered_source_stale'


def test_scan_observer_out_of_order_pairs_and_bounded_memory():
    metrics = ScanPipelineMetrics()
    metrics.observe('filtered', 1, 100., 100_000_000_000)
    metrics.observe('raw', 1, 100.01, 100_010_000_000)
    assert metrics.snapshot(100.01)['timings']['pair.callback_skew_sec']['last_sec'] == -.01
    for i in range(2, 200):
        metrics.observe('raw', i, 100., 100_000_000_000)
    assert len(metrics.pending) == 128
    assert metrics.snapshot(101.)['state'] != 'healthy'


def test_observer_publishes_only_diagnostic_topic_not_any_control_or_scan():
    path = Path(__file__).parents[1]/'smart_delivery_core/scan_pipeline_monitor.py'
    tree = ast.parse(path.read_text())
    publishers = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                  and isinstance(n.func, ast.Attribute) and n.func.attr == 'create_publisher']
    assert len(publishers) == 1 and publishers[0].args[1].value == '/lidar/pipeline_health'


@pytest.mark.parametrize('info', [
    {'source_timestamp':100_080_000_000,'received_timestamp':100_090_000_000},
    SimpleNamespace(source_timestamp=100_080_000_000,received_timestamp=100_090_000_000),
])
def test_jazzy_message_info_dict_and_attribute_adapters_both_retain_dds_time(info):
    from sensor_msgs.msg import LaserScan
    from smart_delivery_core.scan_pipeline_monitor import ScanPipelineMonitor
    node = object.__new__(ScanPipelineMonitor)
    node.metrics = ScanPipelineMetrics()
    node.get_clock = lambda: SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=100_100_000_000))
    scan = LaserScan()
    scan.header.stamp.sec = 100
    node._observe('filtered', scan, info)
    report = node.metrics.snapshot(0.)['last']['filtered']
    assert report['dds_source_timestamp_available'] and report['rmw_receive_timestamp_available']
    assert report['writer_to_rmw_sec'] == .01
