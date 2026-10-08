"""Deterministic geometry tests; no ROS nodes or hardware are started."""

import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from smart_delivery_core.recovery_translation import (
    lateral_progress, lateral_speed, load_self_mask, swept_scan_clear, swept_scan_report,
)


def scan(value=3.0):
    return SimpleNamespace(ranges=[value] * 720, angle_min=-math.pi,
                           angle_increment=math.pi / 360, range_min=0.05, range_max=8.0)


@pytest.mark.parametrize('direction', [-1, 1])
@pytest.mark.parametrize('laser_yaw', [0.0, math.pi])
def test_full_swept_footprint_clear_with_reversed_laser(direction, laser_yaw):
    assert swept_scan_clear(scan(), (0.0, 0.0, laser_yaw), direction, .30, .22, .16)


@pytest.mark.parametrize('value', [0.35, float('nan'), float('inf')])
def test_obstacles_unknown_and_infinite_returns_refuse_motion(value):
    assert not swept_scan_clear(scan(value), (0, 0, math.pi), 1, .30, .22, .16)


def test_left_obstacle_does_not_prevent_safe_right_candidate():
    data = scan()
    data.ranges[540] = .35  # +90 degrees in physical base coordinates
    assert not swept_scan_clear(data, (0, 0, 0), 1, .30, .22, .16)
    assert swept_scan_clear(data, (0, 0, 0), -1, .30, .22, .16)


def test_odometry_progress_uses_original_heading_not_map():
    progress, cross, angle = lateral_progress((1, 2, math.pi / 2), (.7, 2, math.pi / 2), 1)
    assert progress == pytest.approx(.3)
    assert cross == pytest.approx(0)
    assert angle == pytest.approx(0)


def test_speed_cap_and_braking_target():
    assert lateral_speed(0) == .25
    assert 0 < lateral_speed(.27) < .25
    assert lateral_speed(.29) == 0
    assert lateral_speed(.31) == 0


def mask():
    return load_self_mask(
        Path(__file__).parents[1] / 'config' / 'my_laser_filter.yaml', .22, .16
    )


def body_echo_scan():
    data = scan()
    # Backwards laser at (.15,-.025); one return at approximately (.001,.042).
    angle = math.atan2(.067, -.149) - math.pi
    index = round((angle - data.angle_min) / data.angle_increment)
    data.ranges[index] = .163
    return data


def test_recorded_self_echo_is_a_small_unknown_hole_not_an_external_obstacle():
    data = body_echo_scan()
    pose = (.15, -.025, math.pi)
    assert not swept_scan_clear(data, pose, 1, .30, .22, .16)
    report = swept_scan_report(data, pose, 1, .30, .22, .16, self_mask=mask(), sensor_z=.425)
    assert report['clear']
    assert report['masked_beams'] == 1
    assert .95 <= report['valid_fraction'] < 1


def test_mask_does_not_hide_external_obstacle_or_wrong_height():
    data = scan()
    data.ranges[540] = .35
    report = swept_scan_report(data, (0, 0, 0), 1, .30, .22, .16, self_mask=mask())
    assert not report['clear']
    assert report['reason'] == 'obstacle_or_occlusion'
    assert report['masked_beams'] == 0
    assert not swept_scan_clear(body_echo_scan(), (.15, -.025, math.pi), 1,
                                .30, .22, .16, self_mask=mask(), sensor_z=.6)


def test_many_body_echoes_are_unknown_and_still_refuse_translation():
    data = scan(.10)
    report = swept_scan_report(data, (0, 0, 0), 1, .30, .22, .16, self_mask=mask())
    assert report['masked_beams'] > 10
    assert not report['clear']
    assert report['reason'] == 'unknown_coverage'
    assert report['missing_arc_deg'] > 4


def test_eleven_degree_unknown_wedge_still_blocks_with_mask():
    data = scan()
    for index in range(530, 553):
        data.ranges[index] = float('nan')
    report = swept_scan_report(data, (0, 0, 0), 1, .30, .22, .16, self_mask=mask())
    assert not report['clear']
    assert report['missing_arc_deg'] > 11


def test_self_mask_cannot_extend_beyond_recovery_footprint():
    with pytest.raises(ValueError, match='inside'):
        load_self_mask(Path(__file__).parents[1] / 'config' / 'my_laser_filter.yaml', .1, .1)


@pytest.mark.parametrize('mutate', ['invert', 'frame', 'nan', 'type', 'chain', 'syntax'])
def test_unverified_self_masks_are_rejected(tmp_path, mutate):
    import yaml
    path = Path(__file__).parents[1] / 'config' / 'my_laser_filter.yaml'
    config = yaml.safe_load(path.read_text())
    chain = config['scan_to_scan_filter_chain']['ros__parameters']
    entry = chain['filter1']
    if mutate == 'invert':
        entry['params']['invert'] = True
    elif mutate == 'frame':
        entry['params']['box_frame'] = 'laser'
    elif mutate == 'nan':
        entry['params']['max_x'] = float('nan')
    elif mutate == 'type':
        entry['type'] = 'other'
    elif mutate == 'chain':
        chain['filters'] = ['filter1', 'filter2']
    test_path = tmp_path / 'mask.yaml'
    test_path.write_text('invalid: [' if mutate == 'syntax' else yaml.safe_dump(config))
    with pytest.raises(ValueError):
        load_self_mask(test_path, .22, .16)
