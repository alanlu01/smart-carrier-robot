"""Deterministic geometry tests; no ROS nodes or hardware are started."""

import math
from types import SimpleNamespace

import pytest

from smart_delivery_core.recovery_translation import (
    lateral_progress, lateral_speed, swept_scan_clear,
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
