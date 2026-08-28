import math
from types import SimpleNamespace

from smart_delivery_core.localization_health import (
    normalize_angle,
    occupancy_match_score,
    pose_is_near,
    pose_jump,
    pose_quality,
    quaternion_to_yaw,
)


def covariance(xy_variance, yaw_variance):
    values = [0.0] * 36
    values[0] = xy_variance
    values[7] = xy_variance
    values[35] = yaw_variance
    return values


def test_pose_quality_uses_standard_deviation_thresholds():
    quality = pose_quality(covariance(0.15**2, 0.10**2), 0.20, 0.17, 0.50, 0.45)
    assert quality.healthy
    assert not quality.critical
    assert quality.xy_std == 0.15


def test_pose_quality_flags_critical_axis():
    quality = pose_quality(covariance(0.51**2, 0.10**2), 0.20, 0.17, 0.50, 0.45)
    assert not quality.healthy
    assert quality.critical


def test_pose_near_handles_wrapped_yaw():
    assert pose_is_near(0.1, -0.1, -math.pi + 0.05, 0.0, 0.0, math.pi, 0.5, 0.2)
    assert not pose_is_near(0.7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.2)


def test_pose_jump_handles_wrapped_yaw():
    distance, angle = pose_jump((0.0, 0.0, math.pi - 0.05), (0.3, 0.4, -math.pi + 0.05))
    assert distance == 0.5
    assert math.isclose(angle, 0.1)


def test_quaternion_to_yaw():
    yaw = 0.75
    quaternion = SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))
    assert math.isclose(quaternion_to_yaw(quaternion), yaw)


def test_normalize_angle():
    assert math.isclose(normalize_angle(3 * math.pi), -math.pi)


def test_occupancy_match_score_allows_nearby_wall_cells():
    grid = [0] * 25
    grid[2 * 5 + 2] = 100
    score = occupancy_match_score(
        [(2.0, 2.0), (3.0, 2.0), (0.0, 0.0)],
        grid,
        width=5,
        height=5,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        neighborhood_cells=1,
    )
    assert math.isclose(score, 2.0 / 3.0)


def test_occupancy_match_score_handles_rotated_map_origin():
    grid = [0] * 9
    grid[1] = 100
    score = occupancy_match_score(
        [(0.0, 1.0)],
        grid,
        width=3,
        height=3,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=math.pi / 2.0,
        neighborhood_cells=0,
    )
    assert score == 1.0
