import math
from types import SimpleNamespace

from smart_delivery_core.localization_health import (
    amcl_timeout_requires_recovery,
    damped_heading_command,
    heading_correction,
    heading_error_is_improving,
    localization_suspicion_required,
    map_match_status,
    normalize_angle,
    occupancy_match_score,
    pose_is_near,
    pose_jump,
    pose_quality,
    quaternion_to_yaw,
    scan_pipeline_status,
    select_scan_samples,
    sensor_timeout_requires_recovery,
    should_extend_global_recovery,
    smoothed_map_score,
    suspect_requires_recovery,
    transient_sensor_recovery_qualified,
    update_stability_samples,
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


def test_heading_correction_returns_to_reference_across_angle_wrap():
    correction = heading_correction(-math.pi + 0.05, math.pi - 0.05)
    assert math.isclose(correction, 0.1)
    assert heading_correction(0.0, math.radians(2.0), math.radians(3.0)) == 0.0


def test_damped_heading_command_scales_and_caps_correction():
    assert math.isclose(
        damped_heading_command(math.radians(10.0), 0.65, math.radians(20.0)),
        math.radians(6.5),
    )
    assert math.isclose(
        damped_heading_command(math.radians(-40.0), 0.65, math.radians(20.0)),
        math.radians(-20.0),
    )


def test_heading_error_improvement_rejects_sign_flip_and_stall():
    assert heading_error_is_improving(None, math.radians(10.0), math.radians(1.0))
    assert heading_error_is_improving(
        math.radians(10.0), math.radians(7.0), math.radians(1.0)
    )
    assert not heading_error_is_improving(
        math.radians(10.0), math.radians(-4.0), math.radians(1.0)
    )
    assert not heading_error_is_improving(
        math.radians(10.0), math.radians(9.5), math.radians(1.0)
    )


def test_stationary_anchor_detects_cumulative_pose_drift():
    anchor = (0.0, 0.0, 0.0)
    samples = [(0.15, 0.0, 0.0), (0.30, 0.0, 0.0), (0.41, 0.0, 0.0)]
    previous = anchor
    for current in samples:
        distance, _ = pose_jump(previous, current)
        assert distance < 0.40
        previous = current
    distance, _ = pose_jump(anchor, samples[-1])
    assert distance >= 0.40


def test_map_match_status_keeps_degraded_scores_out_of_healthy_state():
    assert map_match_status(0.19, 0.5, 0.35, 0.20, 2.0) == "critical"
    assert map_match_status(0.21, 0.5, 0.35, 0.20, 2.0) == "degraded"
    assert map_match_status(0.349, 0.5, 0.35, 0.20, 2.0) == "degraded"
    assert map_match_status(0.35, 0.5, 0.35, 0.20, 2.0) == "healthy"


def test_map_match_status_rejects_missing_or_stale_scores():
    assert map_match_status(None, 0.0, 0.35, 0.20, 2.0) == "unknown"
    assert map_match_status(0.95, 2.1, 0.35, 0.20, 2.0) == "unknown"


def test_scan_timeout_debounces_one_short_receive_gap():
    assert not sensor_timeout_requires_recovery(2.1, 2.0, 1.0)
    assert sensor_timeout_requires_recovery(3.01, 2.0, 1.0)


def test_scan_pipeline_distinguishes_filter_gap_from_lidar_loss():
    assert scan_pipeline_status(0.1, 0.2, 2.0, 1.0) == "healthy"
    assert scan_pipeline_status(0.1, 3.1, 2.0, 1.0) == "filtered_timeout"
    assert scan_pipeline_status(3.2, 3.1, 2.0, 1.0) == "raw_timeout"


def test_stationary_map_score_drop_needs_independent_evidence():
    assert not localization_suspicion_required(
        "critical", True, False, False, False
    )
    assert localization_suspicion_required(
        "critical", True, False, True, True
    )
    assert localization_suspicion_required(
        "degraded", False, False, False, False
    )
    assert localization_suspicion_required(
        "healthy", False, True, False, False
    )


def test_transient_sensor_recovery_requires_all_fresh_stationary_evidence():
    arguments = {
        "quality_healthy": True,
        "map_match_recovered": True,
        "scan_age": 0.1,
        "scan_timeout": 2.0,
        "odom_age": 0.1,
        "odom_timeout": 1.0,
        "amcl_age": 0.1,
        "amcl_timeout": 8.0,
        "commanded_motion": False,
        "odom_motion": False,
    }
    assert transient_sensor_recovery_qualified(**arguments)
    assert not transient_sensor_recovery_qualified(
        **{**arguments, "odom_age": 1.01}
    )
    assert not transient_sensor_recovery_qualified(
        **{**arguments, "map_match_recovered": False}
    )
    assert not transient_sensor_recovery_qualified(
        **{**arguments, "commanded_motion": True}
    )


def test_amcl_timeout_allows_stationary_nomotion_refresh_grace():
    assert not amcl_timeout_requires_recovery(8.1, 8.0, False, 2.0)
    assert amcl_timeout_requires_recovery(10.01, 8.0, False, 2.0)
    assert amcl_timeout_requires_recovery(8.1, 8.0, True, 2.0)


def test_stability_samples_only_count_each_timestamp_once():
    count, timestamp = update_stability_samples(True, 10.0, 0.0, 0)
    assert (count, timestamp) == (1, 10.0)
    count, timestamp = update_stability_samples(True, 10.0, timestamp, count)
    assert (count, timestamp) == (1, 10.0)
    count, timestamp = update_stability_samples(True, 11.0, timestamp, count)
    assert (count, timestamp) == (2, 11.0)


def test_stability_samples_reset_and_consume_unhealthy_timestamp():
    count, timestamp = update_stability_samples(False, 12.0, 11.0, 4)
    assert (count, timestamp) == (0, 12.0)
    count, timestamp = update_stability_samples(True, 12.0, timestamp, count)
    assert (count, timestamp) == (0, 12.0)


def test_smoothed_map_score_rejects_single_frame_drop():
    assert smoothed_map_score([0.38, 0.37], minimum_samples=3) is None
    score = smoothed_map_score([0.38, 0.37, 0.04, 0.36, 0.39], minimum_samples=3)
    assert math.isclose(score, 0.37)


def test_suspect_hysteresis_band_has_a_maximum_wait():
    assert not suspect_requires_recovery(False, "healthy", False, 6.0, 5.0, 12.0)
    assert suspect_requires_recovery(False, "healthy", False, 12.0, 5.0, 12.0)
    assert suspect_requires_recovery(False, "degraded", False, 5.0, 5.0, 12.0)
    assert not suspect_requires_recovery(True, "healthy", False, 20.0, 5.0, 12.0)


def test_suspect_allows_finite_grace_for_promising_convergence():
    assert not suspect_requires_recovery(
        False, "healthy", False, 12.0, 5.0, 12.0, True, 30.0
    )
    assert not suspect_requires_recovery(
        False, "healthy", False, 29.9, 5.0, 12.0, True, 30.0
    )
    assert suspect_requires_recovery(
        False, "healthy", False, 30.0, 5.0, 12.0, True, 30.0
    )
    assert suspect_requires_recovery(
        False, "degraded", False, 12.0, 5.0, 12.0, False, 30.0
    )
    assert suspect_requires_recovery(
        False, "healthy", True, 12.0, 5.0, 12.0, False, 30.0
    )


def test_global_recovery_extends_only_for_fresh_map_match():
    assert not should_extend_global_recovery(11.9, 12.0, 30.0, True)
    assert should_extend_global_recovery(12.0, 12.0, 30.0, True)
    assert should_extend_global_recovery(29.9, 12.0, 30.0, True)
    assert not should_extend_global_recovery(12.0, 12.0, 30.0, False)
    assert not should_extend_global_recovery(30.0, 12.0, 30.0, True)


def test_scan_selection_ignores_near_people_but_keeps_broad_static_view():
    ranges = [0.4, 1.0, 2.0, 0.5, 3.0, 4.0, 1.5, 2.5]
    selection = select_scan_samples(
        ranges,
        range_min=0.1,
        range_max=12.0,
        ignore_below_range=0.6,
        maximum_samples=8,
        sector_total=4,
    )
    assert [distance for _index, distance in selection.samples] == [
        1.0,
        2.0,
        3.0,
        4.0,
        1.5,
        2.5,
    ]
    assert math.isclose(selection.near_fraction, 0.25)
    assert selection.sector_count == 4


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
