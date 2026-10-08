"""Exercise real manager methods without starting ROS nodes or moving hardware."""

from collections import deque
from types import SimpleNamespace

from geometry_msgs.msg import PoseWithCovarianceStamped

import pytest

from smart_delivery_core import localization_manager as module
from smart_delivery_core.localization_health import EvidenceWaitClock, pose_quality
from smart_delivery_core.pipeline_health import ReceiptMetrics, ScanPairMetrics


@pytest.fixture
def manager(monkeypatch):
    """Create a deterministic observation harness using the production methods."""
    clock = SimpleNamespace(now=100.0)
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock.now)
    node = object.__new__(module.LocalizationManager)
    node.__dict__.update(
        state='RECOVERING_LOCAL', state_reason='test', ready=False, recovery_step='nomotion_wait',
        state_since=100.0, recovery_step_started=100.0,
        scan_timeout=2.0, scan_timeout_grace=1.0, odom_timeout=1.0,
        amcl_pose_timeout=8.0, map_match_period=1.0,
        latest_raw_scan_at=100.0, latest_scan_at=100.0, latest_odom_at=100.0,
        latest_scan_stamp_ns=0, last_amcl_at=100.0,
        latest_map_score_at=100.0, latest_map_score=0.8,
        latest_map_near_fraction=0.0, latest_map_sector_count=12,
        last_map_match_error='', last_map_match_check_at=100.0,
        map_match_window_min_samples=3, map_score_window=deque([0.8] * 5),
        map_match_min_score=0.30, map_match_recovery_score=0.35,
        map_match_critical_score=0.20,
        stable_samples=0, stable_samples_required=6,
        last_stable_map_score_at=0.0,
        healthy_xy_std=0.25, healthy_yaw_std=0.17,
        critical_xy_std=0.50, critical_yaw_std=0.45,
        latest_pose=(2.0, 5.0, 0.0), verification_pose_anchor=(2.0, 5.0, 0.0),
        verification_pose_distance=0.20, verification_pose_angle=0.174533,
        verify_reference_required=False,
        spin_in_progress=False, spin_waiting_for_stop=False,
        spin_restoring_heading=False, spin_sequence=[],
        commanded_motion=False, odom_motion=False, motion_inhibited=False,
        observation_clock=EvidenceWaitClock(), observation_phase=None,
        convergence_samples=deque(maxlen=10), last_convergence_map_at=0.0,
        convergence_grace_announced=False, verify_timeout=25.0, verify_max_wait=40.0,
        local_max_wait=30.0, receipt_metrics=ReceiptMetrics(),
        tick_last_at=None, tick_max_gap=0.0, tick_max_duration=0.0,
        last_tick_metrics_at=100.0,
        pipeline_clock=EvidenceWaitClock(),
        transient_clock=EvidenceWaitClock(), transient_recovery_eligible=False,
        transient_recovery_started_at=100.0, transient_recovery_max=8.0,
        transient_recovery_samples_required=2, pipeline_wait_timeout=30.0,
        diagnostic_log_period=5.0, last_diagnostic_log_at=0.0,
        amcl_nomotion_refresh=3.0, verification_nomotion_refresh=1.0,
        last_nomotion_request_at=0.0, sensor_waiting_for_pipeline=False,
        local_wait=12.0, global_wait=12.0, global_max_wait=30.0,
        global_convergence_max_wait=40.0,
        scan_pair_metrics=ScanPairMetrics(), lateral_path_reports={}, lateral_self_mask=None,
        enable_small_sweep=True, pre_global_wait=4.0, lateral_phase=None,
        lateral_enabled=False, lateral_attempted=False,
        latest_scan=None, last_pipeline_diagnostic_at=0.0,
        global_grace_announced=False, cancel_grace=2.0,
        suspect_since=100.0, suspect_hold=5.0, suspect_max=12.0,
        suspect_promising_max=30.0,
        last_state_publish_at=100.0,
    )
    cov = [0.0] * 36
    cov[0] = cov[7] = 0.20 ** 2
    cov[35] = 0.10 ** 2
    node.latest_quality = pose_quality(cov, 0.25, 0.17, 0.50, 0.45)
    node.get_logger = lambda: SimpleNamespace(
        info=lambda *_: None, warning=lambda *_: None, error=lambda *_: None
    )
    node.requests = []
    node.manual_reasons = []
    node.started_spins = []
    node._update_map_match_score = lambda *_: None
    node._publish_state = lambda: None

    def request():
        node.last_nomotion_request_at = clock.now
        node.requests.append(clock.now)

    def localized(_reason=None):
        node.state, node.ready = 'LOCALIZED', True

    def manual(reason):
        node.state, node.ready = 'MANUAL_REQUIRED', False
        node.manual_reasons.append(reason)

    node._request_nomotion_update = request
    node._mark_localized = localized
    node._require_manual = manual
    node._start_small_sweep = lambda: node.started_spins.append('small')
    node._start_global_recovery = lambda: node.started_spins.append('global')
    node._begin_recovery = lambda reason, **_: node.started_spins.append(reason)
    return node, clock


def fresh(node, clock, at):
    """Supply one independent scan/map observation and fresh chassis feedback."""
    clock.now = at
    node.latest_raw_scan_at = node.latest_scan_at = node.latest_odom_at = at
    node.latest_map_score_at = at


def amcl(node, xy_std=0.20):
    """Deliver a healthy real ROS pose message to the production callback."""
    msg = PoseWithCovarianceStamped()
    msg.pose.pose.position.x, msg.pose.pose.position.y = 2.0, 5.0
    msg.pose.pose.orientation.w = 1.0
    msg.pose.covariance[0] = msg.pose.covariance[7] = xy_std ** 2
    msg.pose.covariance[35] = 0.10 ** 2
    node._amcl_pose_callback(msg)


def test_suspect_requests_stationary_update_and_recovers_without_spin(manager):
    """Regress the 14:06 case: a high map score must refresh stale AMCL first."""
    node, clock = manager
    node.state = 'SUSPECT'
    node.last_amcl_at = 89.0
    node._tick_state_machine()
    assert node.requests == [100.0]
    assert not node.started_spins
    for at in range(101, 107):
        fresh(node, clock, float(at))
        node.last_amcl_at = float(at)
        node._tick_state_machine()
    assert node.state == 'LOCALIZED'
    assert not node.started_spins


@pytest.mark.parametrize('step', ['nomotion_wait', 'post_small'])
def test_six_amcl_samples_fit_before_local_escalation(manager, step):
    """Keep all six confirmations; do not reset a healthy solution globally."""
    node, clock = manager
    node.recovery_step = step
    for at in range(100, 106):
        fresh(node, clock, float(at))
        node._tick_state_machine()
        amcl(node)
    assert node.state == 'LOCALIZED'
    assert not node.started_spins


def test_same_map_timestamp_cannot_count_as_six_confirmations(manager):
    """Repeated callbacks need independent map observations to count."""
    node, _clock = manager
    for _ in range(8):
        amcl(node)
    assert node.stable_samples == 1
    assert not node.ready


def test_high_map_score_does_not_override_unhealthy_covariance(manager):
    """Fresh matching scans are not permission to accept uncertain localization."""
    node, clock = manager
    node.recovery_step = 'post_small'
    for at in range(100, 113):
        fresh(node, clock, float(at))
        amcl(node, xy_std=0.30)
        node._tick_state_machine()
    assert not node.ready
    assert node.stable_samples == 0
    assert node.started_spins == ['global']


def test_transient_window_pauses_while_slot_operation_is_inhibited(manager):
    """An ongoing take/return must not exhaust the skip-spin deadline."""
    node, clock = manager
    node.transient_recovery_eligible = True
    node.motion_inhibited = True
    for at in [100.0, 104.0, 110.0]:
        fresh(node, clock, at)
        node.last_amcl_at = at
        node._observe_wait(at)
    assert node.transient_clock.elapsed == 0.0
    amcl(node)
    fresh(node, clock, 111.0)
    amcl(node)
    assert node.state == 'LOCALIZED'
    assert not node.started_spins


def test_global_deadline_excludes_filter_and_tf_outage(manager):
    """Regress 14:14: healthy covariance plus a data gap is not localization loss."""
    node, clock = manager
    node.state, node.recovery_step = 'RECOVERING_GLOBAL', 'post_global'
    node._observe_wait(100.0)
    node.observation_clock.elapsed = 11.0
    for at in [103.0, 108.0, 115.0]:
        clock.now = at
        node.latest_raw_scan_at = at
        node.latest_odom_at = at
        node.last_amcl_at = at
        node.last_map_match_error = 'TF extrapolation into future'
        node._tick_state_machine()
    assert node.observation_clock.elapsed == 11.0
    assert not node.manual_reasons
    assert not node.started_spins
    node.last_map_match_error = ''
    fresh(node, clock, 116.0)
    node._tick_state_machine()
    assert node.observation_clock.elapsed == 11.0
    assert not node.manual_reasons


def test_waiting_for_user_is_not_charged_as_a_pipeline_outage(manager):
    """A short data gap after a long slot operation must not immediately fail."""
    node, clock = manager
    node.motion_inhibited = True
    for at in [100.0, 130.0, 140.0]:
        fresh(node, clock, at)
        node.last_amcl_at = at
        node._observe_wait(at)
    clock.now = 144.0
    node._observe_wait(clock.now)
    assert not node.manual_reasons
    assert node.pipeline_clock.blocked_for(clock.now) == 0.0
    assert not node.started_spins


def test_persistent_pipeline_outage_stays_stopped_and_requests_help(manager):
    """Data outages cannot silently stall recovery forever or trigger a spin."""
    node, clock = manager
    clock.now = 104.0
    node._tick_state_machine()
    clock.now = 134.1
    node._tick_state_machine()
    assert node.state == 'MANUAL_REQUIRED'
    assert '資料管線' in node.manual_reasons[0]
    assert not node.started_spins


def test_outage_resets_confirmation_count(manager):
    """Confirmation samples on opposite sides of an outage cannot be combined."""
    node, clock = manager
    node.stable_samples = 5
    clock.now = 104.0
    node._observe_wait(clock.now)
    assert node.stable_samples == 0


def test_localized_nomotion_cadence_is_unchanged(manager):
    """Faster AMCL polling is limited to verification states, not normal service."""
    node, clock = manager
    node.state = 'LOCALIZED'
    node.last_nomotion_request_at = 99.0
    node._request_nomotion_if_due(clock.now)
    assert not node.requests
    clock.now = 102.0
    node._request_nomotion_if_due(clock.now)
    assert node.requests == [102.0]


def test_stale_scan_header_blocks_even_when_receive_time_is_fresh(manager):
    """Do not treat a delayed scan delivery as a new valid observation."""
    node, clock = manager
    node.latest_scan_stamp_ns = 90_000_000_000
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=int(clock.now * 1e9))
    )
    assert 'scan_timestamp_invalid' in node._verification_data_blockers(clock.now)


def test_suspect_reentry_excludes_normal_navigation_time(manager):
    """Regress 15:35: an identical second phase cannot inherit 205 seconds."""
    node, clock = manager
    node._publish_state = lambda: None
    node.state = 'SUSPECT'
    node._observe_wait(100.0)
    fresh(node, clock, 107.0)
    node.last_amcl_at = 107.0
    node._observe_wait(107.0)
    assert node.observation_clock.elapsed == 7.0
    node.ready_publisher = SimpleNamespace(publish=lambda *_: None)
    node._set_state('LOCALIZED', 'recovered')
    fresh(node, clock, 305.0)
    node.last_amcl_at = 305.0
    node._set_state('SUSPECT', 'new episode')
    node._observe_wait(305.0)
    assert node.observation_clock.elapsed == 0.0
    assert node.pipeline_clock.blocked_for(305.0) == 0.0
    assert not node.convergence_samples or len(node.convergence_samples) == 1


def test_verifying_keeps_four_healthy_samples_past_soft_deadline(manager):
    """A converging manual pose gets bounded time to finish all six samples."""
    node, clock = manager
    node.state = 'VERIFYING'
    node.recovery_step = None
    node.initial_pose_last_published = 99.0
    node._observe_wait(100.0)
    node.observation_clock.elapsed = 25.0
    node.stable_samples = 4
    node._tick_state_machine()
    assert not node.started_spins
    assert not node.ready
    fresh(node, clock, 101.0)
    amcl(node)
    fresh(node, clock, 102.0)
    amcl(node)
    assert node.ready


def test_verifying_grace_has_hard_deadline(manager):
    """Map agreement and a few samples cannot extend verification forever."""
    node, _clock = manager
    node.state, node.recovery_step = 'VERIFYING', None
    node.initial_pose_last_published = 99.0
    node._observe_wait(100.0)
    node.observation_clock.elapsed = 40.0
    node.stable_samples = 4
    node._tick_state_machine()
    assert node.started_spins == ['初始位置驗證逾時']
    assert not node.ready


def test_local_convergence_trend_delays_but_never_skips_health_gate(manager):
    """Stationary improving covariance waits locally rather than resetting globally."""
    node, clock = manager
    node.recovery_step = 'post_small'
    node._observe_wait(100.0)
    node.observation_clock.elapsed = 12.0
    node.convergence_samples.extend([(7.0, 0.40), (9.0, 0.36), (11.0, 0.30)])
    amcl(node, xy_std=0.30)
    node._tick_state_machine()
    assert not node.started_spins
    assert not node.ready
    node.observation_clock.elapsed = 30.0
    node._tick_state_machine()
    assert node.started_spins == ['global']


@pytest.mark.parametrize('fault', ['bad_map', 'critical', 'moving', 'stale'])
def test_convergence_grace_rejects_unsafe_evidence(manager, fault):
    """A safe-looking picture is not sufficient to defer real failures."""
    node, clock = manager
    node.observation_clock.elapsed = 25.0
    node.stable_samples = 4
    if fault == 'bad_map':
        node.map_score_window = deque([0.1] * 5)
        node.latest_map_score = 0.1
    elif fault == 'critical':
        amcl(node, xy_std=0.6)
    elif fault == 'moving':
        node.odom_motion = True
    else:
        clock.now = 110.0
    assert not node._convergence_grace(clock.now, 25.0, 40.0)


def test_disabled_small_sweep_goes_directly_global_after_brief_wait(manager):
    node, clock = manager
    node.enable_small_sweep = False
    node.auto_motion_recovery = True
    node.latest_map_score = 0.1
    node.observation_clock.elapsed = 4.0
    node._tick_local_recovery(clock.now)
    assert node.started_spins == ['global']


def test_brief_wait_still_allows_six_healthy_samples_to_finish(manager):
    node, clock = manager
    node.enable_small_sweep = False
    node.auto_motion_recovery = True
    for at in range(100, 106):
        fresh(node, clock, float(at))
        amcl(node)
        if not node.ready:
            node._tick_state_machine()
    assert node.ready
    assert not node.started_spins


def lateral_harness(manager):
    """Wire real manager motion methods to fake sensors and a fake fusion ACK."""
    import json
    from hailo_vision.recovery_lease import RecoveryLease
    node, clock = manager
    node.lateral_enabled = node.auto_motion_recovery = True
    node.lateral_attempted = False
    node.lateral_distance, node.lateral_cap = .30, .25
    node.last_odom_pose = (0.0, 0.0, 0.0)
    node.latest_cmd_at = clock.now
    node.lateral_guard, node.lateral_guard_at = {}, 0.0
    node._cancel_navigation = lambda: None
    node._lateral_sensors_ready = lambda _now: True
    node._lateral_path_clear = lambda *_: True
    node._reset_map_match_history = lambda: None
    node.lateral_commands = []
    node.recovery_cmd_pub = SimpleNamespace(publish=node.lateral_commands.append)
    lease = RecoveryLease()

    def lease_publish(message):
        lease.receive(json.loads(message.data), clock.now)
        node.lateral_guard = {'token': lease.token, 'active': lease.enabled,
                              'blocked': lease.blocked(clock.now)}
        node.lateral_guard_at = clock.now

    node.recovery_lease_pub = SimpleNamespace(publish=lease_publish)
    return node, clock, lease


def test_one_sidestep_stops_then_runs_second_global_recovery(manager):
    node, clock, lease = lateral_harness(manager)
    assert node._try_lateral_recovery(clock.now)
    for _ in range(5):
        clock.now += .05
        node._tick_lateral_motion()
    assert node.lateral_commands[-1].linear.y == .25
    for distance in [.025 * step for step in range(1, 12)] + [.29]:
        clock.now += .10
        node.last_odom_pose = (0.0, distance, 0.0)
        node._tick_lateral_motion()
    assert node.lateral_phase == 'stopping'
    assert lease.blocked(clock.now)
    assert node.lateral_commands[-1].linear.y == 0.0
    for _ in range(25):
        clock.now += .05
        node.latest_cmd_at = clock.now
        node._tick_lateral_motion()
    assert node.lateral_phase is None
    assert not lease.enabled
    assert node.started_spins == ['global']
    assert not node._try_lateral_recovery(clock.now)  # never a second translation


@pytest.mark.parametrize('fault', ['scan', 'obstacle', 'guard', 'inhibit', 'yaw', 'timeout'])
def test_lateral_fault_always_outputs_zero_and_disarms_only_after_stop(manager, fault):
    node, clock, lease = lateral_harness(manager)
    node._try_lateral_recovery(clock.now)
    for _ in range(5):
        clock.now += .05
        node._tick_lateral_motion()
    if fault in {'scan', 'inhibit'}:
        node._lateral_sensors_ready = lambda _: False
    elif fault == 'obstacle':
        node._lateral_path_clear = lambda *_: False
    elif fault == 'guard':
        node.lateral_guard_at = 0.0
        node.recovery_lease_pub = SimpleNamespace(publish=lambda _: None)
    elif fault == 'yaw':
        node.last_odom_pose = (0.0, .1, .2)
    else:
        clock.now += 9.0
    node._tick_lateral_motion()
    assert node.lateral_phase == 'stopping'
    assert node.lateral_abort_reason
    assert node.lateral_commands[-1].linear.y == 0.0
    assert not node.ready
    assert not node.started_spins


def test_amcl_cannot_accept_a_pose_during_translation(manager):
    node, _clock = manager
    node.recovery_step = 'lateral_move'
    node.stable_samples = 5
    amcl(node)
    assert not node.ready
    assert node.stable_samples == 0


def test_odom_jump_cannot_count_as_successful_translation(manager):
    node, clock, _lease = lateral_harness(manager)
    node._try_lateral_recovery(clock.now)
    for _ in range(5):
        clock.now += .05
        node._tick_lateral_motion()
    clock.now += .05
    node.last_odom_pose = (0, .29, 0)
    node._tick_lateral_motion()
    assert node.lateral_phase == 'stopping'
    assert '跳躍' in node.lateral_abort_reason


def test_no_feedback_during_stop_requests_help_but_keeps_zero_lease(manager):
    node, clock, lease = lateral_harness(manager)
    node._try_lateral_recovery(clock.now)
    node._stop_lateral('test sensor loss')
    node._lateral_sensors_ready = lambda _: False
    clock.now += 3.1
    node._tick_lateral_motion()
    assert node.state == 'MANUAL_REQUIRED'
    assert node.lateral_phase == 'stopping'
    assert lease.blocked(clock.now)
    assert node.lateral_commands[-1].linear.y == 0.0


def test_post_global_two_of_six_finishes_during_bounded_grace(manager):
    node, clock = manager
    node.state, node.recovery_step = 'RECOVERING_GLOBAL', 'post_global'
    node._observe_wait(100.0)
    node.observation_clock.elapsed = 30.0
    node.stable_samples = 2
    node._tick_state_machine()
    assert not node.manual_reasons and not node.ready
    assert node.convergence_grace_announced
    for at in range(101, 105):
        fresh(node, clock, float(at))
        amcl(node)
        if not node.ready:
            node._tick_state_machine()
    assert node.ready
    assert not node.started_spins and not node.manual_reasons


def test_post_global_grace_expires_and_only_then_attempts_lateral(manager):
    node, clock = manager
    node.state, node.recovery_step = 'RECOVERING_GLOBAL', 'post_global'
    node.observation_clock.elapsed = 39.9
    node.stable_samples = 2
    attempts = []
    node._try_lateral_recovery = lambda now: attempts.append(now) or True
    node._tick_global_recovery(clock.now)
    assert not attempts and not node.ready
    node.observation_clock.elapsed = 40.0
    node._tick_global_recovery(clock.now)
    assert attempts == [100.0]
    assert not node.ready


@pytest.mark.parametrize('fault', ['bad_map', 'critical', 'moving', 'stale', 'pose_jump'])
def test_post_global_does_not_extend_unsafe_solution(manager, fault):
    node, clock = manager
    node.state, node.recovery_step = 'RECOVERING_GLOBAL', 'post_global'
    node.observation_clock.elapsed = 30.0
    node.stable_samples = 2
    if fault == 'bad_map':
        node.map_score_window = deque([.1] * 5)
        node.latest_map_score = .1
    elif fault == 'critical':
        amcl(node, xy_std=.6)
    elif fault == 'moving':
        node.odom_motion = True
    elif fault == 'pose_jump':
        node.latest_pose = (3, 5, 0)
    else:
        clock.now = 110.0
    node._tick_global_recovery(clock.now)
    assert node.manual_reasons and not node.ready


@pytest.mark.parametrize('fault', [None, 'obstacle', 'tilt', 'stale_tf'])
def test_lateral_manager_checks_both_scans_and_live_tf(manager, fault):
    import math
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import LaserScan
    node, clock = manager
    node.lateral_half_x, node.lateral_half_y = .22, .16
    node.lateral_self_mask = (-.20, .20, -.125, .125, -.5, .5)
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=int(clock.now * 1e9))
    )
    transform = TransformStamped()
    transform.header.stamp.sec = 98 if fault == 'stale_tf' else 100
    transform.transform.rotation.w = 1.0
    sensor = TransformStamped()
    sensor.transform.translation.x = .15
    sensor.transform.translation.y = -.025
    sensor.transform.translation.z = .425
    sensor.transform.rotation.z = 1.0
    sensor.transform.rotation.w = 0.0
    if fault == 'tilt':
        sensor.transform.rotation.x = .01
    node.tf_buffer = SimpleNamespace(lookup_transform=lambda target, *_:
                                    transform if target == 'odom' else sensor)
    raw = LaserScan()
    raw.header.frame_id = 'laser'
    raw.angle_min, raw.angle_increment = -math.pi, math.pi / 360
    raw.range_min, raw.range_max = .05, 8.0
    raw.ranges = [3.0] * 720
    index = round(math.atan2(.067, -.149) / raw.angle_increment)
    raw.ranges[index] = .163
    filtered = LaserScan()
    filtered.header = raw.header
    filtered.angle_min, filtered.angle_increment = raw.angle_min, raw.angle_increment
    filtered.range_min, filtered.range_max = raw.range_min, raw.range_max
    filtered.ranges = list(raw.ranges)
    filtered.ranges[index] = float('nan')
    if fault == 'obstacle':
        filtered.ranges[180] = .35
    node.latest_raw_scan, node.latest_scan = raw, filtered
    assert node._lateral_path_clear(1, .30) == (fault is None), node.lateral_path_reports
    if fault in (None, 'obstacle'):
        assert set(node.lateral_path_reports['1']) == {'scan', 'scan_filtered'}
        assert node.lateral_path_reports['1']['scan']['masked_beams'] == 1
