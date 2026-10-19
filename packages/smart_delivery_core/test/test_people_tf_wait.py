"""Bounded future-TF pairing and stopped resumption; no robot nodes/actions."""
import math
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import TransformStamped
from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, ExtrapolationException, LookupException

from test_optional_idle_runtime import ROOT, runtime
from test_people_retry import begin_spin, prepare
from smart_delivery_core.optional_idle import load_idle_config


def scan_at(seconds):
    scan = LaserScan()
    scan.header.frame_id = 'laser'
    scan.header.stamp.sec = int(seconds)
    scan.header.stamp.nanosec = round((seconds-int(seconds))*1_000_000_000)
    scan.angle_min, scan.angle_increment = -math.pi, 2*math.pi/360
    scan.range_min, scan.range_max = .05, 10.
    scan.ranges = [3.]*360
    return scan


def pending_runtime(runtime):
    manager, navigator, moment = prepare(runtime)
    manager.state = 'waiting_observation_stop'
    navigator.get_clock = lambda: SimpleNamespace(
        now=lambda: Time(nanoseconds=round(moment[0]*1_000_000_000)))
    transform = TransformStamped()
    transform.transform.rotation.w = 1.
    available = [False]
    requested = []
    def lookup(parent, child, stamp):
        requested.append(stamp.nanoseconds)
        if not available[0]:
            raise ExtrapolationException('Lookup would require extrapolation into the future.')
        return transform
    manager.buffer = SimpleNamespace(lookup_transform=lookup)
    return manager, navigator, moment, available, requested


def test_future_tf_never_grants_permission_until_exact_stamp_arrives(runtime):
    manager, _, moment, available, requested = pending_runtime(runtime)
    scan = scan_at(100.)
    manager._scan(scan)
    assert manager.pending_scan and not manager.scan_history
    assert not manager._clear_to_spin()
    assert manager.guard_report['reason'] == 'laser_tf_pending'
    moment[0] += .06
    manager._retry_scan_transform()
    assert manager.pending_scan and not manager.scan_history
    available[0] = True
    moment[0] += .04
    manager._retry_scan_transform()
    assert manager.pending_scan is None and not manager.scan_evidence_error
    assert len(manager.scan_history) == 1
    assert set(requested) == {100_000_000_000}  # no latest-TF fallback
    evidence = manager.scan_history[-1]
    assert evidence.source_sec == 100. and evidence.received_at == 100.
    assert scan.header.stamp.sec == 100 and scan.header.stamp.nanosec == 0
    manager._retry_scan_transform()
    assert len(manager.scan_history) == 1
    assert not manager._clear_to_spin()  # one frame is still insufficient


def test_real_tf_buffer_reproduces_59ms_gap_and_interpolates_later(runtime):
    manager, _, moment, _, _ = pending_runtime(runtime)
    buffer = Buffer()
    mount = TransformStamped()
    mount.header.frame_id, mount.child_frame_id = 'base_footprint', 'laser'
    mount.transform.rotation.z = 1.  # actual inverted mounting, yaw=pi
    mount.transform.rotation.w = 0.
    buffer.set_transform_static(mount, 'test')
    def odom_at(seconds, x):
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'odom', 'base_footprint'
        transform.header.stamp = Time(nanoseconds=round(seconds*1e9)).to_msg()
        transform.transform.translation.x = x
        transform.transform.rotation.w = 1.
        buffer.set_transform(transform, 'test')
    # A running robot has a time history. With only one TF, tf2 emits an
    # ambiguous "only time ..." error; that stays a fail-closed hard error.
    odom_at(99.891, 0.)
    odom_at(99.941, 0.)
    manager.buffer = buffer
    manager._scan(scan_at(100.))
    assert manager.scan_evidence_reason == 'laser_tf_pending'
    assert manager.pending_scan and not manager.scan_history
    moment[0] = 100.08
    odom_at(100.041, .01)
    manager._retry_scan_transform()
    assert manager.pending_scan is None and not manager.scan_evidence_error
    evidence = manager.scan_history[-1]
    assert evidence.source_sec == 100. and evidence.received_at == 100.
    assert evidence.base_pose[0] == pytest.approx(.0059, abs=1e-8)
    assert abs(evidence.sensor_pose[2]) == pytest.approx(math.pi)


def test_pair_wait_has_a_hard_200ms_limit_even_if_tf_later_arrives(runtime):
    manager, _, moment, available, requested = pending_runtime(runtime)
    manager._scan(scan_at(100.))
    calls = len(requested)
    moment[0] += .201
    available[0] = True
    manager._retry_scan_transform()
    assert manager.pending_scan is None and not manager.scan_history
    assert manager.scan_evidence_reason == 'laser_tf_wait_timeout'
    assert len(requested) == calls
    assert not manager._clear_to_spin()


def test_original_source_freshness_is_not_extended_by_tf_wait(runtime):
    manager, _, moment, available, _ = pending_runtime(runtime)
    manager._scan(scan_at(99.75))
    moment[0] += .10
    available[0] = True
    manager._retry_scan_transform()
    assert manager.pending_scan is None and not manager.scan_history
    assert manager.scan_evidence_reason == 'scan_stale_or_absent'


def test_newest_pending_scan_replaces_old_candidate_without_restamping(runtime):
    manager, _, moment, available, _ = pending_runtime(runtime)
    manager._scan(scan_at(100.))
    moment[0] = 100.05
    newer = scan_at(100.05)
    manager._scan(newer)
    moment[0] = 100.12
    available[0] = True
    manager._retry_scan_transform()
    assert [e.source_sec for e in manager.scan_history] == [100.05]
    assert manager.scan_history[-1].received_at == 100.05
    assert newer.header.stamp.nanosec == 50_000_000


@pytest.mark.parametrize('exception', [
    ExtrapolationException('Lookup would require extrapolation into the past.'),
    ExtrapolationException('Lookup would require extrapolation at time 100, but only time 99.941 is in the buffer'),
    LookupException('laser frame does not exist'),
    ValueError('unsupported laser/base tilt or invalid quaternion'),
])
def test_hard_transform_errors_are_not_mislabeled_transient(runtime, exception):
    manager, _, _, _, _ = pending_runtime(runtime)
    def lookup(*args):
        raise exception
    manager.buffer.lookup_transform = lookup
    manager._scan(scan_at(100.))
    assert manager.pending_scan is None
    assert manager.scan_evidence_reason == 'laser_tf_unavailable'
    assert not manager._clear_to_spin()


def test_exit_discards_pending_tf_and_resets_safety_duration(runtime):
    manager, _, _, _, _ = pending_runtime(runtime)
    manager._scan(scan_at(100.))
    manager.safe_scan_since = 90.
    manager.safe_stop_since = 90.
    manager.safe_scan_count = 50
    manager._people_finished('nav2_failed')
    manager._retry_scan_transform()
    assert manager.pending_scan is None and not manager.scan_history
    assert manager.safe_scan_since is None and manager.safe_stop_since is None
    assert manager.safe_scan_count == 0
    assert manager.spin_ready_blocker == 'not_waiting'


@pytest.mark.parametrize('reason', ['near_obstacle','recent_near_obstacle',
                                  'laser_tf_pending','laser_tf_wait_timeout'])
def test_transient_stop_cancels_and_drains_before_bounded_retry(runtime, reason):
    manager, navigator, moment = begin_spin(runtime)
    manager.guard_report['reason'] = reason
    manager.poll(False)
    assert navigator.cancel_calls == 1 and manager.motion
    assert len(navigator.sent) == 1
    manager.idle()
    assert len(navigator.sent) == 1
    navigator.complete, navigator.result = True, TaskResult.CANCELED
    manager.poll(False)
    assert manager.state == 'waiting_observation_retry'
    assert manager.settling_after_cancel
    manager.idle()
    assert len(navigator.sent) == 1
    manager.poll(False)
    moment[0] += 1.1
    manager.poll(False)
    assert not manager.settling_after_cancel
    manager._spin_start_ready = lambda: False
    manager.idle()
    assert len(navigator.sent) == 1  # still blocked, not authorized by elapsed time
    manager._spin_start_ready = lambda: True
    manager.idle()
    assert len(navigator.sent) == 2 and manager.spin_resume_requests == 1
    assert manager.spin_resumes == 0
    manager._people_finished('test_cleanup')


@pytest.mark.parametrize('reason', ['near_obstacle','laser_tf_pending'])
@pytest.mark.parametrize('cap', ['session','requests','actual'])
def test_new_retry_reasons_cannot_bypass_existing_caps(runtime, reason, cap):
    manager, navigator, _ = begin_spin(runtime)
    manager.guard_report['reason'] = reason
    if cap == 'session':
        manager.observation_session_deadline = 99.
    elif cap == 'requests':
        manager.spin_resume_requests = 4
    else:
        manager.spin_resumes = 2
    manager.poll(False)
    navigator.complete, navigator.result = True, TaskResult.CANCELED
    manager.poll(False)
    assert manager.state == 'idle' and len(navigator.sent) == 1


def test_persistent_obstacle_waits_only_ten_seconds_then_exits(runtime):
    manager, navigator, moment = begin_spin(runtime)
    manager.guard_report['reason'] = 'near_obstacle'
    manager.poll(False)
    navigator.complete, navigator.result = True, TaskResult.CANCELED
    manager.poll(False)
    manager.settling_after_cancel = False
    manager._spin_start_ready = lambda: False
    moment[0] = manager.deadline+.01
    manager.idle()
    assert manager.state == 'idle'
    assert manager.reason.startswith('spin_ready_timeout:')
    assert len(navigator.sent) == 1


def test_service_or_localization_loss_still_preempts_retry(runtime):
    manager, navigator, _ = prepare(runtime)
    manager.state = 'waiting_observation_retry'
    manager.observation_session_deadline, manager.deadline = 190., 110.
    manager._safe = lambda: False
    manager.idle()
    assert manager.state == 'idle' and not navigator.sent
    manager.state = 'waiting_observation_retry'
    manager._safe = lambda: True
    manager.poll(True)
    assert manager.state == 'idle' and not navigator.sent


def test_timeout_identifies_continuous_safety_not_just_latest_clear(runtime):
    manager, navigator, moment = prepare(runtime)
    manager.state, manager.deadline = 'waiting_observation_stop', 100.05
    manager._clear_to_spin = lambda *_: True
    manager.guard_report = {'clear': True, 'reason': 'clear'}
    manager.safe_stop_since, manager.safe_scan_since = 98., 99.9
    manager.safe_scan_count = 3
    assert not manager._spin_start_ready()
    assert manager.spin_ready_blocker == 'continuous_safe_duration_insufficient'
    moment[0] = 100.1
    manager.idle()
    assert manager.reason == 'spin_ready_timeout:continuous_safe_duration_insufficient'
    assert not navigator.sent


@pytest.mark.parametrize('state', ['observing','idle','people_destination_selected'])
def test_safe_wait_duration_is_zero_outside_waiting_states(runtime, state):
    manager, _, _ = prepare(runtime)
    captured = []
    manager.status_pub.publish = lambda m: captured.append(m.data)
    manager.state, manager.safe_scan_since = state, 90.
    manager._heartbeat()
    import json
    assert json.loads(captured[-1])['continuous_safe_scan_sec'] == 0.


def test_formal_point_changed_but_all_safety_budgets_preserved():
    config = load_idle_config(ROOT/'config/optional_idle_features.yaml')
    people = config['people']
    assert (people['observation_pose']['x'], people['observation_pose']['y']) == (.5, 3.5)
    assert people['max_unknown_arc_deg'] == 15.
    assert people['safe_scan_samples'] == 3 and people['stopped_before_spin_sec'] == 1.
    assert people['spin_ready_wait_sec'] == 10.
    assert people['max_spin_resumes'] == 2 and people['observation_session_timeout_sec'] == 90.
    assert not config['home']['enabled']
