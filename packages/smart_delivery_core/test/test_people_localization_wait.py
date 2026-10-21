"""No actual nodes/actions: bounded SUSPECT pause and frame-safe resumption."""

import json

import pytest
from nav2_simple_commander.robot_navigator import TaskResult
from std_msgs.msg import String

from test_optional_idle_runtime import runtime
from test_people_retry import begin_spin, prepare


def paused_spin(runtime):
    manager, navigator, moment = begin_spin(runtime)
    manager.observation_alignment = (0., 0., 0.)
    manager._map_alignment = lambda: (0., 0., 0.)
    manager._safe = lambda: False
    manager._can_wait_localization = lambda: True
    manager._localization_phase = lambda: 'SUSPECT'
    assert manager.poll(False) and navigator.cancel_calls == 1
    assert manager.interrupt_reason == 'localization_suspect'
    navigator.complete, navigator.result = True, TaskResult.CANCELED
    assert manager.poll(False)
    assert manager.state == 'waiting_people_localization'
    assert len(navigator.sent) == 1
    manager.poll(False)
    moment[0] += 1.1
    assert not manager.poll(False)  # complete the existing continuous stop proof
    return manager, navigator, moment


def recover(manager):
    manager._safe = manager._stationary = lambda: True
    manager._stamp_fresh = lambda *args: True
    manager._localization_phase = lambda: 'LOCALIZED'
    manager._spin_start_ready = lambda: True


def test_localization_pause_never_moves_until_ready_and_new_scan_permission(runtime):
    manager, navigator, moment = paused_spin(runtime)
    moment[0] += 7.
    assert manager.poll(False) and len(navigator.sent) == 1
    recover(manager)
    assert manager.poll(False) and manager.state == 'waiting_observation_retry'
    assert len(navigator.sent) == 1 and manager.spin_travel == .6
    assert manager.idle() and len(navigator.sent) == 2
    assert manager.state == 'observing' and manager.spin_resume_requests == 1
    manager._people_finished('test_cleanup')


def test_global_frame_correction_discards_old_counts_and_restarts_within_original_budget(runtime):
    manager, navigator, moment = paused_spin(runtime)
    old = manager.people
    recover(manager)
    manager._map_alignment = lambda: (.15, 0., .2)
    manager.poll(False)
    assert manager.people is not old and manager.spin_travel == 0.
    assert manager.localization_restarts == 1 and manager.observation_session_deadline == 190.
    assert manager.people.received_frames == 0
    manager.idle()
    assert len(navigator.sent) == 2 and manager.deadline <= 190.
    manager._people_finished('test_cleanup')


@pytest.mark.parametrize('phase', ['MANUAL_REQUIRED', 'RECOVERING_GLOBAL', 'VERIFYING', 'WAITING_FOR_SENSORS'])
def test_escalated_recovery_drops_auxiliary_work_without_any_resume(runtime, phase):
    manager, navigator, _ = paused_spin(runtime)
    recover(manager)
    manager._localization_phase = lambda: phase
    assert manager.poll(False)
    assert manager.state == 'idle' and phase in manager.reason and len(navigator.sent) == 1


def test_wait_expiration_cannot_be_extended_by_new_status_messages(runtime):
    manager, navigator, moment = paused_spin(runtime)
    original = manager.localization_wait_deadline
    moment[0] = original+.01
    assert manager.poll(False)
    assert manager.state == 'idle' and manager.reason == 'people_localization_wait_timeout'
    assert len(navigator.sent) == 1


def test_order_preempts_wait_without_resuming_observation(runtime):
    manager, navigator, _ = paused_spin(runtime)
    assert not manager.poll(True)
    assert manager.state == 'idle' and len(navigator.sent) == 1


def test_navigation_pause_resumes_same_point_with_original_navigation_deadline(runtime):
    manager, navigator, moment = prepare(runtime)
    manager._start(manager.config['people']['observation_pose'], 'going_observation', '', 180.)
    manager._safe = lambda: False
    manager._can_wait_localization = lambda: True
    manager._localization_phase = lambda: 'SUSPECT'
    manager.poll(False)
    navigator.complete, navigator.result = True, TaskResult.CANCELED
    manager.poll(False)
    manager.poll(False)
    moment[0] += 1.1
    manager.poll(False)
    recover(manager)
    moment[0] += 8.
    assert manager.poll(False) and manager.state == 'going_observation'
    assert len(navigator.sent) == 2 and manager.deadline == pytest.approx(280.)
    assert manager.localization_pauses == 1


def test_pause_cap_and_existing_spin_resume_cap_are_not_reset(runtime):
    manager, navigator, _ = paused_spin(runtime)
    recover(manager)
    manager.spin_resumes = 2
    manager.poll(False)
    manager.idle()
    assert manager.state == 'idle' and manager.reason == 'spin_resume_budget_exhausted'
    assert len(navigator.sent) == 1


def test_status_must_be_fresh_and_suspect_and_not_override_other_safety_sources(runtime):
    manager, _, moment = prepare(runtime)
    manager.ready = lambda: False
    manager._localization(String(data=json.dumps({'sample_ros_stamp_ns':100_000_000_000,'state':'SUSPECT'})))
    assert manager._can_wait_localization()
    manager.inhibited = True
    assert not manager._can_wait_localization()
    manager.inhibited = False
    moment[0] += 2.1
    assert not manager._can_wait_localization()


def test_vision_during_pause_is_not_added_to_old_directional_counts(runtime):
    manager, _, _ = paused_spin(runtime)
    manager.people = type('People', (), {'observe': lambda *_: pytest.fail('paused vision accepted')})()
    manager._vision(String(data='{}'))


def test_wait_timeout_is_not_suspended_when_external_recovery_is_moving(runtime):
    manager, navigator, moment = paused_spin(runtime)
    manager.settling_after_cancel = True
    manager._stationary = lambda: False
    moment[0] = manager.localization_wait_deadline+.1
    assert manager.poll(False)
    assert manager.state == 'idle' and manager.settling_after_cancel
    assert len(navigator.sent) == 1


def test_ready_without_fresh_filtered_scan_cannot_resume(runtime):
    manager, navigator, _ = paused_spin(runtime)
    recover(manager)
    manager._stamp_fresh = lambda *args: False
    assert manager.poll(False) and manager.state == 'waiting_people_localization'
    assert len(navigator.sent) == 1


def test_repeated_alignment_changes_exit_instead_of_restarting_forever(runtime):
    manager, navigator, _ = paused_spin(runtime)
    recover(manager)
    manager.localization_restarts = 1
    manager._map_alignment = lambda: (1., 0., 0.)
    assert manager.poll(False)
    assert manager.state == 'idle' and manager.reason == 'people_localization_repeated_alignment_change'
    assert len(navigator.sent) == 1


def test_two_localization_pauses_are_the_maximum_per_observation(runtime):
    manager, navigator, _ = begin_spin(runtime)
    manager.localization_pauses = 2
    manager._safe = lambda: False
    manager._can_wait_localization = lambda: True
    manager.poll(False)
    navigator.complete, navigator.result = True, TaskResult.CANCELED
    manager.poll(False)
    assert manager.state == 'idle' and manager.localization_pauses == 2


def test_unrelated_global_recovery_does_not_keep_old_pending_spin_evidence(runtime):
    manager, navigator, _ = begin_spin(runtime)
    manager.motion = False
    manager.state = 'waiting_observation_retry'
    manager._safe = lambda: False
    manager._can_wait_localization = lambda: False
    manager._localization_phase = lambda: 'RECOVERING_GLOBAL'
    assert manager.poll(False)
    assert manager.state == 'idle' and not manager.observation_collecting
    assert manager.settling_after_cancel and len(navigator.sent) == 1
