"""Separate scan success from bounded parking; fake actions, no robot motion."""
import json
import math
from types import SimpleNamespace

import pytest
from nav2_simple_commander.robot_navigator import TaskResult

from test_optional_idle_runtime import runtime
from test_people_retry import prepare


def completed_analysis(runtime):
    manager, navigator, moment = prepare(runtime)
    selected = dict(manager.config['people']['standby_points'][0])
    manager.people = SimpleNamespace(select=lambda _: selected,headings=set(range(24)),count=24)
    manager.spin_travel = 2*math.pi
    manager._finish_people_scan()
    return manager, navigator, moment, selected


def start_parking(manager, navigator, moment):
    before = len(navigator.sent)
    manager.idle()
    assert len(navigator.sent) == before  # independent stopped-time proof
    moment[0] += 2.1
    manager.idle()
    assert manager.state == 'going_people_standby'
    assert len(navigator.sent) == before+1
    navigator.complete = False


def finish_navigation(manager, navigator):
    navigator.complete, navigator.result = True, TaskResult.SUCCEEDED
    manager.poll(False)
    assert manager.state == 'people_arrival_check'


def test_completed_scan_immediately_grants_success_cooldown(runtime):
    manager, navigator, moment, selected = completed_analysis(runtime)
    assert manager.analysis_completed and manager.analysis_completed_at == 100.
    assert manager.next_people_at == 700. and manager.people_failures == 0
    assert manager.selected_standby == selected and manager.parking_outcome == 'selected'
    assert manager.parked_point is None and not navigator.sent


@pytest.mark.parametrize('yaw_error,xy_error', [(.12,.035),(.0,.11)])
def test_scan_success_survives_strict_arrival_failure_and_same_point_correction(runtime,yaw_error,xy_error):
    manager, navigator, moment, selected = completed_analysis(runtime)
    manager._pose = lambda *_: dict(selected, x=selected['x']+xy_error,
                                     yaw=selected['yaw']+yaw_error)
    start_parking(manager,navigator,moment)
    finish_navigation(manager,navigator)
    moment[0] += 5.1
    manager.idle()
    assert manager.state == 'people_parking_retry_wait'
    assert manager.analysis_completed and manager.next_people_at == 700.
    assert manager.people_failures == 0 and manager.parked_point is None
    assert manager.parking_last_failure == 'standby_arrival_not_verified'
    start_parking(manager,navigator,moment)
    assert manager.parking_attempts == 2 and manager.parking_outcome == 'correcting'
    for goal in navigator.sent:
        assert goal.pose.pose.position.x == selected['x']
        assert goal.pose.pose.position.y == selected['y']
        assert goal.pose.pose.orientation.w == math.cos(selected['yaw']/2)
    assert manager.deadline-moment[0] == pytest.approx(30.)
    manager._pose = lambda *_: dict(selected)
    finish_navigation(manager,navigator)
    manager.idle()
    assert manager.parked_point is None
    moment[0] += 1.1
    manager.idle()
    assert manager.state == 'idle' and manager.parking_outcome == 'parked'
    assert manager.parked_point == selected and manager.selected_standby == selected
    assert manager.next_people_at == 700.  # no reset to now+30 or now+600


def test_retry_budget_exhaustion_holds_without_false_arrival_or_fourth_goal(runtime):
    manager, navigator, moment, selected = completed_analysis(runtime)
    manager._pose = lambda *_: dict(selected,yaw=selected['yaw']+.12)
    for attempt in range(1,4):
        manager.idle()
        moment[0] = max(moment[0]+1.1,manager.parking_retry.retry_at+.1)
        manager.idle()
        assert manager.parking_attempts == attempt
        finish_navigation(manager,navigator)
        moment[0] += 5.1
        manager.idle()
    assert manager.state == 'people_parking_blocked'
    assert manager.parking_outcome == 'failed' and manager.parked_point is None
    assert manager.analysis_completed and manager.people_failures == 0
    assert manager.next_people_at == 700.
    for _ in range(8):
        moment[0] += 100.
        assert manager.idle()  # do not fall into ordinary parking or a new scan
    assert len(navigator.sent) == 3
    assert not any('people_observe' in goal.behavior_tree for goal in navigator.sent)
    manager.poll(True)
    assert manager.state == 'idle' and manager.parking_outcome == 'preempted'


def test_manual_alignment_can_verify_blocked_parking_without_another_goal(runtime):
    manager, navigator, moment, selected = completed_analysis(runtime)
    manager.parking_outcome = 'failed'
    manager._people_finished('selected_standby_parking_failed:test',
                             terminal_state='people_parking_blocked')
    manager._pose = lambda *_: dict(selected)
    manager.idle()
    assert manager.parked_point is None
    moment[0] += 1.1
    manager.idle()
    assert manager.parked_point == selected and not navigator.sent
    assert manager.parking_outcome == 'parked' and manager.next_people_at == 700.


def test_stopped_arrival_must_remain_in_tolerance_for_one_second(runtime):
    manager, navigator, moment, selected = completed_analysis(runtime)
    manager._pose = lambda *_: dict(selected)
    start_parking(manager,navigator,moment)
    finish_navigation(manager,navigator)
    manager.idle()
    moment[0] += .6
    manager._pose = lambda *_: dict(selected,yaw=selected['yaw']+.11)
    manager.idle()
    assert manager.parking_stop_since is None
    manager._pose = lambda *_: dict(selected)
    manager.idle()
    moment[0] += .5
    manager.idle()
    assert manager.parked_point is None
    moment[0] += .6
    manager.idle()
    assert manager.parked_point == selected


@pytest.mark.parametrize('stage',['people_destination_selected','people_parking_retry_wait',
                                'people_arrival_check','people_parking_blocked'])
def test_new_service_preempts_all_stopped_parking_stages(runtime,stage):
    manager, navigator, _, _ = completed_analysis(runtime)
    manager.state = stage
    manager.poll(True)
    assert manager.state == 'idle' and manager.parking_outcome == 'preempted'
    assert manager.analysis_completed and manager.next_people_at == 700.
    assert manager.parked_point is None and not navigator.sent


def test_service_preempts_inflight_parking_but_drains_cancel_first(runtime):
    manager, navigator, moment, _ = completed_analysis(runtime)
    start_parking(manager,navigator,moment)
    manager.poll(True)
    assert manager.motion and navigator.cancel_calls == 1
    manager.idle()
    assert len(navigator.sent) == 1
    navigator.complete,navigator.result = True,TaskResult.CANCELED
    assert manager.poll(True)
    assert manager.state == 'idle' and manager.parking_outcome == 'preempted'
    assert manager.settling_after_cancel and manager.next_people_at == 700.
    manager.poll(True)
    moment[0] += 1.1
    assert not manager.poll(True)
    assert len(navigator.sent) == 1


def test_localization_loss_waits_without_starting_a_parking_correction(runtime):
    manager,navigator,moment,_ = completed_analysis(runtime)
    manager.state = 'people_parking_retry_wait'
    manager._safe = lambda: False
    manager.idle()
    assert manager.state == 'waiting_people_parking_recovery' and manager.parking_outcome == 'waiting_recovery'
    assert manager.next_people_at == 700. and not navigator.sent


def test_localization_loss_during_parking_cancels_before_waiting(runtime):
    manager,navigator,moment,_ = completed_analysis(runtime)
    start_parking(manager,navigator,moment)
    manager._safe = lambda: False
    manager.poll(False)
    navigator.complete,navigator.result = True,TaskResult.CANCELED
    manager.poll(False)
    assert manager.state == 'waiting_people_parking_recovery' and manager.parking_outcome == 'waiting_recovery'
    assert manager.parking_retry.failures == 0 and manager.next_people_at == 700.


def test_goal_rejection_can_retry_without_losing_analysis_or_replacing_target(runtime):
    manager,navigator,moment,selected = completed_analysis(runtime)
    start_parking(manager,navigator,moment)
    navigator.handle.accepted = False
    manager.poll(False)
    assert manager.state == 'people_parking_retry_wait' and not manager.motion
    assert manager.parking_retry.failures == 1 and manager.selected_standby == selected
    assert manager.next_people_at == 700.


def test_aborted_nav_retries_only_after_feedback_proves_stopped(runtime):
    manager,navigator,moment,_ = completed_analysis(runtime)
    start_parking(manager,navigator,moment)
    navigator.complete,navigator.result = True,TaskResult.FAILED
    manager.poll(False)
    assert manager.state == 'people_parking_retry_wait' and manager.settling_after_cancel
    manager._stationary = lambda: False
    moment[0] += 10.
    manager.poll(False)
    manager.idle()
    assert len(navigator.sent) == 1
    manager._stationary = lambda: True
    manager.poll(False)
    moment[0] += 1.1
    manager.poll(False)
    assert not manager.settling_after_cancel


def test_missing_server_wait_is_bounded_even_without_sending_a_goal(runtime):
    manager,navigator,moment,_ = completed_analysis(runtime)
    navigator.nav_to_pose_client.server_is_ready = lambda: False
    manager.idle()
    moment[0] += 1.1
    manager.idle()
    assert manager.state == 'people_destination_selected' and not navigator.sent
    moment[0] = manager.parking_deadline+.1
    manager.idle()
    assert manager.state == 'people_parking_blocked' and manager.parking_outcome == 'failed'
    assert manager.next_people_at == 700. and not navigator.sent


def test_uncertain_action_acceptance_keeps_lease_guard_armed(runtime):
    manager,navigator,moment,_ = completed_analysis(runtime)
    navigator.request.complete = False
    start_parking(manager,navigator,moment)
    manager.deadline = moment[0]-.1
    manager.poll(False)
    moment[0] += 6.
    manager.poll(False)
    assert manager.motion and navigator.leases[-1] == 'suspended'
    assert len(navigator.sent) == 1 and manager.parked_point is None


def test_no_demand_is_completed_analysis_without_any_parking(runtime):
    manager,navigator,_,_ = completed_analysis(runtime)
    manager.people = SimpleNamespace(select=lambda _: None)
    manager._finish_people_scan()
    assert manager.state == 'idle' and manager.parking_outcome == 'not_needed'
    assert manager.analysis_completed and manager.next_people_at == 700.
    assert not navigator.sent


def test_new_observation_cycle_resets_old_parking_result(runtime):
    manager,navigator,moment,selected = completed_analysis(runtime)
    manager.parking_attempts,manager.parking_outcome = 3,'failed'
    manager._start(manager.config['people']['observation_pose'],'going_observation','',180.)
    assert not manager.analysis_completed and manager.analysis_completed_at is None
    assert manager.selected_standby is None and manager.parking_attempts == 0
    assert manager.parking_outcome == 'not_started'


def test_diagnostics_preserve_completed_scan_and_selected_point_after_failure(runtime):
    manager,navigator,moment,selected = completed_analysis(runtime)
    manager.parking_attempts,manager.parking_outcome = 3,'failed'
    manager._people_finished('selected_standby_parking_failed:test',
                             terminal_state='people_parking_blocked')
    messages=[]
    manager.status_pub.publish = lambda m: messages.append(json.loads(m.data))
    manager._heartbeat()
    status=messages[-1]
    assert status['analysis_completed'] and status['selected_standby'] == selected
    assert status['parking_outcome'] == 'failed' and status['parking_attempts'] == 3
    assert status['parking_max_attempts'] == 3 and status['cooldown_remaining_sec'] == 600.
    assert status['continuous_safe_scan_sec'] == 0.
