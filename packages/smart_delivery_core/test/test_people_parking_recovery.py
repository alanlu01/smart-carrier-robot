"""Bounded intent preservation across recovery; mock actions, never real motion."""

import json
from types import SimpleNamespace

import pytest
from nav2_simple_commander.robot_navigator import TaskResult
from tf2_ros import TransformException

from smart_delivery_core.optional_idle_runtime import OptionalIdleRuntime
from test_optional_idle_runtime import runtime
from test_people_parking import completed_analysis,start_parking
from test_people_retry import prepare


def interrupted(runtime):
    manager,nav,clock,selected=completed_analysis(runtime)
    manager._localization_phase=lambda:'SUSPECT'
    start_parking(manager,nav,clock)
    manager._safe=lambda:False
    manager.poll(False)
    assert manager.motion and manager.cancel_requested
    assert manager.interrupt_reason=='selected_standby_safety_pause'
    nav.complete,nav.result=True,TaskResult.CANCELED
    manager.poll(False)
    assert manager.state=='waiting_people_parking_recovery'
    return manager,nav,clock,selected


def recover(manager,clock):
    manager._safe=manager._stationary=lambda:True
    manager._stamp_fresh=lambda *args:True
    manager._localization_phase=lambda:'LOCALIZED'
    manager.poll(False);clock[0]+=1.1;manager.poll(False)
    manager.poll(False);clock[0]+=1.1;manager.poll(False)


def test_recovery_retargets_same_selected_point_without_new_scan_or_false_arrival(runtime):
    m,n,t,point=interrupted(runtime)
    cooldown,deadline=m.next_people_at,m.parking_deadline
    recover(m,t)
    assert m.state=='people_parking_retry_wait'
    assert m.selected_standby==point and m.parked_point is None
    m.idle()
    assert len(n.sent)==2 and m.state=='going_people_standby'
    assert n.sent[-1].pose.pose.position.x==point['x']
    assert n.sent[-1].pose.pose.position.y==point['y']
    assert m.next_people_at==cooldown and m.parking_deadline==deadline
    assert m.parking_recovery_pauses==1 and m.parking_retry.failures==0
    assert all(not goal.behavior_tree for goal in n.sent)


def test_global_recovery_may_finish_before_same_map_target_is_revalidated(runtime):
    m,n,t,point=interrupted(runtime)
    m._localization_phase=lambda:'RECOVERING_GLOBAL'
    m._stationary=lambda:False
    t[0]+=30.;assert m.poll(False)
    assert m.state=='waiting_people_parking_recovery' and len(n.sent)==1
    recover(m,t)
    assert m.state=='people_parking_retry_wait' and m.selected_standby==point


@pytest.mark.parametrize('hard',['manual','map','battery','inhibited','target'])
def test_hard_failure_discards_intent_and_never_sends_another_goal(runtime,hard):
    m,n,t,_=interrupted(runtime)
    if hard=='manual':m._localization_phase=lambda:'MANUAL_REQUIRED'
    elif hard=='map':m.map_matches=False
    elif hard=='inhibited':m.inhibited=True
    elif hard=='target':m.selected_standby=dict(m.selected_standby,x=999.)
    else:m.home=SimpleNamespace(admission_latched=True,latched=True,healthy=True,cutoff=False)
    m.poll(False)
    assert m.state=='idle' and m.parking_outcome==('preempted' if hard=='battery' else 'interrupted') and len(n.sent)==1


def test_new_order_preempts_recovery_even_when_data_unavailable(runtime):
    m,n,_,_=interrupted(runtime)
    m.poll(True)
    assert m.state=='idle' and m.parking_outcome=='preempted' and len(n.sent)==1


def test_wait_deadline_expires_even_if_external_recovery_never_stops(runtime):
    m,n,t,_=interrupted(runtime)
    end=m.parking_recovery_deadline
    m._stationary=lambda:False
    t[0]=end+.1;m.poll(False)
    assert m.state=='idle' and m.reason=='selected_standby_recovery_timeout'
    assert m.settling_after_cancel and len(n.sent)==1


def test_ready_but_stale_scan_never_resumes_parking(runtime):
    m,n,t,_=interrupted(runtime)
    recover(m,t)
    m.state='waiting_people_parking_recovery';m.parking_recovery_deadline=t[0]+10.
    m._stamp_fresh=lambda *args:False
    for _ in range(3):m.poll(False);t[0]+=.5
    assert m.state=='waiting_people_parking_recovery' and len(n.sent)==1


def test_pause_count_and_original_deadline_do_not_reset(runtime):
    m,n,t,_=interrupted(runtime)
    end=m.parking_deadline
    m.parking_recovery_pauses=2
    assert not m._begin_parking_recovery('third_pause')
    assert m.state=='idle' and m.parking_deadline==end and len(n.sent)==1


def test_pre_spin_transient_unsafe_waits_only_until_original_deadline(runtime):
    m,n,t=prepare(runtime)
    m.state,m.deadline='waiting_observation_stop',110.
    m._safe=lambda:False;m._can_wait_localization=lambda:False
    m._localization_phase=lambda:'LOCALIZED'
    assert m.poll(False) and m.state=='waiting_observation_stop'
    assert m.spin_ready_blocker=='waiting_fresh_safety_data'
    t[0]=109.;m.poll(False)
    assert m.deadline==110. and not n.sent
    t[0]=110.1;m.poll(False)
    assert m.state=='idle' and m.reason=='spin_ready_timeout:unsafe_data'


def test_pre_spin_global_recovery_discards_directional_evidence(runtime):
    m,n,t=prepare(runtime)
    m.state,m.deadline='waiting_observation_stop',110.
    m._safe=lambda:False;m._can_wait_localization=lambda:False
    m._localization_phase=lambda:'RECOVERING_GLOBAL'
    m.poll(False)
    assert m.state=='idle' and m.reason=='observation_wait_unsafe' and not n.sent


def test_safety_diagnostics_distinguish_tf_and_odom_and_are_rate_limited(runtime):
    m,n,t=runtime
    messages=[];n.get_logger=lambda:SimpleNamespace(warning=messages.append,info=messages.append)
    m._pose=lambda *_:None
    assert not OptionalIdleRuntime._safe(m)
    assert m.safety_report['failed_checks']==['map_pose_valid']
    m._pose=lambda *_:{'x':0.,'y':0.,'yaw':0.}
    m.last_odom=(90.,m.last_odom[1])
    assert not OptionalIdleRuntime._safe(m)
    assert m.safety_report['failed_checks']==['odom_fresh']
    assert len(messages)==1
    t[0]+=5.1;OptionalIdleRuntime._safe(m)
    assert len(messages)==2
    assert m.last_unsafe_report['odom_receive_age_sec']>10.


def test_tf_exception_is_preserved_without_granting_a_pose(runtime):
    m,_,_=runtime
    def fail(*args):raise TransformException('specific extrapolation failure')
    m.buffer.lookup_transform=fail
    assert m._pose() is None
    assert m.pose_report['reason']=='tf_lookup_failed'
    assert 'specific extrapolation' in m.pose_report['detail']


def test_recovery_does_not_grant_a_fourth_parking_goal(runtime):
    m,n,t,_=completed_analysis(runtime)
    m.parking_attempts=m.parking_retry.max_attempts
    assert m.idle() and m.state=='people_parking_blocked'
    assert m.parked_point is None and not n.sent


def test_recovery_wait_never_extends_original_parking_deadline(runtime):
    m,n,t,_=completed_analysis(runtime)
    m.parking_deadline=t[0]+5.
    assert m._begin_parking_recovery('transient')
    assert m.parking_recovery_deadline==m.parking_deadline
    t[0]+=5.1;m.poll(False)
    assert m.state=='idle' and m.reason=='selected_standby_recovery_timeout'
    assert not n.sent
