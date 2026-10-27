"""Transient data loss stops/drains before bounded, fresh, remaining-angle resume."""

import math
from pathlib import Path
from xml.etree import ElementTree as ET

import pytest
from nav2_simple_commander.robot_navigator import TaskResult

from test_optional_idle_runtime import runtime
from test_people_retry import begin_spin
from test_people_localization_wait import recover


def data_pause(runtime):
    m,n,t=begin_spin(runtime)
    m.observation_alignment=(0.,0.,0.)
    m._map_alignment=lambda:(0.,0.,0.)
    m._safe=lambda:False
    m._localization_phase=lambda:'LOCALIZED'
    m._can_wait_localization=lambda:False
    m.poll(False)
    assert m.interrupt_reason=='people_data_pause' and n.cancel_calls==1
    assert m.motion and len(n.sent)==1
    n.complete,n.result=True,TaskResult.CANCELED
    m.poll(False)
    assert m.state=='waiting_people_localization' and m.settling_after_cancel
    m.poll(False);t[0]+=1.1;m.poll(False)
    return m,n,t


def test_short_data_gap_can_resume_remaining_angle_not_a_new_full_turn(runtime):
    m,n,t=data_pause(runtime)
    end=m.observation_session_deadline
    t[0]+=2.;recover(m);m.poll(False)
    assert m.state=='waiting_observation_retry' and m.spin_travel==.6
    m.idle()
    assert len(n.sent)==2 and m.state=='observing'
    tree=ET.parse(n.sent[-1].behavior_tree)
    assert float(tree.find('.//Spin').attrib['spin_dist'])==pytest.approx(2*math.pi-.6)
    assert m.observation_session_deadline==end and m.localization_pauses==1
    m._people_finished('test_cleanup')


@pytest.mark.parametrize('fault',['no_odom','no_scan','no_ready'])
def test_resume_still_requires_fresh_evidence_and_dispatch_permission(runtime,fault):
    m,n,t=data_pause(runtime)
    recover(m)
    if fault=='no_ready':m._safe=lambda:False
    elif fault=='no_scan':m.last_scan=None
    else:m._safe=lambda:False
    m.poll(False)
    assert m.state=='waiting_people_localization' and len(n.sent)==1


@pytest.mark.parametrize('phase',['RECOVERING_GLOBAL','MANUAL_REQUIRED','VERIFYING'])
def test_real_recovery_escalation_discards_old_people_evidence(runtime,phase):
    m,n,_=data_pause(runtime)
    m._localization_phase=lambda:phase;m.poll(False)
    assert m.state=='idle' and not m.observation_collecting and len(n.sent)==1


def test_new_service_preempts_data_wait_without_any_resume(runtime):
    m,n,_=data_pause(runtime)
    m.poll(True)
    assert m.state=='idle' and len(n.sent)==1


def test_missing_data_never_extends_twenty_second_wait_or_ninety_second_session(runtime):
    m,n,t=data_pause(runtime)
    deadline=m.localization_wait_deadline
    assert deadline==120. and m.observation_session_deadline==190.
    t[0]=deadline+.01;m.poll(False)
    assert m.state=='idle' and m.reason=='people_localization_wait_timeout' and len(n.sent)==1


def test_pause_budget_not_reset_by_recovery(runtime):
    m,n,_=data_pause(runtime)
    m.state,m.motion,m.cancel_requested='observing',True,False
    m.localization_pauses=2;m._safe=lambda:False
    m.poll(False)
    assert m.interrupt_reason=='unsafe_localization_or_odom_or_battery'
    n.complete=True;m.poll(False)
    assert m.state=='idle' and m.localization_pauses==2


def test_stale_scan_guard_can_request_data_pause_without_weakening_geometry(runtime):
    m,n,_=begin_spin(runtime)
    m._localization_phase=lambda:'LOCALIZED'
    m.guard_report={'clear':False,'reason':'scan_stale_or_absent'}
    m.poll(False)
    assert m.interrupt_reason=='people_data_pause' and n.cancel_calls==1
    assert m.config['people']['max_unknown_arc_deg']==15.


def test_near_obstacle_is_still_immediate_cancel_not_data_grace(runtime):
    m,n,_=begin_spin(runtime)
    m._localization_phase=lambda:'LOCALIZED'
    m.guard_report={'clear':False,'reason':'near_obstacle'}
    m.poll(False)
    assert m.interrupt_reason=='near_obstacle' and n.cancel_calls==1
