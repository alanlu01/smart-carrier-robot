"""Replay braking, blocked turns, wraparound and bounded recovery without hardware."""
import math
from types import SimpleNamespace
import pytest
from smart_delivery_core.recovery_spin import RecoverySpin
from smart_delivery_core.settled_motion import can_arm_stationary_anchor, near_convergence
from smart_delivery_core.serial_recovery import fresh_feedback, feedback_diagnostics


def test_braking_tail_cannot_arm_stationary_anchor():
    assert not can_arm_stationary_anchor(100.,99.8,False,True,False)
    assert not can_arm_stationary_anchor(100.5,99.8,False,False,True)
    assert can_arm_stationary_anchor(101.,99.8,False,False,True)


@pytest.mark.parametrize('sign',[1.,-1.])
def test_blocked_300_degree_turn_resumes_only_remaining_60(sign):
    turn = RecoverySpin(sign*2*math.pi,0.,0.,1)
    for i in range(1,61):
        yaw=sign*math.radians(5*i)
        assert turn.observe(math.atan2(math.sin(yaw),math.cos(yaw)),i*100_000_000,.5)
    assert turn.remaining == pytest.approx(math.pi/3)
    assert turn.tick(6.,blocked=True,available=True)=='run'
    assert turn.tick(6.31,blocked=True,available=True)=='cancel_pause'
    turn.phase='stopping'
    assert turn.speed_limit()==0.
    assert turn.stopped(7.,False)=='wait'
    assert turn.stopped(7.5,True)=='wait'
    assert turn.stopped(8.51,True)=='resume'
    assert turn.remaining == pytest.approx(math.pi/3)
    assert turn.resumes==1


def test_endpoint_brakes_early_and_never_requests_reverse():
    turn=RecoverySpin(2*math.pi,0.,0.,1)
    for i in range(1,73):
        angle=math.radians(min(i*5,358.))
        turn.observe(math.atan2(math.sin(angle),math.cos(angle)),i*100_000_000,.3)
    assert turn.speed_limit()>.35
    assert turn.tick(8.,blocked=False,available=True)=='cancel_endpoint'
    turn.phase='stopping'
    assert turn.stopped(9.,True)=='complete'
    assert turn.remaining >= 0.


def test_source_jump_stale_and_wait_budget_fail_closed():
    turn=RecoverySpin(2*math.pi,0.,0.,1)
    assert not turn.observe(1.,1,.1)
    assert not turn.observe(2.,100_000_001,.1)
    assert turn.tick(.1,blocked=False,available=True)=='fail'
    turn=RecoverySpin(2*math.pi,0.,0.,1)
    assert turn.tick(.1,blocked=False,available=False)=='cancel_pause'
    turn.phase='stopping'
    assert turn.tick(15.2,blocked=False,available=False)=='fail'


def test_near_threshold_extra_wait_requires_improvement_and_stability():
    data=[(float(i),(2.,5.,0.),.29-i*.004,.1) for i in range(7)]
    assert near_convergence(data,.25,.17)
    assert not near_convergence([(t,p,.29,y) for t,p,x,y in data],.25,.17)
    assert not near_convergence(data[:-1]+[(6.,(3.,5.,0.),.266,.1)],.25,.17)
    assert not near_convergence(data[:-1]+[(6.,(2.,5.,0.),.266,.3)],.25,.17)


def test_feedback_diagnosis_preserves_gate_and_identifies_receipt_delay():
    payload=dict(sample_ros_stamp_ns=10_000_000_000,complete_ros_stamp_ns=9_900_000_000,
                 complete_sample_age_sec=.1,healthy=True,connected=True,session='rx')
    assert fresh_feedback(payload,10_100_000_000,.1)
    details=feedback_diagnostics(payload,10_100_000_000,.5)
    assert not details['ready'] and details['reason']=='feedback_receipt_age'
    assert details['received_age_sec']==.5
    assert not fresh_feedback(payload,10_700_000_000,.1)
    assert not feedback_diagnostics({},0,float('inf'))['ready']
