"""Recorded 11/12 evening failures: real policy methods, no hardware."""
import json
import math
from types import SimpleNamespace as NS
import pytest
from test_station_navigation import motion,tick,finish_action,obstacle,grid
from test_localization_recovery import manager
from smart_delivery_core.power_action_health import PowerActionHealth
from test_power_action_health import payload
from smart_delivery_core.recovery_spin import RecoverySpin
from smart_delivery_core.recovery_translation import lateral_speed
from test_atomic_wheel_feedback import sample
from smart_delivery_core.atomic_wheel_feedback import AtomicWheelReceiver
from smart_delivery_core.wheel_feedback import WheelSampleAssembler,WheelStopEvidence


def test_other_uncertain_presence_does_not_block_known_bank_inventory():
    data=json.loads(payload());data['ch2']['status']='unknown'
    h=PowerActionHealth();slots=h.observe(json.dumps(data),100.)
    assert h.valid and slots[0]['status']=='full' and slots[1]['status']=='unknown'
    assert not h.action_ready(slots)
    data['ch2']['sensor_ok']=False
    with pytest.raises(ValueError):h.observe(json.dumps(data),101.)
    assert not h.valid


def test_recorded_ch1_low_power_full_and_hysteresis_are_preserved():
    from power_monitor.power_status import SlotStateTracker
    s=SlotStateTracker(confirm_samples=3)
    for _ in range(3):s.update(.059,11.66)
    assert s.state=='full'
    for _ in range(10):assert s.update(.012,11.66)=='full'
    for _ in range(3):s.update(.005,11.66)
    assert s.state=='empty'  # Never permanently invent presence at idle current.


def test_map_shift_during_alignment_is_not_physical_translation(motion):
    m,n,c,p,stopped,permission=motion
    od=[(0.,0.,0.)];m._odom_pose=lambda:od[0]
    m.mode='station';tick(m,c)
    p[0]=(n.sent[-1][0][0],n.sent[-1][0][1],.15)
    n.complete=True;tick(m,c)
    assert m.phase=='aligning'
    anchor=p[0];p[0]=(anchor[0]+.0536,anchor[1],.1)
    od[0]=(.013,0.,-.05)
    assert not m.isTaskComplete() and not m.cancelled
    od[0]=(.06,0.,-.05)
    assert not m.isTaskComplete() and m.cancelled


def test_translation_obstacle_drains_cancel_then_waits_and_rechecks(motion):
    m,n,c,p,stopped,permission=motion
    m.mode='station';tick(m,c);finish_action(m,n,c,p)
    assert m.phase=='entry'
    old=m.grid;obstacle(m.grid,.8,0.)
    assert not m.isTaskComplete() and m.clearance_pending and n.cancels==1
    m.isTaskComplete();assert n.cancels==1
    n.complete=True
    tick(m,c)
    assert m.phase=='waiting_clearance' and m.running
    count=len(n.sent)
    m.grid=grid();m.grid_stamp+=1
    m.isTaskComplete()
    assert m.phase=='entry' and len(n.sent)==count+1
    finish_action(m,n,c,p)
    assert not m.running


def test_stopped_large_sweep_can_finish_without_perfect_heading():
    spin=RecoverySpin(2*math.pi,0.,0.,1,allow_scan_completion=True)
    spin.travel=math.radians(348.);spin.reason='endpoint_braking';spin.phase='stopping'
    assert spin.stopped(12.,False)=='complete'
    assert spin.reason=='settled_scan_coverage' and not spin.snapshot(12.)['precise_endpoint']
    short=RecoverySpin(math.radians(8.),0.,0.,1)
    short.reason='endpoint_braking';short.phase='stopping'
    assert short.stopped(1.,False)=='wait'  # Small tails cannot finish without moving.


def test_recorded_lateral_momentum_brakes_before_272mm():
    assert 0 < lateral_speed(.24,.30,.15,feedback_age=.15,measured_speed=.10) < .10
    assert lateral_speed(.10,.30,.15,feedback_age=.15,measured_speed=.10)>0.


def test_clock_backstep_rebuilds_stop_epoch_without_accepting_replay():
    a=WheelSampleAssembler(.25,0.);stop=WheelStopEvidence(0.);r=AtomicWheelReceiver(a,stop)
    for i,t in enumerate((100.,100.5,101.),1):r.observe(sample(t,i),round(t*1e9),t)
    assert stop.ready(101.)
    assert r.observe(sample(93.,4),93_000_000_000,102.) is not None
    assert not stop.ready(102.)
    assert r.observe(sample(93.1,3),93_100_000_000,102.1) is None
    assert r.observe(sample(93.2,1,'new'),93_200_000_000,102.2) is not None
    assert r.observe(sample(93.3,5,'rx'),93_300_000_000,102.3) is None


def test_pipeline_sensor_update_does_not_need_idle_command_heartbeat(manager):
    n,t=manager
    n.state='MANUAL_REQUIRED';n.manual_intervention_kind='pipeline';n.last_amcl_at=0.
    n.navigation_health={'state':'ready'};n.navigation_health_at=t.now
    n._lateral_sensor_blockers=lambda _:[]
    n.recovery_stop=NS(ready=lambda _:True)
    n.commanded_motion=n.odom_motion=False;n.latest_cmd_at=t.now-60.
    n.get_clock=lambda:NS(now=lambda:NS(nanoseconds=int(t.now*1e9)))
    calls=[];n._request_nomotion_if_due=lambda _:calls.append(True)
    assert n._refresh_pipeline_amcl(t.now) and calls and not n.started_spins
    t.now+=1.;n.last_amcl_at=t.now
    assert not n._refresh_pipeline_amcl(t.now) and not n.ready


def test_pipeline_refresh_is_bounded_and_does_not_issue_motion(manager):
    n,t=manager
    n.state='MANUAL_REQUIRED';n.manual_intervention_kind='pipeline';n.last_amcl_at=0.
    n.navigation_health={'state':'ready'};n.navigation_health_at=t.now
    n._lateral_sensor_blockers=lambda _:[];n.recovery_stop=NS(ready=lambda _:True)
    n.commanded_motion=n.odom_motion=False
    n.get_clock=lambda:NS(now=lambda:NS(nanoseconds=int(t.now*1e9)))
    calls=[];n._request_nomotion_if_due=lambda _:calls.append(True)
    assert n._refresh_pipeline_amcl(t.now)
    t.now+=9.;n.navigation_health_at=t.now
    assert not n._refresh_pipeline_amcl(t.now)
    assert not n._refresh_pipeline_amcl(t.now+.1)
    assert len(calls)==1 and not n.ready and not n.started_spins


def test_invalid_numeric_slot_data_is_not_usable_inventory():
    data=json.loads(payload());data['ch2']['current']=float('nan')
    with pytest.raises(ValueError):PowerActionHealth().observe(json.dumps(data),100.)


def test_destination_fallback_does_not_depart_from_mid_route_pose(motion):
    m,n,c,p,stopped,permission=motion
    tick(m,c)
    p[0]=(.3,0.,.3);m.mode_switch_scope='destination'
    m._request_station_fallback();n.complete=True
    tick(m,c)
    assert m.phase!='departure' and m.mode=='station'
    assert m.after_stop!='prepare'


def test_translation_physical_guard_survives_a_map_only_shift(motion):
    m,n,c,p,stopped,permission=motion
    od=[(0.,0.,0.)];m._odom_pose=lambda:od[0]
    m.mode='station';tick(m,c);finish_action(m,n,c,p)
    anchor=p[0];p[0]=(anchor[0]+.1,anchor[1]+.11,0.)
    od[0]=(.1,.01,0.)
    assert not m.isTaskComplete() and not m.cancelled
    od[0]=(.1,.11,0.)
    assert not m.isTaskComplete() and m.cancelled
