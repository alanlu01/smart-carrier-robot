"""Stationary AMCL correction and nonblocking auxiliary recovery regressions."""
import math
from types import SimpleNamespace as NS
import pytest
from sensor_msgs.msg import LaserScan
from smart_delivery_core.standby_idle_clock import StandbyIdleClock
from test_standby_idle_clock import POINT,tick
from test_optional_idle_runtime import runtime
from test_station_navigation import motion,tick as station_tick
from nav2_simple_commander.robot_navigator import TaskResult


def gradual(clock,at,yaw,**changes):
    return tick(clock,at,pose=dict(x=2.,y=5.18,yaw=yaw),**changes)


def test_gradual_thirteen_degree_correction_preserves_time_until_trusted_and_settled():
    c=StandbyIdleClock();c.mark_parked(POINT)
    for i in range(80):
        at=i*.5;gradual(c,at,min(math.radians(13),at*.008),trusted_map_correction=False)
    assert c.elapsed>20. and c.pause_since is not None and not c.correction_used
    before=c.elapsed
    for i in range(9):gradual(c,40.+i*.5,math.radians(13),trusted_map_correction=True)
    assert c.correction_used and c.elapsed>=before and c.deviation_resets==0
    assert c.point==POINT and c.map_reference['yaw']==math.radians(13)
    # Rebased time never changes the real navigation point or authorized heading.
    assert c.point['yaw']==0.


def test_untrusted_pose_expires_once_then_rearms_only_with_stable_fresh_quality():
    c=StandbyIdleClock();c.mark_parked(POINT)
    for i in range(181):gradual(c,i*.5,min(math.radians(13),i*.004),trusted_map_correction=False)
    assert c.elapsed==0. and c.deviation_resets==1 and not c.correction_used
    for i in range(8):gradual(c,91.+i*.5,math.radians(13),trusted_map_correction=True)
    assert c.correction_used and c.elapsed<=.5 and c.deviation_resets==1


@pytest.mark.parametrize('fault',['jump','motion','too_far','quality'])
def test_map_correction_cannot_hide_kidnap_or_bad_evidence(fault):
    c=StandbyIdleClock();c.mark_parked(POINT);gradual(c,0.,0.)
    for i in range(1,181):
        heading=(math.radians(13) if fault=='jump' else
                 min(math.radians(21 if fault=='too_far' else 13),i*.004))
        gradual(c,i*.5,heading,trusted_map_correction=(fault!='quality' and (fault!='too_far' or heading>math.radians(20))),
                odom=(.03,0.,0.) if fault=='motion' else (0.,0.,0.))
    assert not c.correction_used and c.elapsed==0.


def test_confirmation_cannot_count_an_executor_gap():
    c=StandbyIdleClock();c.mark_parked(POINT)
    for i in range(70):gradual(c,i*.5,min(.227,i*.004),trusted_map_correction=True)
    # Start a distinct fresh parking proof, gradually cross into correction.
    c.mark_parked(POINT)
    for i in range(32):gradual(c,i*.5,min(.227,i*.008),trusted_map_correction=False)
    gradual(c,16.,.227,trusted_map_correction=True)
    gradual(c,20.,.227,trusted_map_correction=True)
    assert not c.correction_used and c.correction_since==20.


def test_async_cancel_returns_while_ack_pending_and_keeps_old_goal_owned(motion):
    m,n,c,p,_,_=motion;station_tick(m,c)
    pending=NS(done=lambda:False);requests=[]
    n.goal_handle=NS(cancel_goal_async=lambda:requests.append(True) or pending)
    n.cancelTask=lambda:pytest.fail('blocking cancelTask must not be used')
    m.cancelTask();m.cancelTask()
    assert requests==[True] and m.running and m.active_goal
    assert not m.isTaskComplete() and len(n.sent)==1
    pending.done=lambda:True  # A cancellation response is not terminal proof.
    assert not m.isTaskComplete() and m.active_goal
    n.complete=True;n.result=TaskResult.CANCELED
    assert m.isTaskComplete() and m.result==TaskResult.CANCELED


def test_cancel_transport_exception_never_releases_an_unconfirmed_goal(motion):
    m,n,c,p,_,_=motion;station_tick(m,c)
    def error():raise RuntimeError('transport unavailable')
    n.goal_handle=NS(cancel_goal_async=error)
    m.cancelTask();assert not m.isTaskComplete() and m.active_goal
    assert m.failure_kind=='cancel_request_error' and m.data_unavailable


def prepare_wait(runtime):
    m,n,c=runtime
    m._safe=m._stationary=lambda:True;m._localization_phase=lambda:'LOCALIZED'
    m.last_scan=(c[0],LaserScan());m.last_scan[1].header.stamp.sec=100
    m._stamp_fresh=lambda *a:True
    m._begin_localization_wait('going_observation',120.,data_only=True)
    m.state='waiting_people_localization'
    sent=[];m._start=lambda *a,**k:sent.append((a,k)) or True
    return m,n,c,sent


def new_sources(m,c):
    sec=int(c[0]);ns=round((c[0]-sec)*1e9)
    for msg in (m.last_odom[1],m.last_scan[1]):
        msg.header.stamp.sec=sec;msg.header.stamp.nanosec=ns
    m.pose_report=dict(source_stamp_ns=round(c[0]*1e9))


def test_data_resume_needs_post_pause_sources_and_continuous_fresh_interval(runtime):
    m,n,c,sent=prepare_wait(runtime)
    for at in (100.1,100.4):
        c[0]=at;m._poll_localization_wait()
    assert not sent and m.localization_resume_blocker=='waiting_post_pause_sources'
    c[0]=100.5;new_sources(m,c);m._poll_localization_wait();assert not sent
    c[0]=100.6;m._safe=lambda:False;m._poll_localization_wait()
    c[0]=100.7;m._safe=lambda:True;new_sources(m,c);m._poll_localization_wait();assert not sent
    c[0]=100.91;new_sources(m,c);m._poll_localization_wait()
    assert len(sent)==1 and sent[0][1]['resume'] and sent[0][0][3]<20.


@pytest.mark.parametrize('fault',['deadline','global','order'])
def test_data_pause_stays_bounded_and_never_overrides_service(runtime,fault):
    m,n,c,sent=prepare_wait(runtime)
    if fault=='deadline':c[0]=121.
    if fault=='global':m._localization_phase=lambda:'RECOVERING_GLOBAL'
    if fault=='order':m.poll(True)
    else:m._poll_localization_wait()
    assert not sent and m.state=='idle'


def test_stopped_time_during_cancel_is_reused_only_with_fresh_stop_proof(runtime):
    m,n,c=runtime;m._safe=m._stationary=lambda:True
    assert m._start(m.config['people']['observation_pose'],'going_observation','',20.)
    assert m.poll(True)
    c[0]+=1.1;assert m.poll(True)  # Cancel ACK not terminal yet.
    n.complete=True;n.result=TaskResult.CANCELED
    assert m.poll(True) and m.settling_after_cancel
    assert not m.poll(True) and not m.settling_after_cancel


def test_cooldown_is_shown_instead_of_a_misleading_sixty_second_countdown(runtime):
    m,n,c=runtime;logs=[];n.get_logger=lambda:NS(info=logs.append,warning=logs.append)
    m.idle_clock.mark_parked(POINT);m.idle_clock.reason='counting_verified_stationary_parking'
    m.next_people_at=c[0]+600.;m._log_people_state()
    assert '冷卻' in logs[-1] and '600' in logs[-1] and '計時中' not in logs[-1]
    m.reason='insufficient_directional_demand';m.idle_clock.point=None;m._log_people_state()
    assert '分析完成' in logs[-1] and '可用人形不足' in logs[-1]
    m.log_key=None;m._log_people_state()
    assert '冷卻' in logs[-1] and '分析完成' not in logs[-1]


@pytest.mark.parametrize('fault',['unsafe','moving','long_tf_gap','executor_gap'])
def test_unobserved_physical_history_cannot_rebase_until_new_parking_proof(fault):
    c=StandbyIdleClock();c.mark_parked(POINT);gradual(c,0.,0.);gradual(c,.5,.002)
    if fault=='unsafe':gradual(c,1.,.004,safe=False)
    if fault=='moving':gradual(c,1.,.004,stationary=False)
    if fault=='long_tf_gap':
        tick(c,1.,safe=False,pose=None,transient_tf_gap=True)
        tick(c,2.1,safe=False,pose=None,transient_tf_gap=True)
    if fault=='executor_gap':gradual(c,3.,.004)
    for i in range(1,181):gradual(c,4.+i*.5,min(.227,i*.004),trusted_map_correction=True)
    assert c.correction_disqualified and not c.correction_used and c.elapsed==0.
    c.mark_parked(POINT)
    assert not c.correction_disqualified
