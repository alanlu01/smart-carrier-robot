"""Bounded, fail-closed crowd observation using fake navigation actions."""
import math
from types import SimpleNamespace
from xml.etree import ElementTree
import pytest
from nav2_simple_commander.robot_navigator import TaskResult
from sensor_msgs.msg import LaserScan
from test_optional_idle_runtime import runtime


def prepare(runtime):
    manager,navigator,moment=runtime
    manager._safe=manager._stationary=lambda:True
    manager._pose=lambda *args:dict(manager.config['people']['observation_pose'])
    manager.home=None
    manager.mark_parked(manager.config['people']['standby_points'][0])
    scan=LaserScan()
    scan.header.stamp.sec=100
    manager.last_scan=(100.,scan)
    return manager,navigator,moment


def test_idle_clock_begins_only_at_verified_standby(runtime):
    manager,navigator,moment=prepare(runtime)
    point=manager.config['people']['standby_points'][0]
    manager._pose=lambda *args:dict(point)
    assert not manager.idle(parked=False) and manager.idle_since is None
    moment[0]+=100.
    assert not manager.idle(parked=True)
    assert manager.idle_since==200.
    for _ in range(59):
        moment[0]+=1.
        assert not manager.idle(parked=True)
    assert not manager.idle(parked=True) and not navigator.sent
    moment[0]+=1.
    assert manager.idle(parked=True) and len(navigator.sent)==1
    assert manager.state=='going_observation'


def test_motion_or_wrong_standby_pose_resets_idle_window(runtime):
    manager,_,moment=prepare(runtime)
    manager._pose=lambda *args:dict(manager.config['people']['standby_points'][0])
    manager.idle(parked=True)
    moment[0]+=30.
    manager._stationary=lambda:False
    assert not manager.idle(parked=True) and manager.idle_since is None
    manager._stationary=lambda:True
    manager._pose=lambda *args:{'x':99.,'y':99.,'yaw':0.}
    assert not manager.idle(parked=True) and manager.idle_since is None


def test_spin_requires_independent_safe_scans_and_continuous_stop(runtime):
    manager,_,moment=prepare(runtime)
    manager._clear_to_spin=lambda *args:True
    for at in (100.,100.1,100.2,100.3):
        moment[0]=at
        assert not manager._spin_start_ready()
    assert manager.safe_scan_count==1
    for idx,at in enumerate((100.4,101.1),start=1):
        moment[0]=at
        manager.last_scan[1].header.stamp.nanosec=idx
        answer=manager._spin_start_ready()
    assert answer
    manager._clear_to_spin=lambda *args:False
    assert not manager._spin_start_ready() and manager.safe_scan_count==0
    assert manager.safe_stop_since==100.
    manager._stationary=lambda:False
    assert not manager._spin_start_ready() and manager.safe_stop_since is None


def test_wait_deadline_cannot_be_bypassed_by_late_safe_scans(runtime):
    manager,navigator,moment=prepare(runtime)
    manager.state='waiting_observation_stop'
    manager.deadline=99.
    manager._spin_start_ready=lambda:pytest.fail('expired phase cannot start')
    assert manager.idle()
    assert manager.state=='idle' and not navigator.sent
    assert manager.next_people_at==130.


def begin_spin(runtime):
    manager,navigator,moment=prepare(runtime)
    manager.observation_collecting=True
    manager.observation_session_deadline=190.
    manager._start(manager.config['people']['observation_pose'],'observing','people.xml',35.)
    manager.spin_travel=.6
    manager.people=SimpleNamespace(select=lambda _:None)
    manager.guard_report={'clear':False,'reason':'unknown_scan_coverage'}
    manager._clear_to_spin=lambda *args:False
    return manager,navigator,moment


def test_unknown_scan_cancels_and_drains_before_any_resume(runtime):
    manager,navigator,moment=begin_spin(runtime)
    assert manager.poll(False) and navigator.cancel_calls==1
    assert manager.interrupt_reason=='unknown_scan_coverage'
    navigator.complete=True
    navigator.result=TaskResult.CANCELED
    assert manager.poll(False)
    assert manager.state=='waiting_observation_retry' and manager.observation_collecting
    assert manager.idle() and len(navigator.sent)==1
    assert manager.poll(False)
    moment[0]+=1.1
    assert not manager.poll(False)
    manager._spin_start_ready=lambda:True
    manager.idle()
    assert manager.state=='observing' and manager.spin_resumes==0
    assert manager.spin_resume_requests==1 and manager.spin_resume_pending
    assert len(navigator.sent)==2
    spin=ElementTree.parse(navigator.sent[-1].behavior_tree).find('.//Spin')
    assert float(spin.get('spin_dist'))==pytest.approx(2*math.pi-.6)
    assert manager.spin_travel==.6
    manager._people_finished('nav2_failed')  # remove test temporary BT


@pytest.mark.parametrize('reason',['scan_stale_or_absent','invalid_laser_geometry',
                                   'laser_tf_unavailable','unsafe_localization_or_odom'])
def test_hard_sensor_or_obstacle_rejection_never_enters_unknown_retry(runtime,reason):
    manager,navigator,_=begin_spin(runtime)
    manager.guard_report['reason']=reason
    manager.poll(False)
    navigator.complete=True
    navigator.result=TaskResult.CANCELED
    manager.poll(False)
    assert manager.state=='idle' and manager.reason==reason
    assert not manager.observation_collecting


def test_spin_resume_and_session_caps(runtime):
    manager,navigator,_=begin_spin(runtime)
    manager.spin_resumes=2
    manager.poll(False)
    navigator.complete=True
    navigator.result=TaskResult.CANCELED
    manager.poll(False)
    assert manager.state=='idle'
    manager.state='waiting_observation_retry'
    manager.observation_session_deadline=99.
    manager.deadline=110.
    manager.settling_after_cancel=False
    assert manager.idle()
    assert manager.state=='idle' and len(navigator.sent)==1


def test_service_preempts_waiting_resume_without_another_goal(runtime):
    manager,navigator,_=prepare(runtime)
    manager.state='waiting_observation_retry'
    assert not manager.poll(True)
    assert manager.state=='idle' and not navigator.sent


def test_failed_cycles_get_short_cooldown_then_bounded_backoff(runtime):
    manager,_,moment=prepare(runtime)
    for index,expected in enumerate((30.,30.,600.)):
        manager._people_finished('unknown_scan_coverage')
        assert manager.next_people_at-moment[0]==expected
        moment[0]+=expected
    assert manager.people_failures==0
    manager._people_finished('insufficient_directional_demand')
    assert manager.next_people_at-moment[0]==600.


def test_diagnostics_heartbeat_does_not_spam_unchanged_states(runtime):
    manager,navigator,moment=prepare(runtime)
    logged=[]
    navigator.get_logger=lambda:SimpleNamespace(info=logged.append,warning=logged.append)
    manager._heartbeat()
    manager._heartbeat()
    assert len(logged)==1
    manager.state='waiting_observation_retry'
    manager.reason='unknown_scan_coverage'
    manager._heartbeat()
    assert len(logged)==2 and '雷達視野不足' in logged[-1]
    assert manager.reason=='unknown_scan_coverage'  # machine code is unchanged
    moment[0]+=5.1
    manager._heartbeat()
    assert len(logged)==3


def test_successful_spin_stops_before_selected_parking_goal(runtime):
    manager,navigator,moment=begin_spin(runtime)
    manager._clear_to_spin=lambda *args:True
    manager.spin_travel=6.2
    manager.people=SimpleNamespace(select=lambda _:manager.config['people']['standby_points'][0])
    navigator.complete=True
    assert manager.poll(False)
    assert manager.state=='people_destination_selected'
    manager.idle()
    assert len(navigator.sent)==1
    manager.poll(False)
    moment[0]+=1.1
    assert not manager.poll(False)
    manager.idle()
    moment[0]+=1.1
    manager.idle()
    assert manager.state=='going_people_standby' and len(navigator.sent)==2


def test_no_vision_demand_finishes_without_invented_destination(runtime):
    manager,navigator,_=begin_spin(runtime)
    manager._clear_to_spin=lambda *args:True
    manager.spin_travel=6.2
    navigator.complete=True
    manager.poll(False)
    assert manager.state=='idle' and manager.reason=='insufficient_directional_demand'
    assert len(navigator.sent)==1 and manager.next_people_at==700.


def test_unknown_guard_report_does_not_reclassify_inf_as_clear(runtime):
    manager,_,_=prepare(runtime)
    data=manager.last_scan[1]
    data.header.frame_id='inverted_lidar'
    data.angle_min=-math.pi
    data.angle_increment=math.pi/180
    data.range_min=.05
    data.range_max=10.
    data.ranges=[2.]*360
    for index in range(100,120):
        data.ranges[index]=float('inf')
    assert not manager._clear_to_spin(False)
    assert manager.guard_report['reason']=='unknown_scan_coverage'
    assert manager.guard_report['unknown_arc_deg']==pytest.approx(20.)


def test_history_guard_cache_expires_and_new_near_scan_overrides_it(runtime,monkeypatch):
    from smart_delivery_core import optional_idle_runtime as module
    manager,_,moment=prepare(runtime)
    manager.state='observing'
    for sec,nsec in ((99,800000000),(99,900000000),(100,0)):
        scan=LaserScan()
        scan.header.stamp.sec,scan.header.stamp.nanosec=sec,nsec
        scan.header.frame_id='laser'
        scan.angle_min=-math.pi
        scan.angle_increment=2*math.pi/360
        scan.range_min,scan.range_max=.05,10.
        scan.ranges=[3.]*360
        moment[0]=sec+nsec/1e9
        manager._scan(scan)
    real=module.aligned_spin_report
    calls=[]
    def counted(*args):
        calls.append(1)
        return real(*args)
    monkeypatch.setattr(module,'aligned_spin_report',counted)
    assert manager._clear_to_spin(False) and len(calls)==1
    moment[0]+= .02
    assert manager._clear_to_spin(False) and len(calls)==1
    moment[0]+= .03
    assert manager._clear_to_spin(False) and len(calls)==2
    scan.header.stamp.nanosec=50000000
    scan.ranges[0]=.2
    manager._scan(scan)
    assert not manager._clear_to_spin(False)
    assert manager.guard_report['reason']=='near_obstacle'
    moment[0]+=1.
    assert not manager._clear_to_spin(False)
    assert manager.guard_report['reason']=='scan_stale_or_absent'


def test_permission_must_stay_clear_for_750ms_not_just_three_scans(runtime):
    manager,_,moment=prepare(runtime)
    manager._clear_to_spin=lambda *_:True
    for i in range(5):
        moment[0]=100.+.1*i
        manager.last_scan[1].header.stamp.nanosec=i
        assert not manager._spin_start_ready()
    moment[0]=101.1
    assert manager._spin_start_ready()
    manager._clear_to_spin=lambda *_:False
    assert not manager._spin_start_ready() and manager.safe_scan_since is None
    manager._clear_to_spin=lambda *_:True
    moment[0]=101.2
    assert not manager._spin_start_ready()


def test_resumed_request_only_counts_after_real_progress_and_has_request_cap(runtime):
    from nav_msgs.msg import Odometry
    manager,navigator,moment=prepare(runtime)
    manager.observation_collecting=True
    manager._start(manager.config['people']['observation_pose'],'observing','people.xml',35.,resume=True)
    assert manager.spin_resume_requests==1 and manager.spin_resumes==0
    data=Odometry();data.pose.pose.orientation.w=1.;manager._odom(data)
    data=Odometry();data.pose.pose.orientation.w=math.cos(.03/2);data.pose.pose.orientation.z=math.sin(.03/2)
    manager._odom(data);assert manager.spin_resumes==0
    data=Odometry();data.pose.pose.orientation.w=math.cos(.06/2);data.pose.pose.orientation.z=math.sin(.06/2)
    manager._odom(data);assert manager.spin_resumes==1
    manager._odom(data);assert manager.spin_resumes==1
    manager.spin_resume_requests=4
    manager._clear_to_spin=lambda *_:False
    manager.guard_report={'clear':False,'reason':'unknown_scan_coverage'}
    manager.observation_session_deadline=190.
    navigator.complete=True;navigator.result=TaskResult.CANCELED
    manager.poll(False)
    assert manager.state=='idle'  # no fifth transport request
