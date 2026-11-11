"""Recorded remote blockage, physical dead zone and manual-seed races."""
import math
from types import SimpleNamespace as NS
import pytest
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_simple_commander.robot_navigator import TaskResult
from test_station_navigation import motion,tick,finish_action,grid,obstacle
from test_localization_recovery import manager
from smart_delivery_core.recovery_spin import RecoverySpin
from smart_delivery_core.standby_retry import StandbyRetryPolicy


def test_remote_dynamic_goal_cell_does_not_prevent_reaching_preparation(motion):
    m,n,c,p,_,_=motion;p[0]=(-1.,0.,0.)
    m.static_grid=grid();obstacle(m.grid,1.,0.)
    assert not m.grid.staging(m.target,p[0])
    tick(m,c)
    assert m.phase=='staging' and m.remote_preparation and len(n.sent)==1
    assert n.sent[0][0]!=m.target
    finish_action(m,n,c,p)
    assert m.phase=='waiting_clearance' and m.clearance_scope=='destination_final'
    assert len(n.sent)==1 and not m.motion_stalled
    m.grid=grid();m.grid_stamp+=1;m.isTaskComplete()
    assert m.phase=='entry' and len(n.sent)==2 and n.sent[-1][0]==m.target
    finish_action(m,n,c,p)
    assert m.getResult()==TaskResult.SUCCEEDED


def test_static_wall_or_unknown_cannot_be_deferred_as_old_dynamic_obstacle(motion):
    m,n,c,p,_,_=motion;p[0]=(-1.,0.,0.)
    m.static_grid=grid();obstacle(m.static_grid,1.,0.,-1);obstacle(m.grid,1.,0.)
    tick(m,c)
    assert m.phase=='waiting_clearance' and not n.sent


def test_distant_preparation_waits_for_reobservation_without_zero_goal_loop(motion):
    m,n,c,p,_,_=motion;p[0]=(-1.,0.,0.)
    m.static_grid=grid();obstacle(m.grid,1.,0.)
    original=m.grid.clear
    m.grid.clear=lambda pose,rotation=False:original(pose,rotation) and (not rotation or pose[0]<0.)
    tick(m,c)
    assert m.phase=='staging' and m.next_phase=='approach'
    finish_action(m,n,c,p)
    assert m.phase=='waiting_clearance' and len(n.sent)==1
    for _ in range(5):m.isTaskComplete()
    assert len(n.sent)==1
    m.grid=grid();m.grid_stamp+=1;m.isTaskComplete()
    assert len(n.sent)==2 and m.phase=='staging'


def test_rotation_stall_cancels_native_action_then_drains(motion):
    m,n,c,p,_,_=motion
    m.mode='station';tick(m,c)
    p[0]=(*n.sent[-1][0][:2],-.125);n.complete=True
    m.odom=Odometry();m.odom.pose.pose.orientation.w=1.
    m.odom.header.stamp.sec=int(c[0]);m.odom_at=c[0]
    tick(m,c);assert m.phase=='aligning'
    command=Twist();command.angular.z=.20  # Positive target correction.
    for _ in range(15):
        c[0]+=.2;m.odom.header.stamp.sec=int(c[0]);m.odom.header.stamp.nanosec=round((c[0]%1)*1e9);m.odom_at=c[0]
        m._chassis_command(command)
        assert not m.isTaskComplete()
        if m.motion_stalled:break
    assert m.motion_stalled and n.cancels==1
    n.complete=True;n.result=TaskResult.CANCELED
    assert m.isTaskComplete() and m.getResult()==TaskResult.FAILED
    retry=StandbyRetryPolicy();retry.block_motion()
    assert not retry.can_attempt(c[0]+100.) and retry.status(c[0])=='blocked'
    retry.reset();assert retry.can_attempt(c[0])


@pytest.mark.parametrize('command',[0.,.09])
def test_safety_zero_or_reduced_final_command_does_not_claim_rotation_stall(motion,command):
    m,n,c,p,_,_=motion;m.mode='station';tick(m,c)
    p[0]=(*n.sent[-1][0][:2],-.125);n.complete=True
    m.odom=Odometry();m.odom.pose.pose.orientation.w=1.
    tick(m,c);assert m.phase=='aligning'
    value=Twist();value.angular.z=command
    for _ in range(20):
        c[0]+=.2;m.odom.header.stamp.sec=int(c[0]);m.odom.header.stamp.nanosec=round((c[0]%1)*1e9);m.odom_at=c[0]
        m._chassis_command(value);assert not m.isTaskComplete()
    assert not m.motion_stalled and n.cancels==0


@pytest.mark.parametrize('remaining',[3.1,5.1,8.8,19.])
def test_recorded_spin_tails_are_not_quartered_below_effective_speed(remaining):
    turn=RecoverySpin(2*math.pi,100.,0.,1)
    turn.travel=turn.target-math.radians(remaining);turn.finishing=True
    native=max(.1,min(.6,math.sqrt(2*.8*turn.remaining)))
    assert .19<=native*turn.speed_limit()<=.200001
    assert turn.speed_limit()<=1.


def test_spin_without_fresh_feedback_motion_does_not_burn_35_seconds():
    turn=RecoverySpin(2*math.pi,100.,0.,1)
    assert turn.tick(101.,blocked=False,available=True)=='run'
    assert turn.tick(102.6,blocked=False,available=True)=='cancel_failed'
    assert turn.reason=='rotation_no_progress'
    turn.phase='stopping';assert turn.speed_limit()==0.


def test_sidestep_gets_its_original_wait_before_spin_fallback(manager):
    n,c=manager;n.state='RECOVERING_GLOBAL';n.recovery_step='post_global'
    n.lateral_enabled=True;n._lateral_sensors_ready=lambda _:True
    n._try_lateral_recovery=lambda _:False;n._recovery_spin_clear=lambda _:True
    n._global_recovery_failed(c.now,'test')
    assert n.recovery_step=='second_guard_wait' and not n.started_spins
    c.now+=7.9;n._tick_second_guard_wait(c.now);assert not n.started_spins
    c.now+=.1;n._tick_second_guard_wait(c.now);assert n.started_spins==['global']


def test_manual_seed_during_post_global_restarts_verification_not_old_stall(manager):
    n,c=manager;n.state='RECOVERING_GLOBAL';n.recovery_step='post_global'
    n.manual_initial_pose_pending=False;n.global_verification_required=True
    n.get_clock=lambda:NS(now=lambda:NS(nanoseconds=int(c.now*1e9)))
    cancelled=[];n._cancel_navigation=lambda:cancelled.append(True)
    n._set_state=lambda state,reason:(setattr(n,'state',state),setattr(n,'state_reason',reason))
    n._external_initial_pose_callback(None)
    assert n.state=='VERIFYING' and n.recovery_step is None
    assert not n.global_verification_required and n.last_amcl_at==0 and n.stable_samples==0
    old=PoseWithCovarianceStamped();old.header.stamp.sec=99
    n._amcl_pose_callback(old)
    assert n.last_amcl_at==0 and n.manual_seed_after_ns==100_000_000_000
    assert cancelled==[True] and not n.manual_reasons
    new=PoseWithCovarianceStamped();new.header.stamp.sec=100;new.header.stamp.nanosec=1
    new.pose.pose.orientation.w=1.
    n._amcl_pose_callback(new)
    assert n.last_amcl_at==100. and n.manual_seed_after_ns==0 and not n.ready


def test_real_obstacle_in_both_sweeps_can_fall_back_without_visibility_wait(manager):
    n,c=manager;n.state='RECOVERING_GLOBAL';n.recovery_step='post_global'
    n.lateral_enabled=True;n._lateral_sensors_ready=lambda _:True
    n._try_lateral_recovery=lambda _:False;n._recovery_spin_clear=lambda _:True
    n.lateral_path_reports={str(d):dict(scan=dict(reason='obstacle_or_occlusion')) for d in (1,-1)}
    n._global_recovery_failed(c.now,'test')
    assert n.started_spins==['global']
