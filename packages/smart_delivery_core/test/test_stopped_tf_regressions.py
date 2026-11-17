"""Recorded braking and TF outage regressions; fake clock/ROS, no hardware."""
import math
from types import SimpleNamespace as NS
import pytest
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import TaskResult
from test_localization_recovery import manager
from test_station_navigation import motion,tick,grid,obstacle
from smart_delivery_core.settled_motion import StationaryPoseGuard
from smart_delivery_core.pipeline_recheck import TrustedPipelineReference


def test_source_boundary_ignores_delayed_braking_pose_but_detects_later_kidnap():
    g=StationaryPoseGuard()
    args=dict(distance=.5,angle=math.radians(20))
    assert not g.observe((0.,0.,-1.18),100_000_000_000,100_000_000_000,settled=False,**args)
    assert not g.observe((0.,0.,-1.18),100_000_000_000,101_000_000_000,settled=True,**args)
    assert not g.observe((.15,0.,-1.62),100_500_000_000,102_000_000_000,settled=True,**args)
    assert g.anchor is None
    assert not g.observe((.15,0.,-1.62),102_000_000_000,103_000_000_000,settled=True,**args)
    assert g.observe((1.15,0.,-1.62),103_000_000_000,104_000_000_000,settled=True,**args)
    g.observe((0.,0.,0.),104_000_000_000,104_000_000_000,settled=False,**args)
    assert g.anchor is None


def test_manager_does_not_call_braking_amcl_drift_external_move(manager):
    n,c=manager;n.state='LOCALIZED';n.last_motion_command_at=98.
    n.pose_jump_distance=.5;n.pose_jump_angle=math.radians(20.)
    n._observe_covariance_probe=lambda *args:None;n._persist_trusted_pose=lambda **kw:None
    def pose(stamp,yaw):
        m=PoseWithCovarianceStamped();m.header.stamp.sec=stamp
        m.pose.pose.orientation.z=math.sin(yaw/2);m.pose.pose.orientation.w=math.cos(yaw/2)
        m.pose.covariance[0]=m.pose.covariance[7]=.04;m.pose.covariance[35]=.01
        n.latest_odom_at=c.now;n._amcl_pose_callback(m)
    n.recovery_stop=NS(ready=lambda _:False);pose(100,-1.18)
    c.now=101.;n.recovery_stop=NS(ready=lambda _:True);pose(100,-1.62)
    c.now=102.;pose(100,-1.62)
    assert n.stationary_amcl_anchor is None and not n.started_spins
    c.now=103.;pose(102,-1.62)
    c.now=104.;pose(103,-2.12)
    assert len(n.started_spins)==1 and '實際停穩後' in n.started_spins[0]


@pytest.mark.parametrize('change', ['source','clock','motion','yaw','nan'])
def test_trusted_cache_rejects_old_epoch_or_unbounded_motion(change):
    r=TrustedPipelineReference(100.,100_000_000_000,(0.,0.,0.),(0.,0.,0.),.2,.1,.9)
    now=105.;ns=105_000_000_000;odom=(0.,0.,0.)
    if change=='source':ns=113_000_000_000
    if change=='clock':ns=90_000_000_000
    if change=='motion':odom=(.21,0.,0.)
    if change=='yaw':odom=(0.,0.,.18)
    if change=='nan':odom=(float('nan'),0.,0.)
    assert r.select(now,ns,odom,.20,.174533) is None


def start_alignment(motion):
    m,n,c,p,stopped,permission=motion;m.mode='station';tick(m,c)
    p[0]=(*n.sent[-1][0][:2],-.2);n.complete=True;tick(m,c)
    assert m.phase=='aligning'
    m.motion_odom_anchor=(0.,0.,0.)
    return m,n,c,p,stopped,permission


def fresh_sources(m,c):
    stamp=int(c[0]*1e9)
    m.pose_gate=dict(stamp_ns=stamp)
    m.grid_stamp=stamp
    m.odom=Odometry();m.odom.pose.pose.orientation.w=1.
    m.odom.header.stamp.sec=stamp//10**9;m.odom.header.stamp.nanosec=stamp%10**9;m.odom_at=c[0]


def pause(motion):
    m,n,c,p,stopped,permission=start_alignment(motion)
    fresh_sources(m,c)
    real=m._pose;m._pose=lambda:None
    assert not m.isTaskComplete() and m.phase=='alignment_pausing' and n.cancels==1
    assert not m.cancelled and len(n.sent)==2
    m._pose=real
    return m,n,c,p,stopped,permission


def test_alignment_drain_stop_new_sources_then_resume_once(motion):
    m,n,c,p,stopped,permission=pause(motion)
    c[0]+=.3;fresh_sources(m,c)
    assert not m.isTaskComplete() and len(n.sent)==2  # No terminal ACK yet.
    n.complete=True;n.result=TaskResult.CANCELED;stopped[0]=False
    assert not m.isTaskComplete() and m.phase=='alignment_wait'
    stopped[0]=True;c[0]+=.1;fresh_sources(m,c);m.isTaskComplete()
    c[0]+=.6;fresh_sources(m,c);m.isTaskComplete()
    assert m.phase=='aligning' and len(n.sent)==3 and m.alignment_attempts==2
    m._pose=lambda:None;m.isTaskComplete()
    assert m.cancelled and n.cancels==2 and m.failure_kind=='alignment_data_repeated'


@pytest.mark.parametrize('reason',['cancel','permission','displacement','blocked','timeout'])
def test_alignment_pause_cannot_bypass_cancel_or_physical_safety(motion,reason):
    m,n,c,p,stopped,permission=pause(motion)
    n.complete=True;n.result=TaskResult.CANCELED
    if reason=='cancel':m.cancelTask()
    if reason=='permission':permission[0]=False
    if reason=='timeout':c[0]=m.deadline+.1
    else:c[0]+=.1
    fresh_sources(m,c)
    if reason=='displacement':m.odom.pose.pose.position.x=.051
    if reason=='blocked':m.grid.turn_clear=lambda *args:False
    m.isTaskComplete()
    if m.running:m.isTaskComplete()
    assert not m.running and m.result!=TaskResult.SUCCEEDED and len(n.sent)==2


def test_unknown_static_and_dynamic_blocker_classification(motion):
    m,n,c,p,_,_=motion;m.static_grid=grid()
    obstacle(m.grid,.20,.10,-1)
    m._wait_clearance('test','prepare')
    assert m.clearance_kind=='unknown' and m.clearance_blocker['position']
    m.grid=grid();obstacle(m.grid,.20,.10)
    m._wait_clearance('test','prepare');assert m.clearance_kind=='dynamic_costmap'
    obstacle(m.static_grid,.20,.10)
    m._wait_clearance('test','prepare');assert m.clearance_kind=='static'


def test_amcl_future_tf_tolerance_does_not_disable_trusted_capture(manager):
    from smart_delivery_core import localization_manager as module
    n,c=manager;n.latest_odom_at=c.now;n.last_odom_pose=(0.,0.,0.)
    n.latest_pose_observed_at=c.now;n.latest_odom_stamp_ns=int(c.now*1e9)
    n.tf_buffer=NS(lookup_transform=lambda *a,**kw:NS(
        header=NS(stamp=NS(sec=101,nanosec=0)),
        transform=NS(translation=NS(x=1.,y=2.),rotation=NS(x=0.,y=0.,z=0.,w=1.))))
    module.LocalizationManager._capture_pipeline_reference(n,c.now)
    assert n.pipeline_trusted_reference.select(c.now,100_000_000_000,(0.,0.,0.),.2,.17)==(1.,2.,0.)


@pytest.mark.parametrize('source',['amcl','odom'])
def test_cache_does_not_trust_old_source_with_fresh_receipt(manager,source):
    from smart_delivery_core import localization_manager as module
    n,c=manager;n.latest_odom_at=c.now;n.last_odom_pose=(0.,0.,0.)
    n.latest_pose_observed_at=c.now if source!='amcl' else c.now-3.
    n.latest_odom_stamp_ns=int((c.now if source!='odom' else c.now-1.)*1e9)
    n.tf_buffer=NS(lookup_transform=lambda *a,**kw:NS(
        header=NS(stamp=NS(sec=101,nanosec=0)),
        transform=NS(translation=NS(x=1.,y=2.),rotation=NS(x=0.,y=0.,z=0.,w=1.))))
    module.LocalizationManager._capture_pipeline_reference(n,c.now)
    assert not hasattr(n,'pipeline_trusted_reference')
