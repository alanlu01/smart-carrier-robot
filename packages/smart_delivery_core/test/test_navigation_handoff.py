"""Goal ownership, fault recovery, and recorded stationary drift regressions."""
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from geometry_msgs.msg import PoseStamped
from smart_delivery_core.navigation_policy import navigation_policy, PROFILES
from smart_delivery_core.navigation_action_health import ActionRecovery
from smart_delivery_core.navigation_transcript import NavigationTranscript
from smart_delivery_core.standby_idle_clock import StandbyIdleClock
from smart_delivery_core.standby_retry import StandbyRetryPolicy


def test_owned_policy_uses_exact_installed_files_including_individual_symlinks(tmp_path):
    source=tmp_path/'source'; share=tmp_path/'install'; other=tmp_path/'other'
    for directory in (source,share/'behavior_trees',other):directory.mkdir(parents=True)
    for name in ('smart_delivery_navigate.xml',*PROFILES):
        (source/name).write_text('<root/>')
        (share/'behavior_trees'/name).symlink_to(source/name)
    pose=PoseStamped();pose.header.frame_id='map';pose.header.stamp.sec=123
    pose.pose.orientation.w=1.
    for name in ('smart_delivery_navigate.xml',*PROFILES):
        tree,payload=navigation_policy(pose,str(share/'behavior_trees'/name),share)
        p=json.loads(payload)
        assert tree==str(share/'behavior_trees/smart_owned_navigate.xml')
        assert p['stamp_ns']==123000000000
        assert p['controller']==PROFILES.get(name,('FollowPath','general_goal_checker'))[0]
        assert p['station']==(name in PROFILES)
    assert navigation_policy(pose,'',share) is not None
    (other/'station_y.xml').write_text('<root/>')
    assert navigation_policy(pose,str(other/'station_y.xml'),share) is None
    assert navigation_policy(pose,str(share/'behavior_trees/people_observe.xml'),share) is None


class Future:
    def __init__(self,success=True,done=True):self.success,self.complete=success,done
    def done(self):return self.complete
    def result(self):return NS(success=self.success)


def fault(tmp_path):
    path=tmp_path/'fault.json';path.write_text(json.dumps(dict(sequence=1,wall_time=100.,reason='ack timeout')))
    p=ActionRecovery(path);p.observe();return p,path


def tick(p,t,calls,**changes):
    args=dict(stopped=True,lifecycle_ready=True,service_ready=True,
              call=lambda command:(calls.append(command) or Future()))
    args.update(changes);p.tick(t,**args)


def hold(p,t,calls,**changes):
    for i in range(11):tick(p,t+i*.105,calls,**changes)


def recover(p,t,calls):
    assert p.request(t)
    hold(p,t,calls)
    assert p.state=='resetting' and calls[-1]==3
    tick(p,t+1.1,calls);assert p.state=='waiting_start'
    tick(p,t+1.2,calls);assert p.state=='starting' and calls[-1]==0
    tick(p,t+1.3,calls);assert p.state=='verifying'
    tick(p,t+1.4,calls);assert p.state=='healthy'


def test_recovery_requires_request_stop_then_acknowledged_reset_startup_and_verification(tmp_path):
    p,_=fault(tmp_path);calls=[]
    tick(p,100.,calls);assert not calls
    assert p.request(101.) and not p.request(101.1)
    tick(p,101.,calls,stopped=False);tick(p,104.,calls,stopped=False);assert not calls
    for i in range(10):tick(p,104.1+i*.1,calls)
    assert not calls
    tick(p,105.2,calls);assert calls==[3]
    tick(p,105.3,calls);assert p.state=='waiting_start'
    tick(p,105.4,calls,stopped=False);assert calls==[3]
    tick(p,105.5,calls);assert calls==[3,0]
    tick(p,105.6,calls);tick(p,105.7,calls,lifecycle_ready=False)
    assert p.state=='verifying'
    tick(p,105.8,calls);assert p.recovered_generation==1 and p.state=='healthy'


def test_reset_rpc_timeout_cannot_startup_later_or_recover_from_new_log(tmp_path):
    p,path=fault(tmp_path);calls=[];late=Future(done=False)
    assert p.request(0.)
    hold(p,0.,calls,call=lambda c:(calls.append(c) or late))
    tick(p,12.,calls);assert p.state=='manual_required' and calls==[3]
    late.complete=True
    path.write_text(json.dumps(dict(sequence=2,wall_time=200.,reason='another timeout')))
    tick(p,13.,calls)
    assert p.state=='manual_required' and calls==[3] and not p.request(14.)


def test_no_stop_proof_no_recovery_and_no_restart_storm_after_limit(tmp_path):
    p,path=fault(tmp_path);calls=[]
    assert p.request(0.);tick(p,10.,calls,stopped=False)
    assert p.state=='manual_required' and not calls
    p,path=fault(tmp_path)
    recover(p,20.,calls)
    path.write_text(json.dumps(dict(sequence=2,wall_time=200.,reason='next fault')));p.observe()
    recover(p,30.,calls)
    path.write_text(json.dumps(dict(sequence=3,wall_time=300.,reason='third fault')));p.observe()
    assert not p.request(40.) and p.state=='manual_required' and calls==[3,0,3,0]


@pytest.mark.parametrize('success',[False,None])
def test_failed_or_throwing_lifecycle_service_fails_closed(tmp_path,success):
    p,_=fault(tmp_path);calls=[];assert p.request(0.);tick(p,0.,calls)
    def call(c):
        calls.append(c)
        if success is None:raise RuntimeError('service disappeared')
        return Future(success=success)
    hold(p,.1,calls,call=call);tick(p,1.3,calls)
    assert p.state=='manual_required' and calls==[3]


def test_scheduler_gap_cannot_count_as_continuous_stop_proof(tmp_path):
    p,_=fault(tmp_path);calls=[];assert p.request(0.)
    tick(p,0.,calls);tick(p,5.,calls)
    assert not calls
    hold(p,5.1,calls);assert calls==[3]


def test_transcript_recognizes_split_ack_error_but_not_normal_cancel_halt(tmp_path):
    path=tmp_path/'action.json';w=NavigationTranscript(tmp_path,action_health_file=path)
    e=NS(cmd=['component_container_isolated','__node:=nav2_container'],pid=123,from_stderr=True)
    e.text=b'Failed to get result for compute_path_to_pose in node halt!\n';w.output(e)
    e.text=b'Timed out while waiting for action server to acknow';w.output(e)
    e.text=b'ledge goal request for compute_path_to_pose\n';w.output(e);w.close()
    p=json.loads(path.read_text());assert p['sequence']==1 and 'compute_path_to_pose' in p['reason']


POINT=dict(name='CYCU EE',x=2.,y=5.18,yaw=0.)


def idle(c,t,offset=0.,**changes):
    args=dict(parked=True,safe=True,stationary=True,
        pose=dict(x=2.+offset,y=5.18,yaw=0.),odom=(0.,0.,0.),trusted_map_correction=True)
    args.update(changes);return c.update(t,**args)


def test_gradual_stationary_map_drift_preserves_elapsed_after_bounded_confirmation():
    c=StandbyIdleClock();c.mark_parked(POINT)
    for i in range(141):idle(c,i*.5,min(.25,i*.5*.004))
    assert c.correction_used and c.elapsed>=60. and c.point==POINT
    assert c.map_reference['x']>2.20


@pytest.mark.parametrize('changes',[dict(trusted_map_correction=False),dict(pose=dict(x=3.,y=5.18,yaw=0.))])
def test_uncertain_or_large_map_change_does_not_rebase_auxiliary_clock(changes):
    c=StandbyIdleClock();c.mark_parked(POINT)
    for i in range(121):idle(c,i*.5,min(.25,i*.5*.004),**changes)
    assert not c.correction_used and c.elapsed==0.


def test_abrupt_map_jump_or_real_wheel_motion_cannot_use_stationary_drift_exception():
    c=StandbyIdleClock();c.mark_parked(POINT);idle(c,0.)
    for i in range(1,15):idle(c,i*.5,.25)
    assert not c.correction_used and c.elapsed==0.
    c.mark_parked(POINT);idle(c,10.)
    assert not idle(c,10.5,.01,odom=(.03,0.,0.))
    assert c.correction_disqualified


def test_recovery_rearms_exhausted_parking_only_once_and_never_a_motor_stall():
    p=StandbyRetryPolicy()
    for i in range(3):p.failed(i)
    assert p.rearm_after_recovery(('localized',1)) and p.can_attempt(100.)
    for i in range(3):p.failed(i)
    assert not p.rearm_after_recovery(('localized',1))
    assert not p.rearm_after_recovery(('localized',2)) and p.status(100.)=='blocked'
    p.reset();p.block_motion();assert not p.rearm_after_recovery(('localized',3))
