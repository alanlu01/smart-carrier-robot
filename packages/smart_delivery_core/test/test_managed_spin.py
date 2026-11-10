"""Exercise action cancellation handshakes and independent stop proof."""
import math
from types import SimpleNamespace as NS
from action_msgs.msg import GoalStatus
from test_localization_recovery import manager, fresh
from smart_delivery_core.localization_manager import LocalizationManager
from smart_delivery_core.recovery_spin import RecoverySpin


def prepare_turn(node, clock):
    node.state='RECOVERING_GLOBAL'
    node.managed_spin=None
    node.spin_time_allowance=35
    node.last_odom_pose=(0.,0.,0.)
    node.latest_odom_stamp_ns=int(clock.now*1e9)
    node.last_motion_command_at=node.last_odom_motion_at=clock.now-2.
    node.get_clock=lambda:NS(now=lambda:NS(nanoseconds=int(clock.now*1e9)))
    node.recovery_feedback=NS(last_complete_at=clock.now,is_stale=lambda *_:False,
                              sample_age=lambda *_:0.)
    node._heading_restore_command=lambda **_:None
    node._recovery_spin_clear=lambda _:True
    node._lateral_sensor_blockers=lambda _:[]
    node.navigation_health={'state':'ready'}
    node.navigation_health_at=clock.now
    node.caps=[]
    node.speed_limit_publisher=NS(publish=lambda m:node.caps.append(m.data))
    node.goals=[]
    node.spin_client=NS(send_goal_async=lambda g:(node.goals.append(g) or NS(add_done_callback=lambda _:None)))
    node.spin_sequence=[2*math.pi]
    node._send_next_spin()
    return node.managed_spin


def update(node,clock,now):
    fresh(node,clock,now)
    node.latest_odom_stamp_ns=int(now*1e9)
    node.navigation_health_at=now
    node._tick_managed_spin()


def test_pause_requires_terminal_ack_stop_proof_then_resumes_remainder(manager):
    node,clock=manager
    turn=prepare_turn(node,clock)
    for i in range(1,61):
        yaw=math.radians(5*i)
        turn.observe(math.atan2(math.sin(yaw),math.cos(yaw)),int((100+i*.1)*1e9),.5)
    node.cancel_count=0
    def cancel():node.cancel_count+=1
    node.managed_spin_handle=NS(cancel_goal_async=cancel)
    node._stop_managed_spin('near_obstacle')
    update(node,clock,101.)
    assert node.cancel_count==1 and len(node.goals)==1 and node.caps[-1]==0.
    node._spin_result(NS(result=lambda:NS(status=GoalStatus.STATUS_CANCELED)))
    update(node,clock,102.)
    assert len(node.goals)==1  # terminal ACK alone is insufficient
    node.recovery_stop=NS(ready=lambda _:True)
    update(node,clock,103.)
    update(node,clock,104.1)
    assert len(node.goals)==2
    assert node.goals[-1].target_yaw == __import__('pytest').approx(math.pi/3)
    assert turn.resumes==1 and not turn.cancel_confirmed
    assert not node.ready


def test_missing_stop_or_cancel_ack_never_unlocks_on_timeout(manager):
    node,clock=manager
    prepare_turn(node,clock)
    node._stop_managed_spin('data_unavailable')
    update(node,clock,106.)
    assert node.manual_reasons and len(node.goals)==1
    assert node.managed_spin is not None and node.caps[-1]==0.


def test_supervised_spin_blocks_amcl_shortcut(manager):
    node,clock=manager
    prepare_turn(node,clock)
    node._mark_localized=lambda *_:(_ for _ in ()).throw(AssertionError('early release'))
    for _ in range(7):
        from test_localization_recovery import amcl
        amcl(node)
    assert not node.ready


def test_near_convergence_wait_is_bounded_and_requires_physical_stop(manager):
    node,clock=manager
    node.observation_clock.elapsed=40.
    clock.now=110.
    node._verification_data_blockers=lambda _:[]
    node._map_match_has_recovered=node._global_pose_is_verified=lambda _:True
    node.latest_map_score=.98
    node.quality_history.samples.extend(dict(at=104.+i,pose=(2.,5.,0.),xy_std=.29-i*.004,yaw_std=.1)
                                        for i in range(7))
    assert not node._near_convergence_grace(clock.now)
    node.recovery_stop=NS(ready=lambda _:True)
    assert node._near_convergence_grace(clock.now)
    node.observation_clock.elapsed=45.
    assert not node._near_convergence_grace(clock.now)
