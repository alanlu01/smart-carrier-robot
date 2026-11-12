"""Adaptive station behaviour: finite waits, strict arrival and action ownership."""
import math
from types import SimpleNamespace as NS

from nav2_simple_commander.robot_navigator import TaskResult
from test_station_navigation import motion, tick, finish_action, grid, obstacle


def test_parked_flag_alone_never_forces_lateral_departure(motion):
    manager,node,clock,pose,stopped,permission=motion
    manager.parked=(0.,0.,0.)
    tick(manager,clock)
    assert manager.mode=='normal' and manager.phase=='direct'
    assert len(node.sent)==1 and node.sent[0][0]==(1.,0.,0.)
    assert manager.decision_time_ms<50.


def test_parallel_wall_does_not_require_unused_full_rotation():
    g=grid()
    for x in range(-50,60):obstacle(g,x*.025,.25)
    assert not g.clear((0.,0.,0.),rotation=True)
    assert g.normal_route_safe((0.,0.,0.),(1.,0.,0.))
    assert not g.normal_route_safe((0.,0.,0.),(1.,0.,math.pi/2))


def test_swept_turn_rejects_obstacle_between_endpoint_orientations():
    g=grid(.005)
    # Expand the test map because its default spans only one metre.
    g.x=g.y=-.5
    obstacle(g,.31,.015)
    assert g.clear((0.,0.,0.)) and g.clear((0.,0.,math.pi/2))
    assert not g.turn_clear((0.,0.,0.),math.pi/2)


def test_seven_degree_staging_drift_gets_safe_bounded_alignment(motion):
    manager,node,clock,pose,stopped,permission=motion
    manager.mode='station';tick(manager,clock)
    stage=node.sent[-1][0]
    pose[0]=(stage[0]+.07,stage[1],-.125)
    node.complete=True;tick(manager,clock)
    assert manager.phase=='aligning' and 'station_align.xml' in node.sent[-1][1]
    assert manager.alignment_attempts==1
    finish_action(manager,node,clock,pose)
    assert manager.phase=='entry' and node.sent[-1][0]==(1.,0.,0.)
    finish_action(manager,node,clock,pose)
    assert manager.getResult()==TaskResult.SUCCEEDED


def test_final_heading_tolerance_is_not_relaxed(motion):
    manager,node,clock,pose,stopped,permission=motion
    tick(manager,clock);pose[0]=(1.,0.,.125);node.complete=True
    tick(manager,clock)
    assert manager.getResult()!=TaskResult.SUCCEEDED and manager.phase=='aligning'
    finish_action(manager,node,clock,pose)
    assert manager.getResult()==TaskResult.SUCCEEDED
    assert manager.final_corrections==1


def test_person_blockage_waits_for_new_costmap_then_rechecks(motion):
    manager,node,clock,pose,stopped,permission=motion
    manager.mode='station'
    manager.grid.departure=lambda *_:None
    manager.grid.clear=lambda *_a,**_k:False
    tick(manager,clock)
    assert manager.phase=='waiting_clearance' and not node.sent
    deadline=manager.deadline
    for _ in range(4):clock[0]+=.6;manager.isTaskComplete()
    assert manager.deadline==deadline and not node.sent
    manager.grid=grid();manager.grid_stamp+=1
    manager.isTaskComplete()
    assert len(node.sent)==1 and manager.phase=='staging'


def test_persistent_blockage_is_finite_and_not_a_preflight_data_error(motion):
    manager,node,clock,pose,stopped,permission=motion
    manager.mode='station';manager.grid.staging=lambda *_a,**_k:None
    tick(manager,clock)
    assert manager.phase=='waiting_clearance'
    clock[0]+=31.
    assert manager.isTaskComplete() and manager.clearance_timeout
    assert not manager.preflight_timeout and not manager.data_unavailable
    assert not node.sent


def test_mode_switch_waits_for_native_terminal_and_happens_only_once(motion):
    manager,node,clock,pose,stopped,permission=motion
    node.navigation_actions_ready=lambda:True
    tick(manager,clock)
    manager._normal_plan_unsafe=lambda *_:True
    assert not manager.isTaskComplete() and node.cancels==1 and len(node.sent)==1
    assert not manager.cancelled and manager.replan_pending
    manager.isTaskComplete()
    assert node.cancels==1 and len(node.sent)==1
    node.complete=True;node.result=TaskResult.CANCELED
    tick(manager,clock)
    assert manager.mode=='station' and manager.mode_switches==1 and len(node.sent)==2


def test_new_obstacle_cancels_checked_translation_without_releasing_action(motion):
    manager,node,clock,pose,stopped,permission=motion
    manager.mode='station';tick(manager,clock);finish_action(manager,node,clock,pose)
    obstacle(manager.grid,.85,0.)
    assert not manager.isTaskComplete() and node.cancels==1
    node.complete=True;node.result=TaskResult.CANCELED
    assert manager.isTaskComplete() and manager.getResult()==TaskResult.FAILED
    assert '障礙' in manager.error_details()[1]


def test_tf_failure_does_not_trigger_station_fallback(motion):
    manager,node,clock,pose,stopped,permission=motion
    tick(manager,clock);node.complete=True;node.result=TaskResult.FAILED
    node.result_future=NS(result=lambda:NS(result=NS(error_code=102,error_msg='TF error')))
    assert manager.isTaskComplete() and manager.mode_switches==0 and len(node.sent)==1


def test_stopped_ordinary_yaw_tail_drains_then_uses_bounded_alignment(motion):
    manager,node,clock,pose,stopped,permission=motion
    node.navigation_actions_ready=lambda:True
    tick(manager,clock);pose[0]=(.95,0.,.125)
    assert not manager.isTaskComplete() and not node.cancels
    clock[0]+=.6
    assert not manager.isTaskComplete() and node.cancels==1 and len(node.sent)==1
    assert manager.final_heading_pending and manager.replan_pending
    node.complete=True;node.result=TaskResult.CANCELED
    tick(manager,clock)
    assert manager.phase=='aligning' and 'station_align.xml' in node.sent[-1][1]
    node.result=TaskResult.SUCCEEDED
    finish_action(manager,node,clock,pose)
    assert manager.getResult()==TaskResult.SUCCEEDED and len(node.sent)==2


def test_moving_or_unsafe_heading_tail_does_not_interrupt_normal_navigation(motion):
    manager,node,clock,pose,stopped,permission=motion
    tick(manager,clock);pose[0]=(.95,0.,.125);stopped[0]=False
    tick(manager,clock);assert not node.cancels
    stopped[0]=True;manager.grid.turn_clear=lambda *_:False
    tick(manager,clock);assert not node.cancels and not manager.final_heading_pending
