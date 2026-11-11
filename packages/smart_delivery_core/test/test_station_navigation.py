"""Station-only routing, escape geometry, safety gates and action ownership."""
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
from xml.etree import ElementTree

import pytest
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_simple_commander.robot_navigator import TaskResult

from smart_delivery_core.station_geometry import StationGrid
from smart_delivery_core.station_nav_profile import station_parameters
from smart_delivery_core import station_navigation as module

ROOT = Path(__file__).parents[1]


def grid(resolution=.025):
    info = NS(resolution=resolution,width=200,height=200,
        origin=NS(position=NS(x=-2.5,y=-2.5),orientation=NS(x=0.,y=0.,z=0.,w=1.)))
    return StationGrid(NS(info=info,data=[0]*40000))


def obstacle(g,x,y,value=100):
    g.data[math.floor((y-g.y)/g.resolution)*g.width+math.floor((x-g.x)/g.resolution)] = value


@pytest.mark.parametrize('angle',[0.,math.pi/2,math.pi,-math.pi/2])
def test_body_sweep_blocks_thin_obstacles_in_every_direction(angle):
    g = grid()
    end = (.5*math.cos(angle),.5*math.sin(angle),0.)
    assert g.segment((0.,0.,0.),end)
    obstacle(g,.35*math.cos(angle),.35*math.sin(angle))
    assert not g.segment((0.,0.,0.),end)


def test_unknown_offmap_and_unfinished_rotation_cannot_be_called_safe():
    g = grid()
    obstacle(g,.40,0.,-1)
    assert not g.segment((0.,0.,0.),(.5,0.,0.))
    assert not g.clear((2.4,0.,0.))
    assert not g.segment((0.,0.,.3),(.2,0.,0.))


def test_wall_on_left_allows_right_escape_but_blocks_left_and_rotation():
    g = grid()
    for x in range(-20,21): obstacle(g,x*.025,.25)
    assert g.clear((0.,0.,0.))
    assert not g.clear((0.,0.,0.),rotation=True)
    exit = g.departure((0.,0.,0.),(0.,-1.,0.))
    assert exit is not None and exit[1]<0 and abs(exit[0])<1e-6
    assert g.segment((0.,0.,0.),exit)
    assert not g.segment((0.,0.,0.),(0.,.35,0.))


def test_fully_surrounded_station_has_no_staging_or_escape():
    g = grid()
    for i in range(-20,21):
        obstacle(g,i*.025,.3);obstacle(g,i*.025,-.3)
        obstacle(g,.3,i*.025);obstacle(g,-.3,i*.025)
    assert g.staging((0.,0.,0.),(1.,1.,0.)) is None
    assert g.departure((0.,0.,0.),(1.,1.,0.)) is None


def test_recorded_five_cm_cells_allow_three_cm_margin_without_shrinking_body():
    # Equivalent local geometry to the recorded API goal (-3,5): nearest
    # occupied centers at y=+0.224, actual cell edge about +0.199.
    g=grid(.05)
    for i in range(-10,11):obstacle(g,i*.05,.225)
    assert g.clear((0.,0.,0.))
    assert g.staging((0.,0.,0.),(0.,-1.,0.)) is not None
    assert g.clear((0.,.009,0.))  # 3.1 cm remains between body and cell.
    assert not g.clear((0.,.010,0.))  # Touching the 3 cm margin is blocked.
    assert not g.clear((0.,.011,0.))  # Only 2.9 cm remains.
    assert not g.clear((0.,0.,0.),rotation=True)


def single_cell_at(x,y,value=100):
    g=grid(.05)
    g.x,g.y=x-100.5*g.resolution,y-100.5*g.resolution
    g.data[100*g.width+100]=value
    return g


@pytest.mark.parametrize('value',[100,-1])
def test_rotated_rectangle_tests_grid_axes_as_well_as_body_axes(value):
    assert single_cell_at(.350,.040,value).clear((0.,0.,math.pi/4))
    assert not single_cell_at(.330,.040,value).clear((0.,0.,math.pi/4))


def test_rotation_checks_the_cell_square_instead_of_its_half_diagonal():
    radius=math.hypot(.25,.19)
    assert single_cell_at(radius+.030,0.).clear((0.,0.,0.),rotation=True)
    assert not single_cell_at(radius+.024,0.).clear((0.,0.,0.),rotation=True)


def test_footprint_that_fits_near_map_edge_does_not_require_a_whole_rotation_circle():
    g=grid(.05)
    assert g.clear((g.x+.28,0.,0.))
    assert not g.clear((g.x+.28,0.,0.),rotation=True)


@pytest.mark.parametrize('map_yaw',[0.,.3,math.pi/2])
def test_cell_intersections_respect_map_origin_rotation(map_yaw):
    g=grid(.05);g.yaw=map_yaw
    for col in range(45,56):g.data[54*g.width+col]=100
    c,s=math.cos(map_yaw),math.sin(map_yaw)
    pose=(g.x+c*2.5-s*2.5,g.y+s*2.5+c*2.5,map_yaw)
    assert g.clear(pose)
    assert not g.clear((pose[0]-s*.011,pose[1]+c*.011,pose[2]))


def parameters():
    return yaml.safe_load((ROOT/'config/my_nav2_params.yaml').read_text(encoding='utf-8'))


def test_station_overlay_leaves_normal_controller_smoother_and_footprint_exactly_unchanged():
    original=parameters();updated=station_parameters(original)
    a,b=original['controller_server']['ros__parameters'],updated['controller_server']['ros__parameters']
    assert a['FollowPath']==b['FollowPath'] and a['FollowPath']['vy_std']==.05
    for key in ('velocity_smoother','local_costmap','global_costmap','localization_manager'):
        assert original[key]==updated[key]
    for name in ('StationX','StationY','StationExitX','StationExitY'):
        profile=b[name];cap=.15
        assert math.hypot(profile['vx_max'],profile['vy_max'])<=cap+1e-9
        assert profile['wz_max']==.15
        assert profile['CostCritic']['consider_footprint']
        assert not profile['PreferForwardCritic']['enabled']
        assert not profile['PathAngleCritic']['enabled']
        if 'Exit' not in name:
            assert profile['plugin']=='smart_station_controller::StationMPPI'
            assert profile['station']==dict(maximum_speed=.15,slow_speed=.06,slow_distance=.20,ramp_distance=.40)
    assert original['controller_server']['ros__parameters']['controller_plugins']==['FollowPath']


def inside(point, polygon):
    signs=[]
    for a,b in zip(polygon,polygon[1:]+polygon[:1]):
        signs.append((b[0]-a[0])*(point[1]-a[1])-(b[1]-a[1])*(point[0]-a[0]))
    return all(s>=-1e-8 for s in signs) or all(s<=1e-8 for s in signs)


def selected(vx,vy,w):
    p=station_parameters(parameters())['collision_monitor']['ros__parameters']['NearFootprintStop']
    v,a=math.hypot(vx,vy),math.atan2(vy,vx)
    return next(json.loads(p[n]['points']) for n in p['velocity_polygons']
        if p[n]['linear_min']<=v<=p[n]['linear_max'] and p[n]['theta_min']<=w<=p[n]['theta_max']
        and p[n]['direction_start_angle']<=a<=p[n]['direction_end_angle'])


def test_direction_guard_allows_away_motion_and_stops_toward_near_wall():
    # Left-side wall outside the body: right motion may leave it, left may not.
    assert not inside((0.,.235),selected(0.,-.15,0.))
    assert inside((0.,.235),selected(0.,.15,0.))
    assert inside((.29,0.),selected(.15,0.,0.))
    assert not inside((.29,0.),selected(-.15,0.,0.))
    for v in ((.3,.15,.8),(-.15,.1,0.),(0.,0.,.5),(0.,-.06,0.),(.6,0.,0.)):
        polygon=selected(*v)
        assert all(inside(p,polygon) for p in ((.22,.16),(.22,-.16),(-.22,.16),(-.22,-.16)))


def test_station_trees_use_explicit_profiles_no_spin_backup_or_goal_tolerance_change():
    for name in ('x','y','exit_x','exit_y','align'):
        root=ElementTree.parse(ROOT/f'behavior_trees/station_{name}.xml')
        assert not any(root.findall('.//'+tag) for tag in ('Spin','BackUp','ControllerSelector','RecoveryNode'))
        follow=root.find('.//FollowPath')
        assert follow.get('goal_checker_id')==('station_alignment_checker' if name=='align' else 'general_goal_checker')
        expected='StationAlign' if name=='align' else 'Station'+('Exit' if 'exit' in name else '')+name[-1].upper()
        assert follow.get('controller_id')==expected


@pytest.fixture
def motion(monkeypatch):
    clock=[100.];pose=[(0.,0.,0.)];stopped=[True];permission=[True]
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(module.rclpy,'spin_once',lambda *_a,**_k:None)
    node=NS(sent=[],complete=False,result=TaskResult.SUCCEEDED,cancels=0,messages=[],logs=[])
    node.create_publisher=lambda *_a,**_k:NS(publish=lambda msg:node.messages.append(json.loads(msg.data)))
    node.create_subscription=lambda *_a,**_k:None
    node.get_logger=lambda:NS(info=node.logs.append)
    node.get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:PoseStamped().header.stamp,nanoseconds=int(clock[0]*1e9)))
    def send(goal,behavior_tree=''):
        node.sent.append((module.coordinates(goal),behavior_tree));node.complete=False;return True
    def cancel():node.cancels+=1
    node.goToPose=send;node.cancelTask=cancel
    node.isTaskComplete=lambda:node.complete;node.getResult=lambda:node.result
    manager=module.StationNavigation(node,None,ROOT)
    manager._pump_callbacks=lambda:None
    manager.grid=grid();manager._data=lambda:True
    manager._pose=lambda:pose[0];manager._stopped=lambda:stopped[0]
    goal=PoseStamped();goal.header.frame_id='map';goal.pose.position.x=1.;goal.pose.orientation.w=1.
    manager.start(goal,lambda:permission[0])
    return manager,node,clock,pose,stopped,permission


def tick(manager,clock):
    manager.isTaskComplete();clock[0]+=.6;return manager.isTaskComplete()


def finish_action(manager,node,clock,pose):
    pose[0]=node.sent[-1][0];node.complete=True
    tick(manager,clock)


def test_special_entry_is_one_continuous_final_goal_and_requires_real_stop(motion):
    manager,node,clock,pose,stopped,permission=motion
    manager.mode='station'
    tick(manager,clock)
    assert manager.phase=='staging' and len(node.sent)==1
    finish_action(manager,node,clock,pose)
    assert manager.phase=='entry' and 'station_x.xml' in node.sent[-1][1]
    assert node.sent[-1][0]==(1.,0.,0.) and len(node.sent)==2
    stopped[0]=False;node.complete=True;pose[0]=(1.,0.,0.)
    assert not tick(manager,clock) and manager.getResult()!=TaskResult.SUCCEEDED
    stopped[0]=True;tick(manager,clock)
    assert manager.getResult()==TaskResult.SUCCEEDED and manager.parked==(1.,0.,0.)


def test_permission_change_after_terminal_action_never_starts_successor(motion):
    manager,node,clock,pose,stopped,permission=motion
    tick(manager,clock);node.complete=True;node.result=TaskResult.CANCELED
    permission[0]=False
    assert manager.isTaskComplete()
    assert len(node.sent)==1 and node.cancels==1
    assert manager.getResult()==TaskResult.CANCELED


def test_cancel_must_drain_before_completion_and_never_starts_next_stage(motion):
    manager,node,clock,pose,stopped,permission=motion
    tick(manager,clock);manager.cancelTask()
    assert not manager.isTaskComplete() and len(node.sent)==1
    node.complete=True;node.result=TaskResult.CANCELED
    assert manager.isTaskComplete() and manager.getResult()==TaskResult.CANCELED
    assert len(node.sent)==1


def test_wrong_actual_pose_never_becomes_arrived_even_on_nav2_success(motion):
    manager,node,clock,pose,stopped,permission=motion
    tick(manager,clock);node.complete=True
    assert not manager.isTaskComplete()
    clock[0]+=7.
    assert manager.isTaskComplete() and manager.getResult()==TaskResult.FAILED


def test_missing_costmap_times_out_without_any_navigation_goal(motion):
    manager,node,clock,pose,stopped,permission=motion
    manager._data=lambda:False
    assert not manager.isTaskComplete()
    clock[0]+=7.
    assert not manager.isTaskComplete() and not manager.preflight_timeout
    clock[0]+=24.
    assert manager.isTaskComplete() and not node.sent
    assert manager.getResult()==TaskResult.FAILED
    assert manager.data_unavailable
    assert manager.preflight_timeout and manager.goals_submitted==0
    assert '尚未送出' in manager.error_details()[1]


def test_clear_departure_precedes_next_goal_and_uses_current_heading(motion):
    manager,node,clock,pose,stopped,permission=motion
    manager.parked=(0.,0.,0.)
    manager.mode='station';obstacle(manager.grid,0.,.25)
    tick(manager,clock)
    assert manager.phase=='departure' and node.sent[0][0][2]==0.
    assert math.hypot(*node.sent[0][0][:2])<=.50+1e-9
    finish_action(manager,node,clock,pose)
    # Departure already reached the next staging pose with the correct yaw;
    # do not submit a redundant zero-distance normal-navigation goal.
    assert manager.phase=='entry' and len(node.sent)==2


def test_heading_drift_during_translation_cancels_and_drains(motion):
    manager,node,clock,pose,stopped,permission=motion
    manager.mode='station';obstacle(manager.grid,0.,.25);tick(manager,clock)
    pose[0]=(0.,0.,.3)
    assert not manager.isTaskComplete() and node.cancels==1
    node.complete=True;node.result=TaskResult.CANCELED
    assert manager.isTaskComplete() and manager.getResult()==TaskResult.FAILED
    assert not manager.data_unavailable


def test_local_detour_cannot_leave_the_checked_translation_corridor(motion):
    manager,node,clock,pose,stopped,permission=motion
    manager.mode='station';obstacle(manager.grid,0.,.25);tick(manager,clock)
    pose[0]=(.10,.20,0.)
    assert not manager.isTaskComplete() and node.cancels==1
    node.complete=True;node.result=TaskResult.CANCELED
    assert manager.isTaskComplete() and manager.getResult()==TaskResult.FAILED
    assert '通道' in manager.error_details()[1]


def test_native_failure_details_are_not_reused_after_new_station_attempt(motion):
    manager,node,clock,pose,stopped,permission=motion
    tick(manager,clock)
    node.complete=True;node.result=TaskResult.FAILED
    node.result_future=NS(result=lambda:NS(result=NS(error_code=102,error_msg='TF error')))
    assert manager.isTaskComplete() and manager.error_details()==(102,'TF error')
    manager.start(manager.goal)
    assert manager.error_details()==(0,'')


def test_preflight_wait_can_be_cancelled_without_any_goal(motion):
    manager,node,clock,pose,stopped,permission=motion
    manager._data=lambda:False
    assert not manager.isTaskComplete()
    permission[0]=False
    assert manager.isTaskComplete() and manager.getResult()==TaskResult.CANCELED
    assert not node.sent and not manager.preflight_timeout


def test_safety_callbacks_are_pumped_before_deciding_that_data_is_stale(motion):
    manager,node,clock,pose,stopped,permission=motion
    fresh=[False]
    manager._data=lambda:fresh[0]
    manager._pump_callbacks=lambda:fresh.__setitem__(0,True)
    tick(manager,clock)
    assert manager.phase=='direct' and len(node.sent)==1


def test_stopped_confirmation_must_restart_after_a_data_gap(motion):
    manager,node,clock,pose,stopped,permission=motion
    assert not manager.isTaskComplete()
    clock[0]+=.3
    manager._data=lambda:False
    assert not manager.isTaskComplete() and manager.stop_since is None
    clock[0]+=.3
    manager._data=lambda:True
    assert not manager.isTaskComplete() and not node.sent
    clock[0]+=.6
    manager.isTaskComplete()
    assert len(node.sent)==1


def test_wait_diagnostics_explain_bad_grid_frame_and_are_rate_limited(motion):
    manager,node,clock,pose,stopped,permission=motion
    message=OccupancyGrid();message.header.frame_id='odom';message.header.stamp.sec=100
    manager._grid(message)
    manager._data=lambda:module.StationNavigation._data(manager)
    assert not manager.isTaskComplete()
    payload=node.messages[-1]
    assert payload['wait_blockers']==['costmap_missing']
    assert payload['inputs']['costmap']['error']=='frame_not_map'
    assert payload['inputs']['costmap']['source_age_sec']==0.
    assert payload['goals_submitted']==0 and not payload['preflight_timeout']
    count=len(node.messages)
    for _ in range(4):
        clock[0]+=.1;manager.isTaskComplete()
    assert len(node.messages)==count
    clock[0]+=1.;manager.isTaskComplete()
    assert len(node.messages)==count+1 and len(node.logs)==1
    clock[0]+=5.;manager.isTaskComplete()
    assert len(node.logs)==2
