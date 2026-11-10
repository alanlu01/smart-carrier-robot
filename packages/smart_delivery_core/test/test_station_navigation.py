"""Station-only routing, escape geometry, safety gates and action ownership."""
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
from xml.etree import ElementTree

import pytest
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import TaskResult

from smart_delivery_core.station_geometry import StationGrid
from smart_delivery_core.station_nav_profile import station_parameters
from smart_delivery_core import station_navigation as module

ROOT = Path(__file__).parents[1]


def grid():
    info = NS(resolution=.025,width=200,height=200,
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


def parameters():
    return yaml.safe_load((ROOT/'config/my_nav2_params.yaml').read_text(encoding='utf-8'))


def test_station_overlay_leaves_normal_controller_smoother_and_footprint_exactly_unchanged():
    original=parameters();updated=station_parameters(original)
    a,b=original['controller_server']['ros__parameters'],updated['controller_server']['ros__parameters']
    assert a['FollowPath']==b['FollowPath'] and a['FollowPath']['vy_std']==.05
    for key in ('velocity_smoother','local_costmap','global_costmap','localization_manager'):
        assert original[key]==updated[key]
    for name in ('StationX','StationY','StationXSlow','StationYSlow'):
        profile=b[name];cap=.06 if name.endswith('Slow') else .15
        assert math.hypot(profile['vx_max'],profile['vy_max'])<=cap+1e-9
        assert profile['wz_max']==.15
        assert profile['CostCritic']['consider_footprint']
        assert not profile['PreferForwardCritic']['enabled']
        assert not profile['PathAngleCritic']['enabled']
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
    for name in ('x','y','x_slow','y_slow'):
        root=ElementTree.parse(ROOT/f'behavior_trees/station_{name}.xml')
        assert not any(root.findall('.//'+tag) for tag in ('Spin','BackUp','ControllerSelector','RecoveryNode'))
        follow=root.find('.//FollowPath')
        assert follow.get('goal_checker_id')=='general_goal_checker'
        assert follow.get('controller_id')=='Station'+name[0].upper()+('Slow' if 'slow' in name else '')


@pytest.fixture
def motion(monkeypatch):
    clock=[100.];pose=[(0.,0.,0.)];stopped=[True];permission=[True]
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(module.rclpy,'spin_once',lambda *_a,**_k:None)
    node=NS(sent=[],complete=False,result=TaskResult.SUCCEEDED,cancels=0)
    node.create_publisher=lambda *_a,**_k:NS(publish=lambda _:None)
    node.create_subscription=lambda *_a,**_k:None
    node.get_logger=lambda:NS(info=lambda _:None)
    node.get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:PoseStamped().header.stamp,nanoseconds=int(clock[0]*1e9)))
    def send(goal,behavior_tree=''):
        node.sent.append((module.coordinates(goal),behavior_tree));node.complete=False;return True
    def cancel():node.cancels+=1
    node.goToPose=send;node.cancelTask=cancel
    node.isTaskComplete=lambda:node.complete;node.getResult=lambda:node.result
    manager=module.StationNavigation(node,None,ROOT)
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


def test_staging_entry_endpoint_and_real_stop_are_required_before_arrived(motion):
    manager,node,clock,pose,stopped,permission=motion
    tick(manager,clock)
    assert manager.phase=='staging' and len(node.sent)==1
    finish_action(manager,node,clock,pose)
    assert manager.phase=='entry' and 'station_x.xml' in node.sent[-1][1]
    finish_action(manager,node,clock,pose)
    assert manager.phase=='endpoint' and 'station_x_slow.xml' in node.sent[-1][1]
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
    assert manager.isTaskComplete() and not node.sent
    assert manager.getResult()==TaskResult.FAILED
    assert manager.data_unavailable


def test_clear_departure_precedes_next_goal_and_uses_current_heading(motion):
    manager,node,clock,pose,stopped,permission=motion
    manager.parked=(0.,0.,0.)
    tick(manager,clock)
    assert manager.phase=='departure' and node.sent[0][0][2]==0.
    assert math.hypot(*node.sent[0][0][:2])<=.50+1e-9
    finish_action(manager,node,clock,pose)
    # Departure already reached the next staging pose with the correct yaw;
    # do not submit a redundant zero-distance normal-navigation goal.
    assert manager.phase=='entry' and len(node.sent)==2


def test_heading_drift_during_translation_cancels_and_drains(motion):
    manager,node,clock,pose,stopped,permission=motion
    manager.parked=(0.,0.,0.);tick(manager,clock)
    pose[0]=(0.,0.,.3)
    assert not manager.isTaskComplete() and node.cancels==1
    node.complete=True;node.result=TaskResult.CANCELED
    assert manager.isTaskComplete() and manager.getResult()==TaskResult.FAILED
    assert not manager.data_unavailable


def test_local_detour_cannot_leave_the_checked_translation_corridor(motion):
    manager,node,clock,pose,stopped,permission=motion
    manager.parked=(0.,0.,0.);tick(manager,clock)
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
