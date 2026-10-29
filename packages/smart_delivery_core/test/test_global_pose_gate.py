"""Independent static-wall contradictions, frame invariance and bounded geometry."""
import math
from types import SimpleNamespace as S
import pytest
from smart_delivery_core.global_pose_gate import ray_consistency
from test_localization_recovery import manager,amcl,fresh


def scene():
    size=200
    data=[0]*(size*size)
    for i in range(size):
        data[i]=data[(size-1)*size+i]=100
        data[i*size]=data[i*size+size-1]=100
    grid=S(info=S(width=size,height=size,resolution=.05,
        origin=S(position=S(x=0.,y=0.),orientation=S(x=0.,y=0.,z=0.,w=1.))),data=data)
    ranges=[]
    for i in range(720):
        a=-math.pi+i*math.pi/360
        co,si=math.cos(a),math.sin(a)
        ranges.append(min(4.95/abs(co) if abs(co)>1e-6 else 100.,
                          4.95/abs(si) if abs(si)>1e-6 else 100.))
    return grid,S(ranges=ranges,angle_min=-math.pi,angle_increment=math.pi/360,range_min=.05,range_max=12.)


def test_consistent_room_and_rotated_map_frame_pass():
    grid,scan=scene()
    good=ray_consistency(grid,scan,(5.,5.,0.))
    assert good['verified'] and good['rays']<=60
    grid.info.origin.position.x=100.
    grid.info.origin.position.y=-4.
    grid.info.origin.orientation.z=grid.info.origin.orientation.w=math.sqrt(.5)
    rotated=ray_consistency(grid,scan,(95.,1.,math.pi/2))
    assert rotated['verified'] and rotated['conflict_fraction']==good['conflict_fraction']


def test_endpoint_matches_do_not_hide_rays_passing_through_static_wall():
    grid,scan=scene()
    for y in range(200):grid.data[y*200+140]=100
    report=ray_consistency(grid,scan,(5.,5.,0.))
    assert report['endpoint_match']>.8 and report['conflict_fraction']>.3
    assert not report['verified']


@pytest.mark.parametrize('kind',['unknown','no_returns','wrong_pose'])
def test_unknown_or_unmatched_geometry_cannot_verify(kind):
    grid,scan=scene();pose=(5.,5.,0.)
    if kind=='unknown':grid.data=[-1]*40000
    elif kind=='no_returns':scan.ranges=[float('inf')]*720
    else:pose=(6.5,5.,0.)
    assert not ray_consistency(grid,scan,pose)['verified']


@pytest.mark.parametrize('field',['angle','origin'])
def test_nonfinite_geometry_fails_closed(field):
    grid,scan=scene()
    if field=='angle':scan.angle_increment=float('nan')
    else:grid.info.origin.position.x=float('nan')
    assert not ray_consistency(grid,scan,(5.,5.,0.))['verified']


def test_six_good_covariance_samples_cannot_bypass_failed_global_geometry(manager):
    node,clock=manager
    node.global_verification_required=True
    node.global_geometry_report={'verified':False,'conflict_fraction':.4}
    for at in range(101,108):
        fresh(node,clock,float(at));node.global_geometry_at=clock.now
        amcl(node)
    assert node.stable_samples==0 and not node.ready


def test_geometry_must_be_fresh_and_normal_navigation_is_unaffected(manager):
    node,clock=manager
    node.global_verification_required=True
    node.global_geometry_report={'verified':True}
    node.global_geometry_at=90.
    assert not node._global_pose_is_verified(clock.now)
    node.global_geometry_at=100.
    assert node._global_pose_is_verified(clock.now)
    node.global_verification_required=False
    node.global_geometry_report={'verified':False}
    assert node._global_pose_is_verified(clock.now)


def test_geometry_and_original_six_samples_are_both_required(manager):
    node,clock=manager
    node.global_verification_required=True
    node.global_geometry_report={'verified':True}
    for at in range(101,107):
        fresh(node,clock,float(at));node.global_geometry_at=clock.now
        amcl(node)
        assert node.ready is (at==106)


def test_post_lateral_wait_does_not_reset_particles_before_safe_rotation(manager):
    node,clock=manager
    node.post_lateral_observation=True
    node.recovery_step='second_guard_wait'
    node.second_guard_started_at=100.
    node.second_guard_reason='post-lateral test'
    node._try_lateral_recovery=lambda _:pytest.fail('second sidestep attempted')
    node._lateral_sensors_ready=lambda _:True
    for at in (101.,103.,106.,107.):
        fresh(node,clock,at);node._tick_state_machine()
        assert not node.started_spins and node.state!='MANUAL_REQUIRED'
    fresh(node,clock,108.1);node._tick_state_machine()
    assert node.state=='MANUAL_REQUIRED' and not node.started_spins
