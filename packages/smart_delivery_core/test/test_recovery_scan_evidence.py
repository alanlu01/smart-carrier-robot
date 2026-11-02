"""Fallback fills transient holes with measured rays; obstacles remain dominant."""
import math
from types import SimpleNamespace
from smart_delivery_core.recovery_scan_evidence import RecoveryScanEvidence


def scan(at,start,end):
    values=[3.]*1800
    values[start:end]=[float('inf')]*(end-start)
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=int(at),nanosec=round((at-int(at))*1e9))),
        ranges=values,angle_min=-math.pi,angle_increment=2*math.pi/1800,range_min=.05,range_max=10.)


def test_independent_transient_holes_are_filled_without_crossing_streams():
    cache=RecoveryScanEvidence()
    for at,begin in ((99.8,100),(99.9,300),(100.,500)):
        cache.observe('scan',scan(at,begin,begin+120),at,(0.,0.,math.pi),(0.,0.,0.),(0.,0.,math.pi),None,.4)
    assert cache.report('scan',(0.,0.,0.),100.,100.)['clear']
    assert not cache.report('scan_filtered',(0.,0.,0.),100.,100.)['clear']
    assert not cache.report('scan',(0.,0.,0.),101.,101.)['clear']


def test_persistent_holes_near_obstacles_and_pose_jump_do_not_grant_permission():
    cache=RecoveryScanEvidence()
    for at in (99.8,99.9,100.):
        cache.observe('scan',scan(at,100,240),at,(0.,0.,math.pi),(0.,0.,0.),(0.,0.,math.pi),None,.4)
    assert not cache.report('scan',(0.,0.,0.),100.,100.)['clear']
    assert not cache.report('scan',(.1,0.,0.),100.,100.)['clear']
    cache.frames['scan'][-1].ranges[0]=.2
    assert cache.report('scan',(0.,0.,0.),100.,100.)['reason']=='near_obstacle'


def sparse(at, phase):
    data=scan(at,100,190)  # Persistent 18-degree hole, with remaining holes interleaved.
    for i in list(range(200+phase,1800,4))[:390]:data.ranges[i]=float('inf')
    return data


def test_73_percent_requires_three_independent_frames_and_80_percent_union():
    cache=RecoveryScanEvidence()
    for at,phase in ((99.8,0),(99.9,1),(100.,2)):
        cache.observe('scan',sparse(at,phase),at,(0.,0.,0.),(0.,0.,0.),(0.,0.,0.),None,.4)
    result=cache.report('scan',(0.,0.,0.),100.,100.)
    assert result['clear'] and .73 <= result['current_valid_fraction'] < .75
    assert result['valid_fraction'] >= .80 and result['evidence_frames']==3
    assert result['unknown_arc_deg'] <= 20.
    cache.frames['scan'].popleft()
    assert not cache.report('scan',(0.,0.,0.),100.,100.)['clear']
    cache=RecoveryScanEvidence()
    for at in (99.8,99.9,100.):
        data=scan(at,0,0)
        for begin in (0,250,500,750,1000,1250):
            data.ranges[begin:begin+80]=[float('inf')]*80
        cache.observe('scan',data,at,(0.,0.,0.),(0.,0.,0.),(0.,0.,0.),None,.4)
    assert not cache.report('scan',(0.,0.,0.),100.,100.)['clear']


def test_lower_than_73_percent_or_a_near_return_cannot_use_union_permission():
    cache=RecoveryScanEvidence()
    for at,phase in ((99.8,0),(99.9,1),(100.,2)):
        cache.observe('scan',sparse(at,phase),at,(0.,0.,0.),(0.,0.,0.),(0.,0.,0.),None,.4)
    latest=cache.frames['scan'][-1]
    latest.ranges[500:530]=float('inf')
    assert not cache.report('scan',(0.,0.,0.),100.,100.)['clear']
    latest.ranges[0]=.2
    assert cache.report('scan',(0.,0.,0.),100.,100.)['reason']=='near_obstacle'
