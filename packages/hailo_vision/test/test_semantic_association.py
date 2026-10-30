"""Different same-class objects cannot share a single distance history."""
import math
from types import SimpleNamespace
from hailo_vision.semantic_association import TargetRangeHistory, range_association


def target(angle=0.,width=100.,cls=0):
    return dict(id=cls,angle=angle,bbox_width=width,score=.8)


def test_two_people_have_separate_near_history_and_immediate_approach_stop():
    h=TargetRangeHistory();h.begin_frame(100.)
    near,a=h.observe(target(-10.),.4,100.)
    far,b=h.observe(target(10.),2.,100.)
    assert a!=b and near==.4 and far==2.
    h.begin_frame(100.1)
    assert h.observe(target(10.5),.3,100.1)==(.3,b)
    assert h.observe(target(-10.5),2.,100.1)==(.4,a)


def test_expired_or_ambiguous_targets_never_inherit_old_near_distance():
    h=TargetRangeHistory();h.begin_frame(100.)
    h.observe(target(-2.),.4,100.);h.observe(target(2.),.4,100.)
    h.begin_frame(100.1)
    assert h.observe(target(),2.,100.1)[0]==2.
    h.begin_frame(101.)
    assert h.observe(target(),3.,101.)[0]==3.


def test_second_nearest_behavior_is_preserved_and_support_is_diagnostic():
    scan=SimpleNamespace(angle_min=0.,angle_increment=2*math.pi/1800,
        ranges=[3.]*1800,range_max=10.)
    scan.ranges[899]=.4;scan.ranges[900]=.42
    distance,detail=range_association(target(),scan)
    assert abs(distance-.42)<1e-6 and detail['near_cluster_support']==2
    assert detail['valid_beams']>2 and detail['reason']=='second_nearest'
