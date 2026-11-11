"""Measured sidestep evidence may fill transient holes, never persistent ones."""
import math
from types import SimpleNamespace as NS
from smart_delivery_core.recovery_scan_evidence import RecoveryScanEvidence


def scan(at,hole=None):
    ranges=[3.]*1800
    if hole:
        for i in range(*hole):ranges[i]=float('nan')
    return NS(header=NS(stamp=NS(sec=int(at),nanosec=round((at%1)*1e9))),
        angle_min=-math.pi,angle_increment=2*math.pi/1800,range_min=.05,range_max=12.,ranges=ranges)


def report(cache,direction=1,pose=(0.,0.,0.),at=100.):
    return cache.translation_report('scan',pose,at,at,direction,.30,.22,.16)


def add(cache,at,hole=None,yaw=0.):
    cache.observe('scan',scan(at,hole),at,(0.,0.,yaw),(0.,0.,0.),(0.,0.,yaw),None,.425)


def test_three_stopped_independent_frames_fill_alternating_six_degree_holes():
    cache=RecoveryScanEvidence()
    for at,begin in ((99.8,1080),(99.9,1160),(100.,1240)):add(cache,at,(begin,begin+30))
    result=report(cache)
    assert result['clear'] and result['valid_fraction']>=.95 and result['evidence_frames']==3
    assert not report(cache,pose=(.15,0.,0.))['clear']
    assert not report(cache,at=101.)['clear']
    assert not cache.translation_report('scan_filtered',(0.,0.,0.),100.,100.,1,.3,.22,.16)['clear']


def test_persistent_unknown_or_near_obstacle_never_becomes_free():
    cache=RecoveryScanEvidence()
    for at in (99.8,99.9,100.):add(cache,at,(1240,1300))
    assert not report(cache)['clear']
    cache=RecoveryScanEvidence()
    for at in (99.8,99.9,100.):add(cache,at)
    cache.frames['scan'][-1].ranges[1350]=.35
    assert report(cache)['reason']=='obstacle_or_occlusion'


def test_old_occlusion_and_missing_raw_stream_remain_blocked():
    cache=RecoveryScanEvidence()
    for at in (99.8,99.9,100.):add(cache,at)
    cache.frames['scan'][0].ranges[1350]=.35
    assert report(cache)['reason']=='recent_obstacle_or_occlusion'
    cache=RecoveryScanEvidence();add(cache,100.)
    assert not report(cache)['clear']


def test_backwards_lidar_right_sweep_and_changed_mount_are_checked():
    cache=RecoveryScanEvidence()
    for at in (99.8,99.9,100.):add(cache,at,yaw=math.pi)
    assert report(cache,-1)['clear']
    cache.frames['scan'][-1].sensor_pose=(.02,0.,math.pi)
    assert not report(cache,-1)['clear']


def test_authorized_translation_can_keep_measured_history_but_not_large_jump():
    cache=RecoveryScanEvidence()
    for at,y in ((99.8,-.04),(99.9,-.02),(100.,0.)):
        cache.observe('scan',scan(at),at,(0.,y,0.),(0.,y,0.),(0.,0.,0.),None,.425)
    assert report(cache)['clear']
    assert not report(cache,pose=(0.,.20,0.))['clear']
