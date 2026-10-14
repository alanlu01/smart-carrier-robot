"""Measured finite rays, ego-motion alignment and conservative evidence expiry."""
from types import SimpleNamespace
import math
import numpy as np
import pytest
from smart_delivery_core.spin_evidence import ScanEvidence, aligned_spin_report


def frame(at,hole=None,range_value=3.,base_yaw=0.,sensor_offset=(0.,0.)):
    ranges=np.full(1800,range_value)
    if hole is not None:ranges[hole[0]:hole[1]]=float('inf')
    sensor=(math.cos(base_yaw)*sensor_offset[0]-math.sin(base_yaw)*sensor_offset[1],
            math.sin(base_yaw)*sensor_offset[0]+math.cos(base_yaw)*sensor_offset[1],base_yaw+math.pi)
    scan=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=int(at),nanosec=round((at-int(at))*1e9))),
        angle_min=-math.pi,angle_increment=2*math.pi/1800,range_min=.05,range_max=10.,ranges=ranges)
    return ScanEvidence.from_scan(scan,at,sensor,(0.,0.,base_yaw))


def report(frames,at=100.,pose=(0.,0.,0.)):
    return aligned_spin_report(frames,pose,at,at,.372)


def test_intermittent_unknown_holes_resolve_only_with_independent_finite_rays():
    frames=[frame(99.8,(100,220)),frame(99.9,(300,420)),frame(100.,(500,620))]
    result=report(frames)
    assert result['clear'] and result['evidence_frames']==3
    assert result['unknown_arc_deg']<=15


def test_permanent_hole_or_current_all_inf_never_becomes_free():
    assert not report([frame(t,(100,240)) for t in (99.8,99.9,100.)])['clear']
    assert not report([frame(99.8),frame(99.9),frame(100.,range_value=float('inf'))])['clear']


def test_current_obstacle_overrides_clear_history_and_recent_occupied_evidence_persists():
    frames=[frame(99.8),frame(99.9),frame(100.)]
    frames[-1].ranges[100]=.2
    assert report(frames)['reason']=='near_obstacle'
    frames[-1].ranges[100]=3.
    frames[0].ranges[100]=.2
    assert report(frames)['reason']=='recent_near_obstacle'


def test_expiry_duplicate_frames_and_translation_jump_do_not_authorize_spin():
    assert not report([frame(99.),frame(99.1),frame(100.)])['clear']
    same=frame(100.)
    assert not report([same,same,same])['clear']
    assert not report([frame(t) for t in (99.8,99.9,100.)],pose=(.10,0.,0.))['clear']


@pytest.mark.parametrize('yaw',[0.,.1,.25])
def test_rotation_and_inverted_offset_laser_are_aligned_in_odom(yaw):
    frames=[frame(t,base_yaw=yaw-d,sensor_offset=(.15,-.025))
            for t,d in ((99.8,.1),(99.9,.05),(100.,0.))]
    assert report(frames,pose=(0.,0.,yaw))['clear']


def test_old_short_range_is_not_free_after_dynamic_obstacle_time_budget():
    frames=[frame(99.65,range_value=.6),frame(99.7,range_value=.6),frame(100.,(100,220),range_value=3.)]
    assert not report(frames)['clear']


def test_changed_mount_calibration_cannot_supply_historical_free_space():
    frames=[frame(99.8,sensor_offset=(.1,0.)),frame(99.9,sensor_offset=(.1,0.)),frame(100.)]
    assert not report(frames)['clear']


def test_latest_pose_jump_is_rejected_even_with_other_clear_frames():
    frames=[frame(99.8),frame(99.9),frame(100.)]
    frames[-1].base_pose=(.1,0.,0.)
    assert report(frames)['reason']=='laser_pose_jump'
