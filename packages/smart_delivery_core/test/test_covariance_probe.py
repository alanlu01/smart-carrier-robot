"""Covariance spikes retain goals only while stopped and evidence stays fresh."""
import pytest
from smart_delivery_core.covariance_probe import CovarianceProbe, StoppedConfidenceRecovery


def observe(p, now, stamp, *, healthy=False, critical=False, ok=True, x=0., ox=0.):
    p.observe(now,stamp,(x,0.,0.),(ox,0.,0.),healthy=healthy,critical=critical,evidence_ok=ok)


def test_transient_spike_requires_two_independent_healthy_samples():
    p=CovarianceProbe()
    observe(p,100.,1,healthy=True)
    observe(p,100.1,2,critical=True,x=.02,ox=.02)
    assert p.active and p.stopped
    observe(p,100.3,3,healthy=True,x=.03,ox=.03)
    assert p.active
    observe(p,100.5,4,healthy=True,x=.03,ox=.03)
    assert not p.stopped


@pytest.mark.parametrize('fault',['jump','data_gap','duplicate','timeout'])
def test_bad_evidence_escalates_without_reopening_the_grace_window(fault):
    p=CovarianceProbe();observe(p,100.,1,healthy=True);observe(p,100.1,2,critical=True)
    if fault=='jump':observe(p,100.2,3,healthy=True,x=.3)
    elif fault=='duplicate':observe(p,100.2,2,healthy=True)
    else:p.check_deadline(103.11 if fault=='timeout' else 100.2,fault!='data_gap')
    assert p.failed and p.stopped and not p.active
    observe(p,102.,4,healthy=True)
    assert p.failed  # Only a state-machine recovery resets this latch.


def test_no_prior_trusted_sample_cannot_open_a_probe():
    p=CovarianceProbe();observe(p,100.,1,critical=True)
    assert p.failed and not p.active


def test_recorded_repeated_outliers_get_two_fresh_healthy_samples_within_three_seconds():
    p=CovarianceProbe();observe(p,100.,1,healthy=True)
    observe(p,102.4,2,critical=True,x=.4,ox=.4)
    assert p.active
    observe(p,103.9,3,critical=True,x=.4,ox=.4)
    assert p.active
    observe(p,104.2,4,healthy=True,x=.4,ox=.4)
    observe(p,104.5,5,healthy=True,x=.4,ox=.4)
    assert not p.stopped
    assert p.MAX_WAIT==3.


def test_bad_map_opens_only_a_zero_motion_probe_and_cannot_verify_a_position():
    p=CovarianceProbe();observe(p,100.,1,healthy=True)
    p.observe(100.1,2,(0.,0.,0.),(0.,0.,0.),healthy=False,critical=True,evidence_ok=True,map_ok=False)
    assert p.active and p.stopped and p.reason=='waiting_current_map'
    for at,stamp in ((100.5,3),(101.,4)):
        p.observe(at,stamp,(0.,0.,0.),(0.,0.,0.),healthy=True,critical=False,evidence_ok=True,map_ok=False)
    assert p.stopped and p.healthy_samples==0
    p.check_deadline(103.2,True)
    assert p.failed and p.reason=='window_expired'


def test_stopped_confidence_recovery_needs_distinct_map_and_amcl_sources():
    h=StoppedConfidenceRecovery(((0.,0.,0.),(0.,0.,0.)),100.)
    args=dict(sources_ok=True,stopped=True,qualified=True)
    assert not h.observe(101.,1,1,(.02,0.,0.),(.02,0.,0.),**args)
    assert not h.observe(101.2,1,2,(.02,0.,0.),(.02,0.,0.),**args)
    assert not h.observe(101.3,2,1,(.02,0.,0.),(.02,0.,0.),**args)
    assert h.observe(102.,2,2,(.02,0.,0.),(.02,0.,0.),**args)


@pytest.mark.parametrize('fault',['source','pose_jump','timeout','moving','poor_map'])
def test_stopped_confidence_recovery_never_accepts_bad_or_moving_evidence(fault):
    h=StoppedConfidenceRecovery(((0.,0.,0.),(0.,0.,0.)),100.)
    for i in (1,2):
        assert not h.observe(110. if fault=='timeout' else 100.+i,i,i,
            (.3 if fault=='pose_jump' else 0.,0.,0.),(0.,0.,0.),
            sources_ok=fault!='source',stopped=fault!='moving',qualified=fault!='poor_map')
