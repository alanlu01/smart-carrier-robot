"""Covariance spikes retain goals only while stopped and evidence stays fresh."""
import pytest
from smart_delivery_core.covariance_probe import CovarianceProbe


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
    else:p.check_deadline(101.61 if fault=='timeout' else 100.2,fault!='data_gap')
    assert p.failed and p.stopped and not p.active
    observe(p,102.,4,healthy=True)
    assert p.failed  # Only a state-machine recovery resets this latch.


def test_no_prior_trusted_sample_cannot_open_a_probe():
    p=CovarianceProbe();observe(p,100.,1,critical=True)
    assert p.failed and not p.active
