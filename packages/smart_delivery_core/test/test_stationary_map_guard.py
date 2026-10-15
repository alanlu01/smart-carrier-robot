"""Persistent mismatch must be observed while still, before dispatching a goal."""
import pytest
from smart_delivery_core.stationary_map_guard import StationaryMapGuard, dispatch_permitted


def test_critical_low_score_needs_ten_seconds_and_independent_observations():
    guard=StationaryMapGuard()
    for t in range(100,110):
        assert not guard.observe(t,t,True)
    assert guard.active and guard.observe(110,110,True)


def test_same_stamp_gaps_and_occlusion_never_fake_persistent_evidence():
    guard=StationaryMapGuard()
    for t in (100.,100.2,100.5):
        assert not guard.observe(t,100.,True)
    assert guard.count==1
    assert not guard.observe(110.,110.,True) and guard.count==1
    assert not guard.observe(111.,111.,False) and not guard.active


@pytest.mark.parametrize('state,allowed', [({},True),({'dispatch_allowed':False},False),
    ({'dispatch_allowed':True},True),({'dispatch_allowed':'true'},False),(None,False)])
def test_dispatch_supplements_ready_not_string_coercion(state,allowed):
    assert dispatch_permitted(state)==allowed
