"""Bounded soft-observation and stall detection, without ROS/hardware."""
import pytest
from smart_delivery_core.degraded_localization import MildMapGuard, convergence_is_stalled


def test_mild_guard_counts_independent_maps_not_timer_calls():
    guard = MildMapGuard()
    for moment in (100.,100.2,100.4):
        assert guard.allows(moment,100.,(0.,0.,0.))
    assert guard.count == 1
    assert guard.allows(101.,101.,(.1,0.,0.))
    assert not guard.allows(102.,102.,(.2,0.,0.))
    assert not guard.allows(102.1,102.,(.2,0.,0.))
    guard.reset()
    assert guard.allows(103.,103.,(.2,0.,0.))


@pytest.mark.parametrize('moment,pose', [(103.,(0,0,0)),(101.,(.41,0,0)),(101.,(float('nan'),0,0))])
def test_mild_observation_cannot_continue_beyond_time_distance_or_valid_pose(moment,pose):
    guard=MildMapGuard()
    guard.allows(100.,100.,(0,0,0))
    assert not guard.allows(moment,100.,pose)


def test_stall_requires_fresh_independent_span_and_does_not_interrupt_improvement():
    assert convergence_is_stalled([(float(i),2.) for i in range(10)],9.)
    assert not convergence_is_stalled([(float(i),2.-i*.1) for i in range(10)],9.)
    assert not convergence_is_stalled([(0.,2.),(8.,2.)],8.)
    assert not convergence_is_stalled([(float(i),2.) for i in range(10)],15.)
    assert not convergence_is_stalled([(float(i),2.) for i in range(5)],4.)
