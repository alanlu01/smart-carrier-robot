"""Replay the observed 24.9-second clearance wait without any ROS/motion."""
import pytest
from smart_delivery_core.station_timing import TranslationBudget
from smart_delivery_core.navigation_retry import navigation_failure_is_transient, transient_navigation_log


def test_recorded_wait_does_not_consume_the_remaining_motion_budget():
    b=TranslationBudget(100.,340.)
    b.transition(100.28,'waiting')
    assert b.deadline(125.)==pytest.approx(130.28)
    b.transition(125.,'moving')
    assert b.deadline(125.)==pytest.approx(149.72)
    assert b.deadline(126.)>149.


def test_repeated_pauses_cannot_replenish_motion_or_waiting_or_absolute_limits():
    b=TranslationBudget(100.,150.)
    b.transition(110.,'waiting');b.transition(130.,'moving')
    assert b.deadline(130.)==145.
    b.transition(135.,'waiting')
    assert b.deadline(135.)==145.
    b.transition(146.,'moving')
    assert b.waiting==0. and b.deadline(146.)<=150.
    b.transition(90.,'waiting')
    assert b.moving==b.waiting==0.


def test_own_cancel_halt_is_not_transport_fault_but_real_ack_timeout_is():
    assert not transient_navigation_log('Failed to get result for follow_path in node halt!')
    assert transient_navigation_log('Timed out while waiting for action server to acknowledge request')
    assert not navigation_failure_is_transient(0,'平移通道持續受阻',
        failure_kind='translation_blocked',recent_tf=True,recent_transport=True)
    assert navigation_failure_is_transient(0,'等待資料',data_unavailable=True)
