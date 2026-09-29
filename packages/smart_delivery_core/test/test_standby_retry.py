"""Regression coverage for bounded, interruptible idle parking retries."""

from smart_delivery_core.standby_retry import StandbyRetryPolicy


POINT = {'name': 'seat', 'x': 1.2, 'y': -5.0, 'yaw': 0.0}


def test_three_attempts_have_ten_and_twenty_second_backoff():
    policy = StandbyRetryPolicy()
    policy.select_goal(POINT)
    assert policy.can_attempt(100.0)
    policy.failed(100.0)
    assert not policy.can_attempt(109.9)
    assert policy.can_attempt(110.0)
    policy.failed(110.0)
    assert not policy.can_attempt(129.9)
    assert policy.can_attempt(130.0)
    policy.failed(130.0)
    assert not policy.can_attempt(10000.0)
    assert policy.status(10000.0) == 'blocked'


def test_same_goal_selection_preserves_retry_budget():
    policy = StandbyRetryPolicy()
    policy.select_goal(POINT)
    policy.failed(100.0)
    policy.select_goal(dict(POINT))
    assert policy.failures == 1
    assert policy.status(105.0) == 'retry_wait'


def test_service_or_new_goal_rearms_an_exhausted_episode():
    policy = StandbyRetryPolicy()
    policy.select_goal(POINT)
    for _ in range(3):
        policy.failed(100.0)
    policy.reset()
    assert policy.can_attempt(101.0)
    policy.select_goal(POINT)
    policy.failed(102.0)
    policy.select_goal({**POINT, 'x': 2.4})
    assert policy.can_attempt(102.0)
