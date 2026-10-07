"""A stalled recovery producer must stop, not coast on its last command."""

from hailo_vision.recovery_lease import RecoveryLease


def test_normal_navigation_unchanged_without_lease():
    assert not RecoveryLease().blocked(100)


def test_arm_and_stop_require_explicit_permission():
    lease = RecoveryLease()
    assert lease.receive({'token': 'a', 'active': True}, 1)
    assert lease.blocked(1)
    lease.receive({'token': 'a', 'active': True, 'permit': True}, 1.1)
    assert not lease.blocked(1.2)
    lease.receive({'token': 'a', 'active': True, 'permit': False}, 1.2)
    assert lease.blocked(1.2)


def test_producer_loss_latches_stop_until_matching_disarm():
    lease = RecoveryLease()
    lease.receive({'token': 'a', 'active': True, 'permit': True}, 1)
    assert lease.blocked(1.251)
    lease.receive({'token': 'a', 'active': True, 'permit': True}, 1.3)
    assert lease.blocked(1.3)  # a delayed producer must not resurrect movement
    assert not lease.receive({'token': 'other', 'active': False}, 2)
    assert not lease.receive({'token': 'other', 'active': True, 'permit': True}, 2)
    assert lease.blocked(2)
    assert lease.receive({'token': 'a', 'active': False}, 2.1)
    assert not lease.blocked(2.2)
