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


def test_orphan_handoff_only_transfers_stop_and_cannot_revive_old_owner():
    lease = RecoveryLease()
    lease.receive({'token': 'a', 'owner': 'old', 'active': True, 'permit': True}, 1.)
    claim = {'token': 'a', 'owner': 'new', 'active': True, 'permit': False, 'handoff': True}
    assert not lease.receive(claim, 1.2)
    assert lease.receive(claim, 1.3)
    assert lease.owner == 'new' and lease.blocked(1.3)
    assert not lease.receive({'token': 'a', 'owner': 'old', 'active': True, 'permit': True}, 1.4)
    assert not lease.receive({'token': 'a', 'owner': 'old', 'active': False}, 1.4)
    assert lease.receive({'token': 'a', 'owner': 'new', 'active': True, 'permit': True}, 1.4)
    assert lease.blocked(1.4)  # even adopted owner cannot restore translation
    assert lease.receive({'token': 'a', 'owner': 'new', 'active': False}, 1.5)


def test_handoff_with_wrong_token_or_motion_permission_is_never_accepted():
    lease = RecoveryLease()
    lease.receive({'token': 'a', 'owner': 'old', 'active': True, 'permit': True}, 1.)
    assert not lease.receive({'token': 'wrong', 'owner': 'new', 'active': True,
                              'permit': False, 'handoff': True}, 2.)
    assert not lease.receive({'token': 'a', 'owner': 'new', 'active': True,
                              'permit': True, 'handoff': True}, 2.)


def test_delayed_closed_token_cannot_rearm_after_successful_disarm():
    lease = RecoveryLease()
    lease.receive({'token': 'a', 'owner': 'old', 'active': True, 'permit': False}, 1.)
    lease.receive({'token': 'a', 'owner': 'old', 'active': False}, 2.)
    assert not lease.receive({'token': 'a', 'owner': 'old', 'active': True, 'permit': True}, 2.1)
    assert not lease.enabled
    assert lease.receive({'token': 'b', 'owner': 'new', 'active': True, 'permit': False}, 2.2)
