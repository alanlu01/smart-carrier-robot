"""Tests for delivery-owned navigation heartbeat expiry."""

from smart_delivery_core.mission_lease import MissionLeaseMonitor


def test_inactive_monitor_never_expires():
    """An unarmed lease must never expire."""
    monitor = MissionLeaseMonitor(1.0)

    assert not monitor.has_expired(100.0)


def test_active_heartbeat_expires_after_timeout():
    """An active lease expires at its configured deadline."""
    monitor = MissionLeaseMonitor(1.0)
    monitor.observe(True, 10.0)

    assert not monitor.has_expired(10.999)
    assert monitor.has_expired(11.0)


def test_new_heartbeat_extends_lease():
    """A fresh heartbeat moves the expiry deadline forward."""
    monitor = MissionLeaseMonitor(1.0)
    monitor.observe(True, 10.0)
    monitor.observe(True, 10.8)

    assert not monitor.has_expired(11.1)
    assert monitor.has_expired(11.8)


def test_clean_completion_disarms_lease():
    """A false heartbeat records normal action completion."""
    monitor = MissionLeaseMonitor(1.0)
    monitor.observe(True, 10.0)
    monitor.observe(False, 10.2)

    assert not monitor.has_expired(20.0)
