"""Pure state tracking for delivery-owned Nav2 goal heartbeats."""

from dataclasses import dataclass


@dataclass
class MissionLeaseMonitor:
    """Track whether a delivery-owned navigation heartbeat has expired."""

    timeout_sec: float
    armed: bool = False
    last_heartbeat_at: float | None = None

    def __post_init__(self):
        """Validate the lease timeout."""
        if self.timeout_sec <= 0.0:
            raise ValueError('timeout_sec must be greater than zero')

    def observe(self, active: bool, now: float):
        """Record a heartbeat or a clean navigation completion."""
        if active:
            self.armed = True
            self.last_heartbeat_at = float(now)
        else:
            self.disarm()

    def has_expired(self, now: float) -> bool:
        """Return true while an armed lease is older than its timeout."""
        return bool(
            self.armed
            and self.last_heartbeat_at is not None
            and float(now) - self.last_heartbeat_at >= self.timeout_sec
        )

    def disarm(self):
        """Clear the lease after completion or a successful cancellation."""
        self.armed = False
        self.last_heartbeat_at = None
