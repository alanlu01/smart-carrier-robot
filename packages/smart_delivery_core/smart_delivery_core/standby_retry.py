"""Bound idle parking retries without blocking service orders."""

from dataclasses import dataclass


@dataclass
class StandbyRetryPolicy:
    """Allow three attempts per goal, with non-blocking exponential backoff."""

    max_attempts: int = 3
    retry_delay: float = 10.0
    goal_key: tuple | None = None
    failures: int = 0
    retry_at: float = 0.0

    def __post_init__(self):
        if self.max_attempts < 1 or self.retry_delay <= 0:
            raise ValueError("待機重試次數與等待時間必須大於 0")

    def reset(self):
        """Rearm after a new service, successful parking, or manual restart."""
        self.goal_key = None
        self.failures = 0
        self.retry_at = 0.0

    def select_goal(self, point):
        """Keep one retry budget for the same actual parking pose."""
        key = (point['name'], point['x'], point['y'], point['yaw'])
        if key != self.goal_key:
            self.reset()
            self.goal_key = key

    def failed(self, now):
        """Charge failed navigation only, never a localization/order cancel."""
        self.failures += 1
        self.retry_at = float(now) + self.retry_delay * 2 ** (self.failures - 1)

    def can_attempt(self, now):
        """Return permission without sleeping or claiming the robot is parked."""
        return self.failures < self.max_attempts and now >= self.retry_at

    def status(self, now):
        """Expose a small machine-readable reason while remaining interruptible."""
        if self.failures >= self.max_attempts:
            return 'blocked'
        return 'retry_wait' if now < self.retry_at else 'available'
