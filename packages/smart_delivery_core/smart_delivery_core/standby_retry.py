"""Bound idle parking retries without blocking service orders."""

from dataclasses import dataclass


@dataclass
class StandbyRetryPolicy:
    """Bound per-goal attempts and episode-wide unexplained cancellations."""

    max_attempts: int = 3
    retry_delay: float = 10.0
    goal_key: tuple | None = None
    failures: int = 0
    unexpected_cancellations: int = 0
    retry_at: float = 0.0
    motion_blocked: bool = False
    recovery_rearms: int = 0
    recovery_token: object = None

    def __post_init__(self):
        if self.max_attempts < 1 or self.retry_delay <= 0:
            raise ValueError("待機重試次數與等待時間必須大於 0")

    def reset(self):
        """Rearm after a new service, successful parking, or manual restart."""
        self.goal_key = None
        self.failures = 0
        self.unexpected_cancellations = 0
        self.retry_at = 0.0
        self.motion_blocked = False
        self.recovery_rearms = 0
        self.recovery_token = None

    def rearm_after_recovery(self, token):
        """At most one extra parking round per episode, never after motor stall."""
        if token is None or token==self.recovery_token:
            return False
        self.recovery_token=token
        if self.motion_blocked or self.attempts<self.max_attempts or self.recovery_rearms>=1:
            return False
        self.recovery_rearms+=1
        self.failures=self.unexpected_cancellations=0
        self.retry_at=0.
        return True

    def select_goal(self, point):
        """Keep one retry budget for the same actual parking pose."""
        key = (point['name'], point['x'], point['y'], point['yaw'])
        if key != self.goal_key:
            self.failures = 0
            if not self.unexpected_cancellations:
                self.retry_at = 0.0
            self.goal_key = key

    @property
    def attempts(self):
        """Count unsuccessful attempts, including unexplained cancellation."""
        return self.failures + self.unexpected_cancellations

    def unexpected_cancel(self, now):
        """Bound unexplained cancels across parking targets in this episode."""
        self.unexpected_cancellations += 1
        self.retry_at = float(now) + self.retry_delay * 2 ** (self.attempts - 1)

    def failed(self, now):
        """Charge failed navigation only, never a localization/order cancel."""
        self.failures += 1
        self.retry_at = float(now) + self.retry_delay * 2 ** (self.attempts - 1)

    def waiting_data(self, now):
        """Back off before a goal exists without charging a navigation failure."""
        self.retry_at = max(self.retry_at,float(now)+self.retry_delay)

    def can_attempt(self, now):
        """Return permission without sleeping or claiming the robot is parked."""
        return not self.motion_blocked and self.attempts < self.max_attempts and now >= self.retry_at

    def block_motion(self):
        """No blind reattempt after a fresh-feedback rotation stalls."""
        self.motion_blocked = True

    def status(self, now):
        """Expose a small machine-readable reason while remaining interruptible."""
        if self.motion_blocked or self.attempts >= self.max_attempts:
            return 'blocked'
        return 'retry_wait' if now < self.retry_at else 'available'
