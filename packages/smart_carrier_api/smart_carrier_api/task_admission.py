"""Fail-closed claim permission for explicitly enabled automatic home return."""

import math


class TaskAdmissionGate:
    """A stopped home manager must never leave a permanent stale allow flag."""

    def __init__(self, enabled=False, timeout_sec=2.0):
        self.enabled = enabled
        self.timeout = timeout_sec
        self.receipt = None
        self.stamp = None
        self.accepting = False

    def observe(self, payload, now, ros_now):
        if not self.enabled:
            return
        try:
            stamp = float(payload['stamp_sec'])
            if (payload.get('source') != 'optional_idle' or not math.isfinite(stamp) or stamp <= 0
                    or not -0.1 <= ros_now - stamp <= self.timeout
                    or (self.stamp is not None and stamp <= self.stamp)
                    or not isinstance(payload.get('accepting'), bool)):
                self.accepting = False
                return
            self.receipt, self.stamp = now, stamp
            self.accepting = payload['accepting']
        except (KeyError, TypeError, ValueError):
            self.accepting = False

    def allows_claim(self, now):
        return (not self.enabled or (self.accepting and self.receipt is not None
                                    and 0 <= now - self.receipt <= self.timeout))
