"""Short-lived, token-scoped permission for automatic recovery translation."""

import math
from collections import deque


class LocalizationSpeedLimit:
    """No producer is compatible with teleop; an expired known producer stops."""

    def __init__(self, timeout=1.0):
        self.timeout, self.received_at, self.value = timeout, None, 1.0

    def observe(self, value, now):
        self.received_at = now
        self.value = float(value) if math.isfinite(value) and 0 <= value <= 1 else 0.0

    def limit(self, now):
        return self.value if self.received_at is None or now-self.received_at <= self.timeout else 0.0


class RecoveryLease:
    """A lost producer latches STOP until the same token is explicitly disarmed."""

    def __init__(self, timeout=0.25):
        self.timeout = timeout
        self.token = None
        self.received_at = None
        self.enabled = False
        self.permit = False
        self.faulted = False
        self.owner = None
        self.closed_tokens = deque(maxlen=64)

    def receive(self, payload, now):
        # Evaluate expiry BEFORE accepting a delayed heartbeat. Expired permission
        # cannot resurrect motion merely because the producer resumes running.
        self.blocked(now)
        token = str(payload.get('token', ''))
        if not token:
            return False
        if payload.get('active') is True and token in self.closed_tokens:
            return False
        owner = payload.get('owner')
        if (payload.get('handoff') is True and payload.get('active') is True
                and payload.get('permit') is False and token == self.token
                and self.faulted and self.received_at is not None
                and now - self.received_at > self.timeout
                and isinstance(owner, str) and owner):
            # Adoption transfers STOP ownership only. Expiry stays latched.
            self.owner, self.received_at, self.permit = owner, now, False
            return True
        if self.enabled and self.owner is not None and owner != self.owner:
            return False
        if payload.get('active') is True:
            if self.token is not None and token != self.token:
                return False
            self.token, self.received_at, self.enabled = token, now, True
            self.owner = owner
            self.permit = payload.get('permit') is True
            return True
        if token == self.token and payload.get('active') is False:
            self.closed_tokens.append(token)
            self.token, self.received_at, self.enabled = None, None, False
            self.permit = False
            self.faulted = False
            self.owner = None
            return True
        return False

    def blocked(self, now):
        if self.enabled and (
            self.received_at is None or now - self.received_at > self.timeout
        ):
            self.faulted = True
        return self.enabled and (self.faulted or not self.permit)
