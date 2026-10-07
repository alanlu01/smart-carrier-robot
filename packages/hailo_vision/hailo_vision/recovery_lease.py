"""Short-lived, token-scoped permission for automatic recovery translation."""


class RecoveryLease:
    """A lost producer latches STOP until the same token is explicitly disarmed."""

    def __init__(self, timeout=0.25):
        self.timeout = timeout
        self.token = None
        self.received_at = None
        self.enabled = False
        self.permit = False
        self.faulted = False

    def receive(self, payload, now):
        # Evaluate expiry BEFORE accepting a delayed heartbeat. Expired permission
        # cannot resurrect motion merely because the producer resumes running.
        self.blocked(now)
        token = str(payload.get('token', ''))
        if not token:
            return False
        if payload.get('active') is True:
            if self.token is not None and token != self.token:
                return False
            self.token, self.received_at, self.enabled = token, now, True
            self.permit = payload.get('permit') is True
            return True
        if token == self.token and payload.get('active') is False:
            self.token, self.received_at, self.enabled = None, None, False
            self.permit = False
            self.faulted = False
            return True
        return False

    def blocked(self, now):
        if self.enabled and (
            self.received_at is None or now - self.received_at > self.timeout
        ):
            self.faulted = True
        return self.enabled and (self.faulted or not self.permit)
