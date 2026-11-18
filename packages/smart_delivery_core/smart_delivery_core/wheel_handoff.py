"""Retry only the newest wheel group without blocking the serial owner."""


class WheelHandoff:
    """Bound contention by time and attempts; never refresh source evidence."""

    def __init__(self, mailbox):
        self.mailbox = mailbox
        self.pending = None
        self.pending_at = 0.
        self.attempts = 0
        self.received = self.delivered = self.write_failures = self.retries = 0
        self.superseded = self.expired = self.invalidated = 0
        self.last_delivered = None

    def offer(self, payload, now):
        if self.pending is not None:
            self.superseded += 1
        self.pending, self.pending_at, self.attempts = payload, now, 0
        self.received += 1

    def flush(self, now, ros_now_ns, *, session, valid):
        payload = self.pending
        if payload is None:
            return
        if not valid or payload['session'] != session:
            self.pending = None
            self.invalidated += 1
            return
        age = (ros_now_ns-payload['sample_ros_stamp_ns'])/1e9
        wait = now-self.pending_at
        if not 0 <= age <= .65 or not 0 <= wait <= .20 or self.attempts >= 5:
            self.pending = None
            self.expired += 1
            return
        self.retries += int(self.attempts > 0)
        self.attempts += 1
        if self.mailbox.put(payload):
            self.delivered += 1
            self.last_delivered = payload['sequence']
            self.pending = None
        else:
            self.write_failures += 1

    def snapshot(self):
        return dict(received_groups=self.received, delivered_groups=self.delivered,
                    write_failures=self.write_failures, retries=self.retries,
                    superseded_groups=self.superseded, expired_groups=self.expired,
                    invalidated_groups=self.invalidated,
                    pending_sequence=None if self.pending is None else self.pending['sequence'],
                    last_delivered_sequence=self.last_delivered)
