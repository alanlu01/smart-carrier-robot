"""Bounded duplicate delivery; source identity and evidence age never change."""


class WheelResend:
    def __init__(self):
        self.payload = None
        self.next_at = 0.
        self.remaining = 0
        self.groups = self.repeats = self.skipped_versions = 0
        self.version = 0

    def offer(self, version, payload, now):
        self.skipped_versions += max(0, version-self.version-1) if self.version else 0
        self.version = version
        self.payload = payload
        self.next_at, self.remaining = now+.10, 2
        self.groups += 1

    def due(self, now, ros_now_ns):
        if self.payload is None or not self.remaining or now < self.next_at:
            return None
        age = (ros_now_ns-self.payload['sample_ros_stamp_ns'])/1e9
        if not 0 <= age <= .65:
            self.remaining = 0
            return None
        self.remaining -= 1
        self.repeats += 1
        self.next_at = now+.10
        return self.payload

    def snapshot(self):
        payload = self.payload or {}
        return dict(source_sequence=payload.get('sequence'), session=payload.get('session'),
                    published_groups=self.groups, repeated_publications=self.repeats,
                    skipped_mailbox_versions=self.skipped_versions)
