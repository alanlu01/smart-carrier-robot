"""Bounded corroboration of a stationary, confidently wrong AMCL solution."""
import math


class StationaryMapGuard:
    """Latch a suspicion until recovered; count only fresh, qualified evidence."""
    def __init__(self, duration=10., samples=5, maximum_gap=2.5):
        self.duration, self.samples, self.maximum_gap = duration, samples, maximum_gap
        self.reset()

    def reset(self):
        self.started = self.last_source = None
        self.count = 0
        self.elapsed = 0.
        self.confirmed = False
        self.pending = False

    @property
    def active(self):
        return self.pending

    def observe(self, now, source, qualified, *, recovered=None):
        if recovered is True:
            self.reset()
            return False
        if not qualified or not all(math.isfinite(v) for v in (now, source)):
            pending = self.pending and recovered is False
            self.reset()
            self.pending = pending
            return False
        if self.last_source is not None and (source < self.last_source
                or source-self.last_source > self.maximum_gap
                or now-self.last_source > self.maximum_gap):
            self.started = self.last_source = None
            self.count, self.elapsed, self.confirmed = 0, 0., False
        self.pending = True
        if self.last_source != source:
            if self.started is None:
                self.started = source
            self.last_source = source
            self.count += 1
            self.elapsed = max(0., source-self.started)
        self.confirmed = self.count >= self.samples and self.elapsed >= self.duration
        return self.confirmed


def dispatch_permitted(state):
    """State gate supplements, never overrides, the existing ready Bool gate."""
    return isinstance(state, dict) and state.get('dispatch_allowed', True) is True
