"""Bounded corroboration of a stationary, confidently wrong AMCL solution."""
import math


class StationaryMapGuard:
    """Ignore brief occlusion, never ignore persistent critical mismatch forever."""
    def __init__(self, duration=10., samples=5, maximum_gap=2.5):
        self.duration, self.samples, self.maximum_gap = duration, samples, maximum_gap
        self.reset()

    def reset(self):
        self.started = self.last_source = None
        self.count = 0
        self.elapsed = 0.
        self.confirmed = False

    @property
    def active(self):
        return self.started is not None

    def observe(self, now, source, qualified):
        if not qualified or not all(math.isfinite(v) for v in (now, source)):
            self.reset()
            return False
        if self.last_source is not None and (source < self.last_source
                or source-self.last_source > self.maximum_gap
                or now-self.last_source > self.maximum_gap):
            self.reset()
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
