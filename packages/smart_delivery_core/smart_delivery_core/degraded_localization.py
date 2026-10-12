"""Bounded tolerance for mild static-map occlusion, never uncertain AMCL."""

import math


class MildMapGuard:
    """Permit only a short, distance-bounded half-speed observation window."""

    def __init__(self, seconds=3.0, distance=.40, samples=3):
        self.seconds, self.distance, self.samples = seconds, distance, samples
        self.reset()

    def reset(self):
        self.started = None
        self.anchor = None
        self.last_map_at = None
        self.count = 0
        self.exhausted = False

    def allows(self, now, map_at, pose):
        if self.exhausted or pose is None or not all(math.isfinite(v) for v in pose):
            return False
        if self.started is None:
            self.started, self.anchor = now, pose
        if map_at != self.last_map_at:
            self.count += 1
            self.last_map_at = map_at
        moved = math.hypot(pose[0]-self.anchor[0], pose[1]-self.anchor[1])
        self.exhausted = (now-self.started >= self.seconds or moved >= self.distance
                          or self.count >= self.samples)
        return not self.exhausted


def convergence_is_stalled(samples, elapsed, window=8.0, minimum_samples=6):
    """Use only independent fresh observations; callers gate critical covariance."""
    recent = [(at, value) for at, value in samples
              if elapsed-window-1.5 <= at <= elapsed
              and math.isfinite(value) and value > 0]
    if (len(recent) < minimum_samples or elapsed < window
            or elapsed-recent[-1][0] > 2.0
            or recent[-1][0]-recent[0][0] < window-1.5):
        return False
    old = sum(value for _, value in recent[:3])/3
    new = sum(value for _, value in recent[-3:])/3
    return new >= old*.90
