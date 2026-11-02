"""Bounded stop-and-observe window for an isolated covariance spike.

This does not declare localization healthy. Motion stays zero until two fresh
healthy AMCL samples agree with odometry and the high-quality map observation.
"""
import math


def residual(anchor, odom_anchor, pose, odom):
    angle = anchor[2]-odom_anchor[2]
    dx, dy = odom[0]-odom_anchor[0], odom[1]-odom_anchor[1]
    predicted = (anchor[0]+math.cos(angle)*dx-math.sin(angle)*dy,
                 anchor[1]+math.sin(angle)*dx+math.cos(angle)*dy,
                 anchor[2]+odom[2]-odom_anchor[2])
    return (math.hypot(pose[0]-predicted[0], pose[1]-predicted[1]),
            abs(math.atan2(math.sin(pose[2]-predicted[2]), math.cos(pose[2]-predicted[2]))))


class CovarianceProbe:
    def __init__(self):
        self.active = self.failed = False
        self.anchor = None
        self.started = self.anchor_at = 0.
        self.last_stamp = 0
        self.healthy_samples = 0

    def observe(self, now, stamp, pose, odom, *, healthy, critical, evidence_ok):
        valid = (evidence_ok and stamp > self.last_stamp and odom is not None
                 and all(math.isfinite(v) for v in (*pose, *odom)))
        if self.failed:
            return
        if self.active:
            if (not valid or now-self.started >= 1.5
                    or not self._consistent(pose, odom)):
                self.failed, self.active = True, False
                return
            self.last_stamp = stamp
            self.healthy_samples = self.healthy_samples+1 if healthy else 0
            if self.healthy_samples >= 2 and now-self.started >= .3:
                self.active = False
                self.anchor, self.anchor_at = (pose, odom), now
            return
        if critical:
            if (valid and self.anchor is not None and 0 <= now-self.anchor_at <= 1.
                    and self._consistent(pose, odom)):
                self.active, self.started, self.healthy_samples = True, now, 0
            else:
                self.failed = True
        elif valid and healthy:
            self.anchor, self.anchor_at = (pose, odom), now
        if valid:
            self.last_stamp = stamp

    def _consistent(self, pose, odom):
        distance, angle = residual(*self.anchor, pose, odom)
        return distance <= .15 and angle <= .12

    def check_deadline(self, now, evidence_ok):
        if self.active and (not evidence_ok or not 0 <= now-self.started < 1.5):
            self.failed, self.active = True, False

    @property
    def stopped(self):
        return self.active or self.failed
