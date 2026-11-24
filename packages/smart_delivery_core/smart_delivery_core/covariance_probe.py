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
    MAX_WAIT = 3.
    MAX_ANCHOR_AGE = 3.
    def __init__(self):
        self.active = self.failed = False
        self.anchor = None
        self.started = self.anchor_at = 0.
        self.last_stamp = 0
        self.healthy_samples = 0
        self.reason = ''

    def observe(self, now, stamp, pose, odom, *, healthy, critical, evidence_ok, map_ok=True):
        valid = (evidence_ok and stamp > self.last_stamp and odom is not None
                 and all(math.isfinite(v) for v in (*pose, *odom)))
        if self.failed:
            return
        if self.active:
            if (not valid or not 0 <= now-self.started < self.MAX_WAIT
                    or not self._consistent(pose, odom)):
                self.failed, self.active = True, False
                self.reason=('source_invalid' if not valid else 'window_expired'
                             if not 0<=now-self.started<self.MAX_WAIT else 'pose_odom_disagreement')
                return
            self.last_stamp = stamp
            self.healthy_samples = self.healthy_samples+1 if healthy and map_ok else 0
            self.reason='waiting_healthy_samples' if map_ok else 'waiting_current_map'
            if self.healthy_samples >= 2 and now-self.started >= .3:
                self.active = False
                self.anchor, self.anchor_at = (pose, odom), now
                self.reason='verified'
            return
        if critical:
            if (valid and self.anchor is not None and 0 <= now-self.anchor_at <= self.MAX_ANCHOR_AGE
                    and self._consistent(pose, odom)):
                self.active, self.started, self.healthy_samples = True, now, 0
                self.reason='waiting_healthy_samples' if map_ok else 'waiting_current_map'
            else:
                self.failed = True
                self.reason=('source_invalid' if not valid else 'anchor_unavailable'
                    if self.anchor is None or not 0<=now-self.anchor_at<=self.MAX_ANCHOR_AGE
                    else 'pose_odom_disagreement')
        elif valid and healthy and map_ok:
            self.anchor, self.anchor_at = (pose, odom), now
        if valid:
            self.last_stamp = stamp

    def _consistent(self, pose, odom):
        distance, angle = residual(*self.anchor, pose, odom)
        return distance <= .15 and angle <= .12

    def check_deadline(self, now, evidence_ok):
        if self.active and (not evidence_ok or not 0 <= now-self.started < self.MAX_WAIT):
            self.failed, self.active = True, False
            self.reason='source_invalid' if not evidence_ok else 'window_expired'

    @property
    def stopped(self):
        return self.active or self.failed


class StoppedConfidenceRecovery:
    """Two independent strong observations after a bounded probe expires.

    This verifier has no motion permission. It retains the pre-spike odometric
    reference, expires after 8 s and is never used for jumps or source loss.
    """
    def __init__(self, anchor, now):
        self.anchor=anchor
        self.started=now
        self.failed=False
        self.samples=0
        self.last_amcl=self.last_map=0
        self.reason='waiting_fresh_evidence'

    def observe(self, now, amcl_stamp, map_stamp, pose, odom, *, sources_ok, stopped, qualified):
        if self.failed:return False
        if (not sources_ok or self.anchor is None or not 0<=now-self.started<=8.
                or pose is None or odom is None
                or not all(math.isfinite(v) for v in (*pose,*odom))):
            self.failed=True;self.reason='source_or_deadline';return False
        distance,angle=residual(*self.anchor,pose,odom)
        if distance>.15 or angle>.12:
            self.failed=True;self.reason='pose_odom_disagreement';return False
        if not stopped or not qualified:
            self.samples=0;self.reason='waiting_stopped_quality';return False
        if amcl_stamp<=self.last_amcl or map_stamp<=self.last_map:return False
        self.last_amcl,self.last_map=amcl_stamp,map_stamp
        self.samples+=1
        self.reason='verified' if self.samples>=2 else 'waiting_independent_sample'
        return self.samples>=2
