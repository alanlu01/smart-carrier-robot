"""An auxiliary idle clock is not another navigation-arrival checker."""
import math


class StandbyIdleClock:
    """Count only verified, healthy physical parking; tolerate bounded AMCL jitter."""
    def __init__(self):
        self.point = None
        self.reset()

    def reset(self):
        self.elapsed = 0.0
        self.last_tick = None
        self.pause_since = None
        self.odom_anchor = None
        self.data_pause_since = None
        self.reason = 'not_verified_parked'

    def mark_parked(self, point):
        self.point = dict(point)
        self.reset()

    def update(self, now, *, parked, safe, stationary, pose, odom, transient_tf_gap=False):
        # Auxiliary time only: no motion can be authorized on missing TF.
        if (transient_tf_gap and parked and stationary and self.point is not None
                and odom is not None and all(math.isfinite(v) for v in odom)):
            moved = (self.odom_anchor is not None and (
                math.hypot(odom[0]-self.odom_anchor[0],odom[1]-self.odom_anchor[1])>.02
                or abs(math.atan2(math.sin(odom[2]-self.odom_anchor[2]),
                                  math.cos(odom[2]-self.odom_anchor[2])))>.035))
            self.data_pause_since = now if self.data_pause_since is None else self.data_pause_since
            if moved or now-self.data_pause_since>1.:
                self.reset()
                self.reason='physical_odom_displacement' if moved else 'tf_gap_expired'
            else:
                self.last_tick=None
                self.reason='short_tf_gap_paused'
            return False
        if self.data_pause_since is not None and now-self.data_pause_since>1.:
            self.reset()
        self.data_pause_since = None
        if not parked or not safe or not stationary or self.point is None or pose is None or odom is None:
            self.reset()
            self.reason = 'not_parked_or_unsafe_or_moving'
            return False
        if not all(math.isfinite(v) for v in (*odom, pose['x'], pose['y'], pose['yaw'])):
            self.reset()
            self.reason = 'nonfinite_pose'
            return False
        if self.odom_anchor is None:
            self.odom_anchor = odom
        distance = math.hypot(odom[0]-self.odom_anchor[0], odom[1]-self.odom_anchor[1])
        yaw = abs(math.atan2(math.sin(odom[2]-self.odom_anchor[2]),
                             math.cos(odom[2]-self.odom_anchor[2])))
        if distance > .02 or yaw > .035:
            self.reset()
            self.reason = 'physical_odom_displacement'
            return False
        xy = math.hypot(pose['x']-self.point['x'], pose['y']-self.point['y'])
        angle = abs(math.atan2(math.sin(pose['yaw']-self.point['yaw']),
                               math.cos(pose['yaw']-self.point['yaw'])))
        # Auxiliary hysteresis only; caller must already have verified arrival.
        if xy > .20 or angle > .20:
            self.pause_since = now if self.pause_since is None else self.pause_since
            self.last_tick = None
            self.reason = 'map_pose_outside_auxiliary_hysteresis'
            if now-self.pause_since >= 2.0:
                self.reset()
                self.reason = 'map_pose_deviation_persistent'
            return False
        self.pause_since = None
        if self.last_tick is not None:
            gap = now-self.last_tick
            if not 0 <= gap <= 1.0:
                self.reset()
            else:
                self.elapsed += gap
        self.last_tick = now
        self.reason = 'counting_verified_stationary_parking'
        return True
