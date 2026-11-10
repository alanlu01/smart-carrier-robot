"""Unwrapped odometry budget for one bounded, resumable localization turn."""
import math


class RecoverySpin:
    def __init__(self, target, now, yaw, stamp_ns, active_limit=35.):
        self.target = abs(target)
        self.sign = 1. if target >= 0 else -1.
        self.started = self.last_tick = now
        self.last_yaw, self.last_stamp = yaw, stamp_ns
        self.travel = self.active_elapsed = self.wait_elapsed = 0.
        self.phase = 'running'
        self.reason = ''
        self.resumes = 0
        self.active_limit = active_limit
        self.cancel_at = None
        self.cancel_confirmed = False
        self.blocked_since = self.clear_since = None
        self.tail_rate = 0.
        self.finishing = False
        self.braking_reserve = math.radians(1.)

    @property
    def remaining(self):
        return max(0., self.target-self.travel)

    def observe(self, yaw, stamp_ns, rate):
        if stamp_ns <= self.last_stamp:
            return False
        dt = (stamp_ns-self.last_stamp)/1e9
        delta = math.atan2(math.sin(yaw-self.last_yaw), math.cos(yaw-self.last_yaw))
        if not all(math.isfinite(v) for v in (yaw, rate, delta)) or abs(delta) > max(.15, 3.*dt):
            self.reason = 'odom_jump'
            return False
        self.last_yaw, self.last_stamp = yaw, stamp_ns
        self.travel += self.sign*delta
        self.tail_rate = abs(rate)
        return True

    def tick(self, now, *, blocked, available, feedback_age_sec=0.):
        dt = max(0., now-self.last_tick)
        self.last_tick = now
        if self.phase == 'running':
            self.active_elapsed += dt
        elif self.phase in ('stopping', 'waiting'):
            self.wait_elapsed += dt
        if self.reason == 'odom_jump':
            return 'fail'
        if now-self.started >= 60. or self.wait_elapsed >= 15.:
            self.reason = 'bounded_wait_exhausted'
            return 'fail'
        if self.phase != 'running':
            return 'wait'
        if self.active_elapsed >= self.active_limit:
            self.reason = 'active_time_exhausted'
            return 'cancel_failed'
        if not available:
            self.reason = 'data_unavailable'
            return 'cancel_pause'
        self.blocked_since = (self.blocked_since if self.blocked_since is not None else now) if blocked else None
        if self.blocked_since is not None and now-self.blocked_since >= .3:
            self.reason = 'near_obstacle'
            return 'cancel_pause'
        # Account for held wheel velocity (currently about 2 Hz), the 5 Hz
        # supervisor/cancel path and the configured Spin deceleration. This is
        # an early-stop estimate, not a correction to measured odometry.
        age = float(feedback_age_sec)
        age = min(.65, max(0., age)) if math.isfinite(age) else .65
        rate = self.tail_rate
        reserve = rate*(.25+age) + rate*rate/(2.*.8)
        self.braking_reserve = max(math.radians(1.), min(math.radians(30.), reserve))
        if self.remaining <= self.braking_reserve:
            self.reason = 'endpoint_braking'
            return 'cancel_endpoint'
        return 'run'

    def stopped(self, now, safe):
        if self.travel-self.target > math.radians(3.):
            self.reason = 'endpoint_overshoot'
            return 'fail'  # no reverse correction, but never label overshoot precise
        if self.remaining <= math.radians(3.):
            self.phase = 'complete'
            return 'complete'
        self.phase = 'waiting'
        if self.reason == 'endpoint_braking':
            self.finishing = True
        self.clear_since = (self.clear_since if self.clear_since is not None else now) if safe else None
        if self.clear_since is not None and now-self.clear_since >= 1.:
            if self.resumes >= 2:
                self.reason = 'resume_limit'
                return 'fail'
            self.resumes += 1
            self.phase = 'running'
            self.cancel_confirmed = False
            self.cancel_at = self.blocked_since = self.clear_since = None
            return 'resume'
        return 'wait'

    def speed_limit(self):
        """Return a multiplier of the behavior server's configured 0.6 rad/s."""
        if self.phase != 'running':
            return 0.
        if self.finishing:
            return .25
        if self.remaining < math.radians(20.):
            return .35
        return .65 if self.remaining < math.radians(60.) else 1.

    def snapshot(self, now):
        return dict(phase=self.phase, reason=self.reason, travel_deg=math.degrees(self.travel),
                    remaining_deg=math.degrees(self.remaining), resumes=self.resumes,
                    active_sec=self.active_elapsed, wait_sec=self.wait_elapsed,
                    elapsed_sec=max(0., now-self.started), speed_limit=self.speed_limit(),
                    braking_reserve_deg=math.degrees(self.braking_reserve))
