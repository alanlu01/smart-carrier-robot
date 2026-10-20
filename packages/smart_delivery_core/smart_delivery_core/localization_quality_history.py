"""Independent covariance history for diagnosis, never permission to move."""

from collections import deque
import math

from smart_delivery_core.localization_health import pose_jump


class LocalizationQualityHistory:
    """Distinguish short critical bursts from persistent uncertainty cheaply."""

    def __init__(self):
        self.samples = deque(maxlen=32)
        self.last_stamp = None
        self.burst_started = None
        self.burst_samples = 0
        self.burst_has_gap = False
        self.last_burst = None
        self.last_jump = (0., 0.)
        self.rejected = 0

    def observe(self, stamp_ns, now, pose, quality, map_score=None):
        """Accept each valid source stamp once; do not count repeated callbacks."""
        if (stamp_ns <= 0 or (self.last_stamp is not None and stamp_ns <= self.last_stamp)
                or not all(math.isfinite(v) for v in (*pose, quality.xy_std, quality.yaw_std))):
            self.rejected += 1
            return False
        self.last_jump = pose_jump(self.samples[-1]['pose'], pose) if self.samples else (0., 0.)
        self.last_stamp = stamp_ns
        if self.burst_started is not None and self.samples and now-self.samples[-1]['at'] > 1.1:
            self.burst_has_gap = True
        if quality.critical:
            if self.burst_started is None:
                self.burst_started, self.burst_samples = now, 0
                self.burst_has_gap = False
            self.burst_samples += 1
        elif self.burst_started is not None:
            duration = max(0., now-self.burst_started)
            self.last_burst = {'duration_sec': round(duration, 3), 'samples': self.burst_samples,
                               'pattern': 'insufficient_temporal_coverage' if self.burst_has_gap else
                               'short_critical_burst' if duration <= 1.
                               else 'persistent_critical_burst', 'ended_at': now}
            self.burst_started, self.burst_samples = None, 0
        self.samples.append({'at': now, 'pose': pose, 'xy_std': quality.xy_std,
                             'yaw_std': quality.yaw_std, 'critical': quality.critical,
                             'healthy': quality.healthy, 'map_score': map_score})
        return True

    def snapshot(self, now):
        """Report patterns without claiming particles are outliers or pose is true."""
        recent = [s for s in self.samples if 0 <= now-s['at'] <= 3.]
        last = self.samples[-1] if self.samples else None
        age = None if last is None else now-last['at']
        pattern = ('stale' if age is None or age > 8. else
                   'awaiting_update' if age > 2. else 'critical' if last['critical'] else
                   'healthy' if last['healthy'] else 'degraded')
        burst = None if self.last_burst is None else {
            k: v for k, v in self.last_burst.items() if k != 'ended_at'}
        if burst is not None:
            burst['age_sec'] = round(max(0., now-self.last_burst['ended_at']), 3)
        return {'diagnostic_only': True, 'pattern': pattern,
                'independent_samples_3s': len(recent),
                'critical_samples_3s': sum(s['critical'] for s in recent),
                'healthy_samples_3s': sum(s['healthy'] for s in recent),
                'peak_xy_std_3s': round(max((s['xy_std'] for s in recent), default=0.), 4),
                'critical_elapsed_sec': None if self.burst_started is None else
                    round(max(0., now-self.burst_started), 3),
                'last_pose_step_m': round(self.last_jump[0], 4),
                'last_pose_step_rad': round(self.last_jump[1], 4),
                'last_map_score': None if last is None else last['map_score'],
                'last_source_stamp_ns': self.last_stamp,
                'last_sample_age_sec': None if last is None else round(max(0., now-last['at']), 3),
                'last_burst': burst, 'rejected_samples': self.rejected}
