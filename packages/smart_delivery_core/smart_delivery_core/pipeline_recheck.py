"""Bounded stopped verification after a data outage, separate from pose search."""
import math


class PipelineRecheck:
    def __init__(self, started, source_ns, reference, required=6,
                 timeout=30., verification_timeout=8., distance=.20, angle=.174533):
        self.started, self.source_ns, self.reference = started, source_ns, reference
        self.required, self.timeout, self.verification_timeout = required, timeout, verification_timeout
        self.distance, self.angle = distance, angle
        self.verifying_since = None
        self.verification_elapsed = 0.
        self.last_available_at = None
        self.samples = 0
        self.last_map_at = started
        self.last_amcl_stamp_ns = source_ns
        self.reason = 'waiting_data'
        self.position_delta = self.angle_delta = None

    def observe(self, now, *, available, stopped, healthy, new_amcl, map_at, transform,
                amcl_stamp_ns=None):
        stamp = round(now*1e9) if amcl_stamp_ns is None else amcl_stamp_ns
        if now-self.started >= self.timeout:
            self.reason = 'pipeline_timeout' if not available else 'verification_timeout'
            return 'manual_pipeline' if not available else 'pose_recovery'
        if not available or not stopped:
            self.last_available_at = None
            self.samples = 0
            self.last_map_at = max(self.last_map_at, map_at)
            self.reason = 'waiting_data' if not available else 'waiting_stop'
            return 'wait'
        if self.verifying_since is None:
            self.verifying_since = now
        if self.last_available_at is not None:
            self.verification_elapsed += max(0., now-self.last_available_at)
        self.last_available_at = now
        consistent = False
        if self.reference is not None and transform is not None:
            self.position_delta = math.hypot(transform[0]-self.reference[0], transform[1]-self.reference[1])
            delta = transform[2]-self.reference[2]
            self.angle_delta = abs(math.atan2(math.sin(delta), math.cos(delta)))
            consistent = self.position_delta <= self.distance and self.angle_delta <= self.angle
        qualified = healthy and new_amcl and consistent
        if not qualified:
            self.samples = 0
            self.last_map_at = max(self.last_map_at, map_at)
            self.reason = 'position_changed' if not consistent else 'waiting_quality'
        elif map_at > self.last_map_at and stamp > self.last_amcl_stamp_ns:
            self.samples += 1
            self.last_map_at, self.last_amcl_stamp_ns = map_at, stamp
            self.reason = 'verifying'
        if qualified and self.samples >= self.required:
            self.reason = 'verified'
            return 'resume'
        if self.verification_elapsed >= self.verification_timeout:
            return 'pose_recovery'
        return 'wait'

    def snapshot(self, now):
        return dict(elapsed_sec=round(max(0., now-self.started), 3), samples=self.samples,
                    verification_elapsed_sec=round(self.verification_elapsed, 3),
                    required_samples=self.required, reason=self.reason,
                    position_delta_m=self.position_delta, angle_delta_rad=self.angle_delta)
