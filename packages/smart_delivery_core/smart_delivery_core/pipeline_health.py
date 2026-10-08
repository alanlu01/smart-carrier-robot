"""Bounded receive statistics for distinguishing input and executor gaps."""

from collections import OrderedDict
from contextlib import contextmanager
import time


class ReceiptMetrics:
    """Track counters and peak timing without changing watchdog decisions."""

    def __init__(self):
        self.streams = {}

    def observe(self, name, now, source_age=None):
        """Record a receipt or timer edge in the local monotonic clock."""
        record = self.streams.setdefault(name, {
            'count': 0, 'last_at': None, 'max_gap': 0.0, 'peak_at': now,
            'source_age': None,
        })
        if record['last_at'] is not None:
            gap = max(0.0, now - record['last_at'])
            if gap > record['max_gap']:
                record['max_gap'], record['peak_at'] = gap, now
        record['count'] += 1
        record['last_at'], record['source_age'] = now, source_age

    def snapshot(self, now):
        """Include peak age so an old spike is not confused with a new fault."""
        return {
            name: {
                'count': record['count'],
                'receive_age_sec': round(max(0.0, now - record['last_at']), 3),
                'max_gap_sec': round(record['max_gap'], 3),
                'peak_age_sec': round(max(0.0, now - record['peak_at']), 3),
                'source_age_at_receive_sec': (
                    None if record['source_age'] is None
                    else round(record['source_age'], 3)
                ),
            }
            for name, record in self.streams.items()
        }


class DurationMetrics:
    """Constant-space wall-time measurements, not CPU time or DDS-only latency."""

    def __init__(self):
        self.operations = {}

    @contextmanager
    def measure(self, name):
        started = time.monotonic()
        try:
            yield
        finally:
            self.observe(name, max(0.0, time.monotonic() - started), time.monotonic())

    def observe(self, name, duration, now):
        record = self.operations.setdefault(name, {
            'count': 0, 'last_sec': 0.0, 'max_sec': 0.0, 'total_sec': 0.0,
            'over_50ms_count': 0, 'peak_at': now,
        })
        record['count'] += 1
        record['last_sec'] = duration
        record['total_sec'] += duration
        record['over_50ms_count'] += duration > .05
        if duration > record['max_sec']:
            record['max_sec'], record['peak_at'] = duration, now

    def snapshot(self, now):
        return {name: {
            'count': r['count'], 'last_sec': round(r['last_sec'], 6),
            'max_sec': round(r['max_sec'], 6),
            'mean_sec': round(r['total_sec'] / r['count'], 6),
            'over_50ms_count': r['over_50ms_count'],
            'peak_age_sec': round(max(0.0, now - r['peak_at']), 3),
        } for name, r in self.operations.items()}


class ScanPairMetrics:
    """Bounded same-stamp callback skew, NOT filter processing time in isolation."""

    def __init__(self, capacity=128):
        self.capacity = capacity
        self.pending = OrderedDict()
        self.count = self.evicted = 0
        self.last_sec = self.max_abs_sec = 0.0
        self.last_at = None

    def observe(self, name, stamp_ns, now):
        if stamp_ns <= 0:
            return
        pair = self.pending.setdefault(stamp_ns, {})
        pair.setdefault(name, now)
        if 'scan' in pair and 'scan_filtered' in pair:
            self.last_sec = pair['scan_filtered'] - pair['scan']
            self.max_abs_sec = max(self.max_abs_sec, abs(self.last_sec))
            self.count += 1
            self.last_at = now
            del self.pending[stamp_ns]
        while len(self.pending) > self.capacity:
            self.pending.popitem(last=False)
            self.evicted += 1

    def snapshot(self, now):
        return {'matched_count': self.count, 'last_filtered_minus_raw_callback_sec':
                None if self.last_at is None else round(self.last_sec, 6),
                'max_absolute_callback_skew_sec': round(self.max_abs_sec, 6),
                'last_pair_age_sec': None if self.last_at is None else round(now - self.last_at, 3),
                'pending_count': len(self.pending), 'evicted_unmatched_count': self.evicted}
