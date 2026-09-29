"""Bounded receive statistics for distinguishing input and executor gaps."""


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
