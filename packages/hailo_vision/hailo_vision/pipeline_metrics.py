"""Small interval counters for diagnosing latency without retaining images."""


class PipelineMetrics:
    """Separate input delivery gaps, source age and processing stage cost."""

    def __init__(self, started_at):
        self.window_started_at = float(started_at)
        self.last_received = {}
        self.counts = {}
        self.maximums = {}

    def observe(self, stream, now):
        """Count callback receipts independently of processing completions."""
        previous = self.last_received.get(stream)
        if previous is not None:
            self.measure(stream + '_gap_sec', max(0.0, now - previous))
        self.last_received[stream] = float(now)
        self.counts[stream] = self.counts.get(stream, 0) + 1

    def measure(self, stage, duration):
        """Retain a maximum only; never store frames or per-frame log lines."""
        self.maximums[stage] = max(self.maximums.get(stage, 0.0), float(duration))

    def snapshot(self, now):
        """Reset interval peaks while preserving receive edges across windows."""
        result = {
            'window_sec': round(max(0.0, now - self.window_started_at), 3),
            'counts': dict(self.counts),
            'max_sec': {key: round(value, 4) for key, value in self.maximums.items()},
            'receive_age_sec': {
                key: round(max(0.0, now - at), 3)
                for key, at in self.last_received.items()
            },
        }
        self.window_started_at = float(now)
        self.counts.clear()
        self.maximums.clear()
        return result
