import math
import re


WHEEL_LETTERS = ("A", "B", "C", "D")
_WHEEL_REPORT = re.compile(
    r"\b([ABCD])_RT:\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)"
)


def parse_wheel_report(raw_line):
    """Return a wheel letter and raw speed from one STM32 telemetry line."""
    match = _WHEEL_REPORT.search(raw_line)
    if match is None:
        return None
    return match.group(1), float(match.group(2))


class WheelSampleAssembler:
    """Commit wheel speeds only after one fresh A/B/C/D telemetry group."""

    def __init__(self, batch_window_sec, started_at):
        if batch_window_sec <= 0:
            raise ValueError("batch_window_sec must be greater than zero")
        self.batch_window_sec = float(batch_window_sec)
        self.started_at = float(started_at)
        self.last_complete_at = None
        self.complete_sample_count = 0
        self._batch_started_at = None
        self._pending = {}

    def add_line(self, raw_line, received_at):
        report = parse_wheel_report(raw_line)
        if report is None:
            return None

        received_at = float(received_at)
        letter, raw_speed = report
        batch_expired = (
            self._batch_started_at is not None
            and received_at - self._batch_started_at > self.batch_window_sec
        )
        duplicate_wheel = letter in self._pending
        if batch_expired or duplicate_wheel:
            self._pending.clear()
            self._batch_started_at = None

        if not self._pending:
            self._batch_started_at = received_at
        self._pending[letter] = raw_speed

        if any(wheel not in self._pending for wheel in WHEEL_LETTERS):
            return None

        snapshot = {wheel: self._pending[wheel] for wheel in WHEEL_LETTERS}
        self._pending.clear()
        self._batch_started_at = None
        self.last_complete_at = received_at
        self.complete_sample_count += 1
        return snapshot

    def sample_age(self, now):
        reference = self.last_complete_at
        if reference is None:
            reference = self.started_at
        return max(0.0, float(now) - reference)

    def is_stale(self, now, timeout_sec):
        if timeout_sec <= 0:
            raise ValueError("timeout_sec must be greater than zero")
        return self.sample_age(now) > float(timeout_sec)


class WheelStopEvidence:
    """Require three independent zero groups spanning one second, not held values.

    The 0.65 s evidence limit remains below the chassis' 0.80 s watchdog.
    A repeated timer tick can never create another wheel sample. Any individual
    moving wheel, expired group, or interruption invalidates the whole proof.
    """

    MAX_AGE = 0.65

    def __init__(self, started_at):
        self.assembler = WheelSampleAssembler(0.25, started_at)
        self.reset()

    def reset(self):
        self.first_zero_at = None
        self.last_zero_at = None
        self.zero_groups = 0

    def observe(self, line, now):
        report = parse_wheel_report(line)
        if report is not None:
            if not math.isfinite(report[1]) or abs(report[1]) > 1.0:
                self.reset()
        elif '_RT:' in line:
            self.reset()
        sample = self.assembler.add_line(line, now)
        if sample is None:
            return
        if not all(math.isfinite(v) and abs(v) <= 1.0 for v in sample.values()):
            self.reset()
            return
        if self.last_zero_at is None or now - self.last_zero_at > self.MAX_AGE:
            self.reset()
        if self.first_zero_at is None:
            self.first_zero_at = now
        self.last_zero_at = now
        self.zero_groups += 1

    def ready(self, now):
        if self.last_zero_at is None or not 0 <= now - self.last_zero_at <= self.MAX_AGE:
            self.reset()
            return False
        return (self.zero_groups >= 3
                and self.last_zero_at - self.first_zero_at >= 1.0 - 1e-9)
