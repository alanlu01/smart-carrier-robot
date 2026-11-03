"""Bounded serial reconnection and fail-closed feedback evidence."""
import math
import os
import uuid
import serial
from smart_delivery_core.wheel_feedback import WheelSampleAssembler, WheelStopEvidence


def device_identity(path):
    value = os.stat(path)
    return value.st_dev, value.st_ino, value.st_rdev


class ReconnectingSerial:
    def __init__(self, path, baud, logger, *, factory=serial.Serial, identity=device_identity):
        self.path, self.baud, self.logger = path, baud, logger
        self.factory, self.identity = factory, identity
        self.port = None
        self.session = ''
        self.open_count = self.failures = 0
        self.next_retry = 0.
        self.backoff = .5
        self.last_log = -float('inf')
        self.error = ''

    def poll(self, now):
        if self.port is not None:
            try:
                if self.identity(self.path) != self.open_identity:
                    raise OSError('serial device was replaced')
                return True
            except (OSError, serial.SerialException) as exc:
                self.fail(exc, now)
        if now < self.next_retry:
            return False
        try:
            before = self.identity(self.path)
            port = self.factory(self.path, self.baud, timeout=0, write_timeout=.10)
            try:
                if self.identity(self.path) != before:
                    raise OSError('serial device changed during open')
                port.reset_input_buffer()
            except Exception:
                port.close()
                raise
            self.port, self.open_identity = port, before
            self.session = uuid.uuid4().hex
            self.open_count += 1
            self.backoff, self.error = .5, ''
            self.logger.info('🔌 STM32 串口已重開，等待新鮮回授')
            return True
        except (OSError, serial.SerialException) as exc:
            self.fail(exc, now)
            return False

    def fail(self, exc, now):
        was_open = self.port is not None
        self.close()
        self.error = str(exc)[:300]
        self.failures += 1
        self.next_retry = now + self.backoff
        self.backoff = min(5., self.backoff*2)
        if was_open or now-self.last_log >= 5.:
            self.logger.warning('⚠️ STM32 串口中斷，安全停車並等待重連：'+self.error)
            self.last_log = now

    def write(self, payload, now):
        if self.port is None:
            return False
        try:
            if self.port.write(payload) != len(payload):
                raise OSError('short serial write')
            return True
        except (OSError, serial.SerialException) as exc:
            self.fail(exc, now)
            return False

    def close(self):
        port, self.port = self.port, None
        if port is not None:
            try:
                port.close()
            except (OSError, serial.SerialException):
                pass

    def snapshot(self, now):
        return dict(connected=self.port is not None, session=self.session,
                    open_count=self.open_count, failures=self.failures, error=self.error,
                    retry_in_sec=max(0., self.next_retry-now) if self.port is None else 0.)


class FeedbackReadiness:
    def __init__(self, now):
        self.samples = WheelSampleAssembler(.25, now)
        self.stop = WheelStopEvidence(now)
        self.good_groups = 0

    def observe(self, line, now):
        self.stop.observe(line, now)
        if self.samples.is_stale(now, .65):
            self.good_groups = 0
        sample = self.samples.add_line(line, now)
        if sample is not None:
            self.good_groups = (self.good_groups+1 if all(math.isfinite(v) for v in sample.values()) else 0)
        elif '_RT:' in line and ('nan' in line.lower() or 'inf' in line.lower()):
            self.good_groups = 0

    def snapshot(self, now):
        at = self.samples.last_complete_at
        return dict(healthy=at is not None and self.good_groups >= 2 and not self.samples.is_stale(now, .65),
                    complete_sample_age_sec=None if at is None else max(0., now-at),
                    complete_sample_count=self.samples.complete_sample_count,
                    stop_proven=self.stop.ready(now))


def fresh_feedback(payload, ros_now_ns, received_age):
    try:
        stamp = int(payload['sample_ros_stamp_ns'])
        age = (ros_now_ns-stamp)/1e9
        complete = float(payload['complete_sample_age_sec'])
        complete_stamp = int(payload['complete_ros_stamp_ns'])
        return (payload['healthy'] is True and payload['connected'] is True and bool(payload['session'])
                and stamp > 0 and -.1 <= age <= .35 and 0 <= received_age <= .35
                and complete_stamp > 0 and -.1 <= (ros_now_ns-complete_stamp)/1e9 <= .65
                and math.isfinite(complete) and 0 <= complete+max(0.,age) <= .65)
    except (KeyError, ValueError, TypeError):
        return False
