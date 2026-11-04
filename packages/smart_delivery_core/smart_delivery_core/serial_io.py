"""Serial owners have no ROS publishers, network calls, or unbounded queues."""
from collections import deque
import math
import time

from smart_delivery_core.pipeline_health import DurationMetrics
from smart_delivery_core.serial_recovery import (
    FeedbackReadiness, ReconnectingSerial, fresh_feedback, feedback_diagnostics,
)

ZERO = b'$CMD,0.00,0.00,0.00#\n'


class BufferedSerialLog:
    def __init__(self):
        self.sequence = 0
        self.records = deque(maxlen=16)

    def _add(self, level, message):
        self.sequence += 1
        self.records.append([self.sequence, level, str(message)[:400]])

    def info(self, message):
        self._add('info', message)

    def warning(self, message):
        self._add('warning', message)


class SerialReceiver:
    def __init__(self, path, baud, *, link=None):
        self.logger = BufferedSerialLog()
        self.link = link or ReconnectingSerial(path, baud, self.logger)
        self.session = ''
        self.pending = bytearray()
        self.feedback = FeedbackReadiness(time.monotonic())
        self.complete_ros_ns = 0
        self.sequence = self.frames = self.backlog_resets = 0
        self.last_tick = None
        self.max_tick_gap = 0.
        self.raw = deque(maxlen=64)
        self.metrics = DurationMetrics()

    def step(self, now):
        gap = 0. if self.last_tick is None else max(0., now-self.last_tick)
        self.last_tick = now
        self.max_tick_gap = max(self.max_tick_gap, gap)
        with self.metrics.measure('serial_poll'):
            connected = self.link.poll(now)
        if not connected:
            self.pending.clear()
            self.complete_ros_ns = 0
            return None
        if self.session != self.link.session:
            self.session = self.link.session
            self.pending.clear()
            self.feedback = FeedbackReadiness(now)
            self.complete_ros_ns = 0
            self.sequence = 0
        # There is no MCU acquisition stamp. After an unobserved interval, never
        # assign today's time to bytes accumulated in the OS/USB buffer.
        if gap > .65 or self.link.port.in_waiting > 4096:
            self.link.port.reset_input_buffer()
            self.pending.clear()
            self.feedback = FeedbackReadiness(now)
            self.complete_ros_ns = 0
            self.backlog_resets += 1
            self.logger.warning('📡 回授讀取曾中斷，丟棄積壓資料並重新確認停妥')
            return None
        with self.metrics.measure('serial_read'):
            self.pending.extend(self.link.port.read(min(4096, max(1, self.link.port.in_waiting))))
        if len(self.pending) > 8192:
            raise OSError('serial feedback frame buffer overflow')
        latest = None
        with self.metrics.measure('parse_groups'):
            for _ in range(32):
                if b'\n' not in self.pending:
                    break
                line, _, rest = self.pending.partition(b'\n')
                self.pending = bytearray(rest)
                text = line.decode(errors='ignore').strip()[:512]
                if not text:
                    continue
                received = time.monotonic()
                stamp = time.time_ns()
                self.frames += 1
                self.raw.append(dict(sequence=self.frames, stamp_ns=stamp, text=text))
                before = self.feedback.samples.complete_sample_count
                wheels = self.feedback.observe(text, received)
                if self.feedback.samples.complete_sample_count > before:
                    if not all(math.isfinite(v) for v in wheels.values()):
                        self.complete_ros_ns = 0
                        latest = None
                        continue
                    self.sequence += 1
                    self.complete_ros_ns = stamp
                    latest = dict(schema_version=1, source='serial_rx', session=self.session,
                                  sequence=self.sequence, sample_ros_stamp_ns=stamp, wheels=wheels)
        return latest

    def health(self, now):
        data = self.link.snapshot(now)
        data.update(self.feedback.snapshot(now))
        data.update(sample_ros_stamp_ns=time.time_ns(), complete_ros_stamp_ns=self.complete_ros_ns,
                    frames=self.frames, backlog_resets=self.backlog_resets,
                    io_tick_max_gap_sec=self.max_tick_gap,
                    wall_durations=self.metrics.snapshot(now), logs=list(self.logger.records))
        data['healthy'] = bool(data['connected'] and data['healthy'] and self.complete_ros_ns)
        return data


class SerialTransmitter:
    def __init__(self, path, baud, *, link=None, clock_ns=time.time_ns):
        self.logger = BufferedSerialLog()
        self.link = link or ReconnectingSerial(path, baud, self.logger)
        self.clock_ns = clock_ns
        self.session = self.rx_session = ''
        self.feedback, self.feedback_at = {}, 0.
        self.armed = False
        self.last_command = self.last_zero = -float('inf')
        self.opened_ros_ns = 0
        self.rejected_commands = self.accepted_commands = self.arming_count = 0
        self.last_tick = None
        self.max_tick_gap = 0.
        self.metrics = DurationMetrics()
        self.disarm_counts, self.reject_counts = {}, {}
        self.last_disarm = self.last_rejection = None

    def disarm(self, reason, now):
        if self.armed:
            self.disarm_counts[reason] = self.disarm_counts.get(reason, 0)+1
            self.last_disarm = dict(reason=reason, at=now,
                feedback=feedback_diagnostics(self.feedback,self.clock_ns(),now-self.feedback_at))
        self.armed = False

    def feedback_ready(self, now):
        return fresh_feedback(self.feedback, self.clock_ns(), now-self.feedback_at)

    def zero(self, now):
        if self.link.port is not None and now-self.last_zero >= .15:
            with self.metrics.measure('zero_write'):
                self.link.write(ZERO, now)
            self.last_zero = now

    def poll(self, now=None):
        now = time.monotonic() if now is None else now
        gap = 0. if self.last_tick is None else max(0., now-self.last_tick)
        self.max_tick_gap = max(self.max_tick_gap, gap)
        self.last_tick = now
        with self.metrics.measure('serial_poll'):
            connected = self.link.poll(now)
        if not connected:
            self.disarm('tx_disconnected',now)
            return
        if self.session != self.link.session:
            self.session = self.link.session
            self.disarm('tx_session_changed',now)
            self.opened_ros_ns = self.clock_ns()
            self.last_zero = -float('inf')
            self.zero(now)
            if self.link.port is None:
                return
        ready = self.feedback_ready(now)
        if not ready or self.rx_session != self.feedback.get('session') or gap > .25:
            if self.armed:
                self.last_zero = -float('inf')
            reason = ('io_loop_gap' if gap > .25 else 'rx_session_changed'
                      if self.rx_session != self.feedback.get('session') else
                      feedback_diagnostics(self.feedback,self.clock_ns(),now-self.feedback_at)['reason'])
            self.disarm(reason,now)
            self.rx_session = self.feedback.get('session', '')
        if (not self.armed and ready and self.feedback.get('stop_proven') is True
                and self.feedback['sample_ros_stamp_ns'] > self.opened_ros_ns
                and self.feedback['complete_ros_stamp_ns'] > self.opened_ros_ns):
            self.armed = True
            self.arming_count += 1
            self.logger.info('✅ STM32 新鮮回授及停車確認完成')
        if not self.armed or now-self.last_command > .25:
            self.zero(now)

    def command(self, payload, now=None):
        now = time.monotonic() if now is None else now
        values = payload.get('values', [])
        source = payload.get('source_stamp_ns', 0)
        received = payload.get('received_at', -float('inf'))
        if (not self.armed or self.rx_session != self.feedback.get('session')
                or not self.feedback_ready(now) or len(values) != 3
                or not all(math.isfinite(v) for v in values)
                or not 0 <= now-received <= .25
                or not source > 0 or not -.1 <= (self.clock_ns()-source)/1e9 <= .25):
            self.rejected_commands += 1
            reason = ('not_armed' if not self.armed else 'rx_session_changed'
                      if self.rx_session != self.feedback.get('session') else 'feedback_not_ready'
                      if not self.feedback_ready(now) else 'invalid_values'
                      if len(values) != 3 or not all(math.isfinite(v) for v in values)
                      else 'command_receipt_age' if not 0 <= now-received <= .25 else 'command_source_age')
            self.reject_counts[reason] = self.reject_counts.get(reason,0)+1
            self.last_rejection = dict(reason=reason,at=now)
            self.zero(now)
            return
        encoded = f'$CMD,{values[0]:.2f},{values[1]:.2f},{values[2]:.2f}#\n'.encode()
        with self.metrics.measure('command_write'):
            sent = self.link.write(encoded, now)
        if sent:
            self.last_command = now
            self.accepted_commands += 1
        else:
            self.disarm('command_write_failed',now)

    def health(self, now):
        data = self.link.snapshot(now)
        ready = self.armed and self.feedback_ready(now)
        data.update(sample_ros_stamp_ns=self.clock_ns(), armed=ready, rx_session=self.rx_session,
                    rejected_commands=self.rejected_commands, accepted_commands=self.accepted_commands,
                    arming_count=self.arming_count, state='ready' if ready else 'waiting_for_safe_feedback',
                    feedback_gate=feedback_diagnostics(self.feedback,self.clock_ns(),now-self.feedback_at),
                    disarm_counts=dict(self.disarm_counts), last_disarm=self.last_disarm,
                    rejection_counts=dict(self.reject_counts), last_rejection=self.last_rejection,
                    io_tick_max_gap_sec=self.max_tick_gap,
                    wall_durations=self.metrics.snapshot(now), logs=list(self.logger.records))
        return data
