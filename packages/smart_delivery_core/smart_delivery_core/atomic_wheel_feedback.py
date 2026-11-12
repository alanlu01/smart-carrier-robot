"""One source-stamped complete wheel group for all local consumers.

The timestamp is serial-receiver time, not an invented STM32 acquisition time.
Legacy text remains recorded, but subscription scheduling cannot split a group.
"""
import math


class AtomicWheelReceiver:
    def __init__(self, assembler, stop=None):
        self.assembler, self.stop = assembler, stop
        self.session = None
        self.sequence = -1
        self.stamp_ns = 0
        self.last_ros_now_ns = None
        self.retired_sessions = []

    def observe(self, payload, ros_now_ns, now, max_age=.65):
        # Wall-clock corrections invalidate the old stop/source epoch. Keep
        # sequence identity so delayed duplicate wheel groups cannot rearm it.
        if self.last_ros_now_ns is not None and ros_now_ns<self.last_ros_now_ns-500_000_000:
            self.stamp_ns=0
            if self.stop is not None:self.stop.reset()
            if self.assembler is not None:
                self.assembler.last_complete_at=None
                self.assembler._pending.clear()
                self.assembler._batch_started_at=None
        self.last_ros_now_ns=ros_now_ns
        try:
            stamp, seq, session = (payload['sample_ros_stamp_ns'],
                                   payload['sequence'], payload['session'])
            wheels = payload['wheels']
            if (payload['schema_version'] != 1 or payload['source'] != 'serial_rx'
                    or not isinstance(stamp, int) or isinstance(stamp, bool) or stamp <= 0
                    or not isinstance(seq, int) or isinstance(seq, bool) or seq < 1
                    or not isinstance(session, str) or not session or session in self.retired_sessions
                    or set(wheels) != set('ABCD')
                    or any(isinstance(v, bool) or not isinstance(v, (int, float))
                           or not math.isfinite(v) for v in wheels.values())):
                return None
            age = (ros_now_ns-stamp)/1e9
            if (not -.1 <= age <= max_age or stamp <= self.stamp_ns
                    or (session == self.session and seq <= self.sequence)):
                return None
            observed_at = now-max(0., age)
            if session != self.session:
                if self.session is not None:
                    self.retired_sessions=(self.retired_sessions+[self.session])[-8:]
                if self.stop is not None:self.stop.reset()
            self.session, self.sequence, self.stamp_ns = session, seq, stamp
            if self.assembler is not None:
                self.assembler.add_snapshot(wheels, observed_at)
            if self.stop is not None:
                # Preserve its existing three independent zero groups / 1 s proof.
                for name in 'ABCD':
                    self.stop.observe(f'{name}_RT: {wheels[name]}', observed_at)
            return dict(wheels)
        except (KeyError, TypeError, ValueError, OverflowError):
            return None
