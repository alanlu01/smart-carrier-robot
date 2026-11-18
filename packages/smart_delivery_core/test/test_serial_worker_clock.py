"""The actual TX worker must sample age after acquiring the RX snapshot."""
from types import SimpleNamespace

from smart_delivery_core import serial_workers
from smart_delivery_core.serial_io import SerialTransmitter, ZERO


def test_rx_update_between_tick_and_get_does_not_manufacture_negative_age(monkeypatch):
    now = [100.]
    writes = []
    link = SimpleNamespace(session='tx', port=object(), poll=lambda at: True,
                           write=lambda data, at: writes.append(data) or True,
                           close=lambda: None, snapshot=lambda at: dict(connected=True))
    engine = SerialTransmitter('unused', 115200, link=link,
                               clock_ns=lambda: round(now[0]*1e9))
    engine.session, engine.opened_ros_ns = 'tx', 99000000000
    checks = []
    original = engine.poll
    def poll(at):
        checks.append(at-engine.feedback_at)
        return original(at)
    engine.poll = poll
    monkeypatch.setattr(serial_workers, 'protect_parent', lambda *args: [False])
    monkeypatch.setattr(serial_workers, 'SerialTransmitter', lambda *args: engine)
    monkeypatch.setattr(serial_workers.time, 'monotonic', lambda: now[0])
    class Feedback:
        def get(self, _):
            now[0] += .00046554
            return 1, dict(received_at=now[0], payload=dict(
                healthy=True, connected=True, session='rx', stop_proven=True,
                complete_sample_age_sec=.05, sample_ros_stamp_ns=round(now[0]*1e9),
                complete_ros_stamp_ns=round((now[0]-.05)*1e9)))
    stopped = [False]
    stop = SimpleNamespace(is_set=lambda: stopped[0],
                           wait=lambda _: stopped.__setitem__(0, True))
    empty = SimpleNamespace(get=lambda _: None)
    serial_workers.transmit_worker('unused', 115200, stop, empty, Feedback(),
                                  SimpleNamespace(put=lambda _: None))
    assert checks and checks[0] == 0.
    assert engine.armed
    assert writes and all(value == ZERO for value in writes)


def test_genuinely_future_receipt_still_fails_closed():
    from smart_delivery_core.serial_recovery import fresh_feedback
    assert not fresh_feedback(dict(healthy=True, connected=True), 100000000000, -.001)
