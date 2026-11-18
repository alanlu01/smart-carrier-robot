"""Contention may delay a fresh group, never refresh or queue old evidence."""
from contextlib import nullcontext
from types import SimpleNamespace

import pytest
from smart_delivery_core.wheel_handoff import WheelHandoff


def sample(seq=786, stamp=100_000_000_000, session='usb-1'):
    return dict(sequence=seq, sample_ros_stamp_ns=stamp, session=session,
                wheels={'A': 0., 'B': 0., 'C': 0., 'D': 0.})


def test_busy_mailbox_delivers_original_group_on_next_tick():
    delivered = []
    busy = [True]
    def put(value):
        if busy[0]:
            return False
        delivered.append(value.copy())
        return True
    handoff = WheelHandoff(SimpleNamespace(put=put))
    original = sample()
    handoff.offer(original, 10.)
    handoff.flush(10., 100_000_000_000, session='usb-1', valid=True)
    busy[0] = False
    handoff.flush(10.05, 100_050_000_000, session='usb-1', valid=True)
    handoff.flush(10.10, 100_100_000_000, session='usb-1', valid=True)
    assert delivered == [original]
    assert handoff.snapshot()['write_failures'] == 1
    assert handoff.snapshot()['retries'] == 1
    assert handoff.last_delivered == 786


@pytest.mark.parametrize('now,clock,valid,session', [
    (10.21, 100_210_000_000, True, 'usb-1'),
    (10.05, 100_651_000_000, True, 'usb-1'),
    (10.05, 99_999_999_999, True, 'usb-1'),
    (9.99, 100_050_000_000, True, 'usb-1'),
    (10.05, 100_050_000_000, False, 'usb-1'),
    (10.05, 100_050_000_000, True, 'usb-2'),
])
def test_expiry_clock_change_or_disconnect_discards_pending(now, clock, valid, session):
    calls = []
    handoff = WheelHandoff(SimpleNamespace(put=lambda p: calls.append(p) or False))
    handoff.offer(sample(), 10.)
    handoff.flush(now, clock, valid=valid, session=session)
    assert not calls and handoff.pending is None


def test_latest_replaces_pending_and_contention_is_attempt_bounded():
    calls = []
    handoff = WheelHandoff(SimpleNamespace(put=lambda p: calls.append(p) or False))
    handoff.offer(sample(), 10.)
    handoff.offer(sample(seq=787), 10.01)
    for _ in range(20):
        handoff.flush(10.02, 100_020_000_000, valid=True, session='usb-1')
    assert len(calls) == 5 and {p['sequence'] for p in calls} == {787}
    assert handoff.pending is None and handoff.superseded == 1


def test_receive_worker_retries_without_waiting_for_another_mcu_group(monkeypatch):
    from smart_delivery_core import serial_workers as module

    clock, writes, health = [100.], [], []
    original = sample(stamp=100_000_000_000)
    engine = SimpleNamespace(session='usb-1', complete_ros_ns=100_000_000_000,
        frames=0, link=SimpleNamespace(close=lambda: None),
        metrics=SimpleNamespace(measure=lambda _: nullcontext()),
        health=lambda _: {})
    def receive(_):
        if engine.frames:
            return None
        engine.frames = 1
        return original
    engine.step = receive
    engine.raw = []
    def write(payload):
        writes.append(payload.copy())
        return len(writes) > 1
    def wait(_):
        clock[0] += .05
    stop = SimpleNamespace(is_set=lambda: clock[0] >= 100.3, wait=wait)
    monkeypatch.setattr(module, 'protect_parent', lambda *_: [False])
    monkeypatch.setattr(module, 'SerialReceiver', lambda *_: engine)
    monkeypatch.setattr(module, 'time', SimpleNamespace(
        monotonic=lambda: clock[0], time_ns=lambda: int(clock[0]*1e9)))
    module.receive_worker('fake', 115200, stop, SimpleNamespace(put=write),
        SimpleNamespace(put=lambda p: health.append(p)), SimpleNamespace(put=lambda _: True))
    assert writes == [original, original]
    assert health[-1]['wheel_ipc']['received_groups'] == 1
    assert health[-1]['wheel_ipc']['delivered_groups'] == 1
    assert health[-1]['wheel_ipc']['retries'] == 1
