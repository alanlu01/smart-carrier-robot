"""IPC remains bounded and never waits for a stalled diagnostic reader."""
import multiprocessing
import pytest
from smart_delivery_core.serial_mailbox import LatestMailbox


def test_last_value_replaces_queue_and_original_time_is_preserved():
    slot=LatestMailbox(multiprocessing.get_context('spawn'),capacity=1024)
    for seq in range(100):assert slot.put(dict(sequence=seq,stamp=1000+seq))
    version,p=slot.get()
    assert p==dict(sequence=99,stamp=1099) and version==100
    assert slot.get(version) is None


def test_busy_or_crashed_reader_never_blocks_the_serial_owner():
    slot=LatestMailbox(multiprocessing.get_context('spawn'),capacity=1024)
    slot.put(dict(old=True))
    slot.lock.acquire()
    try:
        assert not slot.put(dict(new=True))
        assert slot.get() is None
        assert slot.skipped.value==1
    finally:slot.lock.release()
    assert slot.get()[1]==dict(old=True)
    with pytest.raises(ValueError):slot.put(dict(oversized='a'*2048))
