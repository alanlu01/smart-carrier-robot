"""Dropped DDS deliveries can recover without inventing independent samples."""
from smart_delivery_core.wheel_resend import WheelResend
from smart_delivery_core.atomic_wheel_feedback import AtomicWheelReceiver
from smart_delivery_core.wheel_feedback import WheelSampleAssembler, WheelStopEvidence


def payload(at=100., seq=1):
    return dict(schema_version=1,source='serial_rx',sample_ros_stamp_ns=round(at*1e9),
                sequence=seq,session='rx',wheels={w:0. for w in 'ABCD'})


def test_first_publication_lost_repeat_recovers_original_complete_group():
    resend=WheelResend();a=WheelSampleAssembler(.25,0.);stop=WheelStopEvidence(0.)
    rx=AtomicWheelReceiver(a,stop)
    original=payload()
    resend.offer(1,original,10.)
    duplicate=resend.due(10.11,100110000000)
    assert duplicate==original
    assert rx.observe(duplicate,100110000000,20.11) is not None
    at=a.last_complete_at
    assert rx.observe(resend.due(10.22,100220000000),100220000000,20.22) is None
    assert a.last_complete_at==at and a.complete_sample_count==1 and stop.zero_groups==1
    assert resend.due(10.33,100330000000) is None
    assert not stop.ready(20.22)


def test_no_resend_after_age_limit_or_clock_rewind_and_new_group_supersedes_old():
    r=WheelResend();r.offer(1,payload(),10.)
    assert r.due(11.,101000000000) is None
    r.offer(2,payload(),12.)
    assert r.due(12.2,99000000000) is None
    r.offer(3,payload(102.,2),13.)
    assert r.due(13.2,102200000000)['sequence']==2
