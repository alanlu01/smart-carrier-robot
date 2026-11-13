"""Delayed legacy lines cannot split source-stamped groups or replay stops."""
import pytest
from smart_delivery_core.atomic_wheel_feedback import AtomicWheelReceiver
from smart_delivery_core.wheel_feedback import WheelSampleAssembler, WheelStopEvidence


def sample(at=100., seq=1, session='rx', value=0.):
    return dict(schema_version=1, source='serial_rx', session=session,
                sequence=seq, sample_ros_stamp_ns=round(at*1e9),
                wheels={n:value for n in 'ABCD'})


def test_source_time_and_independent_zero_groups_survive_subscription_jitter():
    assembler=WheelSampleAssembler(.25, 0.)
    stop=WheelStopEvidence(0.)
    receiver=AtomicWheelReceiver(assembler, stop)
    for i,at in enumerate((100.,100.5,101.),1):
        assert receiver.observe(sample(at,i),round((at+.2)*1e9),at+.2) is not None
    assert assembler.last_complete_at==pytest.approx(101.)
    assert assembler.complete_sample_count==3 and stop.ready(101.2)
    # Repeated publication cannot add a fresh sample or extend the stop proof.
    assert receiver.observe(sample(101.,3),101300000000,101.3) is None
    assert not stop.ready(101.7)


@pytest.mark.parametrize('fault', ['late','future','nan','partial','bad_sequence','bool_stamp'])
def test_faulty_groups_cannot_update_wheel_or_stop_evidence(fault):
    a=WheelSampleAssembler(.25,0.);s=WheelStopEvidence(0.);r=AtomicWheelReceiver(a,s)
    p=sample();now=100100000000
    if fault=='late':now=101000000000
    elif fault=='future':now=99000000000
    elif fault=='nan':p['wheels']['C']=float('nan')
    elif fault=='partial':del p['wheels']['B']
    elif fault=='bad_sequence':p['sequence']=True
    else:p['sample_ros_stamp_ns']=True
    assert r.observe(p,now,100.1) is None
    assert a.last_complete_at is None and s.zero_groups==0


def test_reconnect_resets_stop_proof_and_old_sessions_cannot_replay():
    a=WheelSampleAssembler(.25,0.);s=WheelStopEvidence(0.);r=AtomicWheelReceiver(a,s)
    for i,at in enumerate((100.,100.5,101.),1):r.observe(sample(at,i),round(at*1e9),at)
    assert s.ready(101.)
    r.observe(sample(101.1,1,'new'),101100000000,101.1)
    assert not s.ready(101.1)
    assert r.observe(sample(101.,3,'rx'),101200000000,101.2) is None


def test_old_or_duplicate_sequence_cannot_rearm_with_new_stamp():
    r=AtomicWheelReceiver(WheelSampleAssembler(.25,0.))
    assert r.observe(sample(),100000000000,100.) is not None
    assert r.observe(sample(100.5,1),100500000000,100.5) is None



def test_receiver_identity_diagnostics_do_not_turn_duplicates_into_samples():
    receiver=AtomicWheelReceiver(WheelSampleAssembler(.25,0.))
    receiver.observe(sample(),100000000000,100.)
    receiver.observe(sample(),100100000000,100.1)
    receiver.observe(sample(101.,3),101000000000,101.)
    data=receiver.snapshot()
    assert data['accepted_sequence']==3 and data['accepted_groups']==2
    assert data['duplicate_groups']==1 and data['sequence_gaps']==1
    assert data['source_stamp_ns']==101000000000
