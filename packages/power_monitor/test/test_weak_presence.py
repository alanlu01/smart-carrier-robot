"""Healthy persistent weak load vs brief empty-slot noise and outages."""
import pytest
from power_monitor.power_status import SlotStateTracker


def tracker():
    return SlotStateTracker(confirm_samples=3,weak_presence_min_a=.010)


def test_low_current_full_can_be_detected_from_cold_start():
    s=tracker()
    for i in range(14):assert s.update(.015,11.2,now=i*.5)=='unknown'
    assert s.update(.015,11.2,now=7.)=='full'
    assert s.evidence()['method']=='sustained_low_current'
    assert s.evidence()['weak_samples']==15


def test_short_empty_noise_never_creates_bank_and_does_not_slow_removal():
    s=tracker()
    for i in range(3):s.update(.004,11.2,now=i*.5)
    for i in range(6):assert s.update(.016,11.2,now=1.5+i*.5)=='empty'
    assert s.update(.004,11.2,now=4.5)=='empty'
    for i in range(3):s.update(.060,11.2,now=5.+i*.5)
    assert s.state=='full'
    for i in range(3):s.update(.004,11.2,now=6.5+i*.5)
    assert s.state=='empty'


@pytest.mark.parametrize('fault',['gap','sensor','voltage','duplicate','clock'])
def test_weak_evidence_cannot_cross_invalid_samples_or_time_gaps(fault):
    s=tracker()
    for i in range(10):s.update(.015,11.2,now=i*.5)
    if fault=='gap':s.update(.015,11.2,now=7.)
    elif fault=='sensor':s.update(None,None,sensor_ok=False,now=5.)
    elif fault=='voltage':s.update(.015,0.,now=5.)
    elif fault=='clock':s.update(.015,11.2,now=2.)
    else:
        for _ in range(100):s.update(.015,11.2,now=4.5)
    assert s.state=='unknown'
    assert s.update(.015,11.2,now=7.5)=='unknown'


def test_recovery_requires_new_current_evidence_and_not_old_full_memory():
    s=tracker()
    for i in range(3):s.update(.06,11.2,now=i*.5)
    assert s.state=='full'
    assert s.update(None,None,sensor_ok=False,now=2.)=='unknown'
    for i in range(3):s.update(.004,11.2,now=2.5+i*.5)
    assert s.state=='empty'
    for i in range(3):s.update(.06,11.2,now=4.+i*.5)
    assert s.state=='full'  # Normal return confirmation remains three samples.


def test_below_weak_floor_and_invalid_numbers_do_not_invent_presence():
    s=tracker()
    for i in range(30):assert s.update(.009,11.2,now=i*.5)=='unknown'
    assert s.update(float('nan'),11.2,now=15.)=='unknown'
    assert s.evidence()['weak_samples']==0


def test_real_ina_timer_publishes_weak_evidence_and_empty_after_removal(monkeypatch):
    import json
    from power_monitor import ina3221_node
    from test_ina3221_transport import sensor
    clock=[100.];monkeypatch.setattr(ina3221_node.time,'monotonic',lambda:clock[0])
    node,bus=sensor(monkeypatch)
    assert node._probe_sensor(force=True)
    node.state_trackers=[tracker() for _ in range(3)]
    current=[.015]
    node._read_cycle=lambda:[(current[0],11.2),(.004,11.2),(.005,11.2)]
    for i in range(15):
        clock[0]=100.+i*.5;node.timer_callback()
    data=json.loads(node.messages[-1].data)
    assert data['ch1']['status']=='full'
    assert data['ch1']['presence_evidence']['method']=='sustained_low_current'
    assert data['ch2']['status']==data['ch3']['status']=='empty'
    current[0]=.004
    for i in range(10):
        clock[0]+=.5;node.timer_callback()
    assert json.loads(node.messages[-1].data)['ch1']['status']=='empty'
