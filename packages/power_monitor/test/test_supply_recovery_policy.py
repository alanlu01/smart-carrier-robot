"""Fresh charging measurements recover a transient I2C gap, not a power loss."""
from power_monitor.supply_health import SupplyHealth


def test_three_new_good_cycles_restore_only_a_short_communication_gap():
    h=SupplyHealth()
    assert h.observe([12.]*3,100.)['healthy']
    assert not h.observe([None]*3,100.5,False)['healthy']
    assert not h.observe([12.]*3,101.)['healthy']
    assert not h.observe([12.]*3,101.,True)['healthy']  # Retransmission is not a sample.
    assert not h.observe([12.]*3,101.5)['healthy']
    assert h.observe([12.]*3,102.)['healthy']


def test_real_low_voltage_discards_fast_communication_recovery():
    h=SupplyHealth();h.observe([12.]*3,100.)
    h.observe([None]*3,100.5,False)
    h.observe([9.]*3,101.)
    for t in (101.5,102.,102.5,103.,104.,105.,106.):
        assert not h.observe([12.]*3,t)['healthy']
    assert h.observe([12.]*3,106.5)['healthy']


def test_long_sensor_outage_or_clock_epoch_cannot_use_three_sample_shortcut():
    for epoch in (True,False):
        h=SupplyHealth();h.observe([12.]*3,100.)
        h.observe([None]*3,100.5,False)
        base=1. if epoch else 104.
        assert not h.observe([12.]*3,base)['healthy']
        assert not h.observe([12.]*3,base+.5)['healthy']
        assert not h.observe([12.]*3,base+1.)['healthy']
