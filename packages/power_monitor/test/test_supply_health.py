"""Separate readable I2C data from a powered charging bus."""
import json
import pytest
from power_monitor.supply_health import SupplyHealth
from test_ina3221_transport import sensor


def test_power_loss_is_immediately_unusable_and_recovery_is_hysteretic():
    h=SupplyHealth()
    assert h.observe([12.]*3,100.)['healthy']
    assert h.observe([0.]*3,100.5)['state']=='supply_low'
    for at in (101.,101.5,102.,102.5):
        result=h.observe([0.]*3,at)
        assert not result['healthy']
    assert result['fault_confirmed']
    for at in (103.,104.,105.,106.,107.):
        assert not h.observe([12.]*3,at)['healthy']
    assert h.observe([12.]*3,108.)['healthy']
    assert not h.observe([12.,float('nan'),12.],108.5)['healthy']


def test_disabled_branch_is_ignored_but_enabled_branch_drop_is_not():
    h=SupplyHealth()
    assert h.observe([12.,12.,0.],1.,enabled=[True,True,False])['healthy']
    assert not h.observe([12.,0.,0.],1.5,enabled=[True,True,False])['healthy']
    assert not h.observe([12.,12.],2.)['healthy']


def test_sensor_restart_and_clock_gap_cannot_skip_recovery_debounce():
    h=SupplyHealth()
    h.observe([12.]*3,1.)
    assert h.observe([None]*3,1.5,sensor_ok=False)['state']=='sensor_unknown'
    assert h.observe([12.]*3,2.)['state']=='recovering'
    assert h.observe([12.]*3,20.)['state']=='recovering'


def test_real_power_callback_does_not_publish_false_empty_on_supply_cut(monkeypatch):
    from power_monitor import ina3221_node
    clock=[100.]
    monkeypatch.setattr(ina3221_node.time,'monotonic',lambda:clock[0])
    node,_=sensor(monkeypatch)
    node._probe_sensor(force=True)
    samples=[(.1,12.)]*3
    node._read_cycle=lambda:samples
    for _ in range(6):node.timer_callback();clock[0]+=.5
    assert json.loads(node.messages[-1].data)['ch1']['status']=='full'
    samples=[(0.,0.)]*3
    for _ in range(6):
        node.timer_callback();clock[0]+=.5
        data=json.loads(node.messages[-1].data)
        assert all(data[f'ch{i}']['status']=='unknown' for i in (1,2,3))
        assert all(data[f'ch{i}']['sensor_ok'] for i in (1,2,3))
    assert data['slot_supply']['fault_confirmed']
    assert not data['vehicle_battery']['source_verified']
