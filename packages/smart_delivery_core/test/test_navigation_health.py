from smart_delivery_core.navigation_health import TARGETS,navigation_snapshot
from smart_delivery_core.navigation_health import NavigationHealth
from smart_delivery_core.pipeline_health import ReceiptMetrics
from types import SimpleNamespace as NS


def test_all_lifecycle_services_need_recent_active_responses():
    data={n:dict(id=3,label='active',at=100.) for n in TARGETS}
    assert navigation_snapshot(data,101.)['state']=='ready'
    assert navigation_snapshot(data,104.)['state']=='unavailable'
    data['amcl']['id']=2
    assert navigation_snapshot(data,101.)['state']=='unavailable'
    data.pop('amcl')
    assert navigation_snapshot(data,101.)['state']=='unavailable'


def test_bond_diagnostics_distinguish_source_cadence_from_receipt_stall(monkeypatch):
    from smart_delivery_core import navigation_health as module
    clock=[100.]
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    monitor=object.__new__(NavigationHealth)
    monitor.bonds={};monitor.receipts=ReceiptMetrics()
    monitor.get_clock=lambda:NS(now=lambda:NS(nanoseconds=int(clock[0]*1e9)))
    def bond(stamp):
        return NS(id='amcl',instance_id='instance',active=True,
                  header=NS(stamp=NS(sec=stamp,nanosec=0)))
    monitor.observe_bond(bond(100))
    clock[0]=106.;monitor.observe_bond(bond(101))
    sample=monitor.bonds[('amcl','instance')]
    assert sample['max_receive_gap_sec']==6. and sample['max_source_gap_sec']==1.
    monitor.observe_tf(NS(transforms=[NS(header=NS(frame_id='map',stamp=NS(sec=105,nanosec=500000000)),
                                         child_frame_id='odom')]))
    assert monitor.receipts.snapshot(106.)['map->odom']['source_age_at_receive_sec']==.5


def test_map_server_late_reply_has_bounded_independently_verified_grace():
    records={n:dict(id=3,at=103.) for n in TARGETS}
    records['map_server']['at']=100.
    assert navigation_snapshot(records,103.1)['state']=='unavailable'
    assert navigation_snapshot(records,103.1,fresh_safety=True)['state']=='ready'
    assert navigation_snapshot(records,104.6,fresh_safety=True)['state']=='unavailable'
    records['map_server']['id']=2
    assert navigation_snapshot(records,103.1,fresh_safety=True)['state']=='unavailable'
    records['map_server']['id']=3
    records['amcl']['at']=100.
    assert navigation_snapshot(records,103.1,fresh_safety=True)['state']=='unavailable'
