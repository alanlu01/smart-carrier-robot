from smart_delivery_core.navigation_health import TARGETS,navigation_snapshot


def test_all_lifecycle_services_need_recent_active_responses():
    data={n:dict(id=3,label='active',at=100.) for n in TARGETS}
    assert navigation_snapshot(data,101.)['state']=='ready'
    assert navigation_snapshot(data,104.)['state']=='unavailable'
    data['amcl']['id']=2
    assert navigation_snapshot(data,101.)['state']=='unavailable'
    data.pop('amcl')
    assert navigation_snapshot(data,101.)['state']=='unavailable'
