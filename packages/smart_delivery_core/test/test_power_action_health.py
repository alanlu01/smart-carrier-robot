"""Unknown supply never reuses the previous healthy action sample."""
import json
import pytest
from smart_delivery_core.power_action_health import PowerActionHealth


def payload(status='full', **extra):
    return json.dumps({**{f'ch{i}':dict(current=.1,voltage=12.,status=status,sensor_ok=True)
                         for i in (1,2,3)},**extra})


def test_legacy_payloads_remain_supported_and_bad_data_invalidates_immediately():
    h=PowerActionHealth()
    assert len(h.observe(payload(),100.))==3 and h.valid
    with pytest.raises(ValueError):h.observe(payload('unknown'),100.5)
    assert not h.valid and not h.failure(100.5)
    with pytest.raises(ValueError):h.observe(payload('unknown'),105.5)
    assert '持續無效' in h.failure(105.5)
    assert not h.failure(110.)  # A stale fault is not a fresh confirmation.
    assert h.observe(payload(),111.) and h.valid


def test_confirmed_supply_failure_is_distinct_from_slot_presence():
    h=PowerActionHealth()
    h.observe(payload(),100.)
    with pytest.raises(ValueError):
        h.observe(payload('empty',slot_supply=dict(healthy=False,fault_confirmed=True)),101.)
    assert '供電不足' in h.failure(101.) and not h.valid
    with pytest.raises(ValueError):h.observe('null',102.)
    assert not h.confirmed_fault
    assert h.observe(payload(),103.) and h.valid


def test_brief_voltage_drop_allows_monitor_recovery_and_slot_debounce():
    from power_monitor.supply_health import SupplyHealth
    supply=SupplyHealth();h=PowerActionHealth()
    health=supply.observe([12.]*3,100.)
    h.observe(payload(slot_supply=health),100.)
    with pytest.raises(ValueError):
        h.observe(payload('unknown',slot_supply=supply.observe([0.]*3,100.5)),100.5)
    for at in (101.,101.5,102.,102.5,103.,103.5,104.,104.5,105.,105.5,106.,106.5):
        health=supply.observe([12.]*3,at)
        with pytest.raises(ValueError):h.observe(payload('unknown',slot_supply=health),at)
        assert not h.failure(at)
    assert health['healthy']
    assert h.observe(payload(slot_supply=supply.observe([12.]*3,107.)),107.) and h.valid


def test_recovery_metadata_cannot_extend_confirmation_forever():
    h=PowerActionHealth()
    for at in (100.,101.,102.,103.,104.,105.,106.,107.,108.,109.,110.):
        with pytest.raises(ValueError):
            h.observe(payload('unknown',slot_supply=dict(healthy=False,state='recovering')),at)
    assert h.failure(110.)


@pytest.mark.parametrize('persistent',[False,True])
def test_actual_delivery_wait_resets_confirmations_and_fails_a_confirmed_supply_fault(persistent):
    import ast
    import datetime
    import uuid
    from pathlib import Path
    from types import SimpleNamespace
    from smart_delivery_core.delivery_state import SlotActionVerifier
    source=Path(__file__).parents[1]/'smart_delivery_core/smart_delivery.py'
    main=next(x for x in ast.parse(source.read_text()).body
              if isinstance(x,ast.FunctionDef) and x.name=='_run_delivery')
    selected=[x for x in main.body if isinstance(x,ast.FunctionDef)
              and x.name in ('power_status_callback','wait_for_slot_action')]
    for definition in selected:
        definition.body=[ast.Global(names=x.names) if isinstance(x,ast.Nonlocal) else x
                         for x in definition.body]
    code=compile(ast.fix_missing_locations(ast.Module(body=selected,type_ignores=[])),str(source),'exec')
    clock=[100.];results=[];inhibits=[];samples=[]
    data=json.loads(payload());data['ch1']['status']='empty'
    empty=json.dumps(data)
    bad=payload('unknown',slot_supply=dict(healthy=False,fault_confirmed=persistent))
    events=[empty,empty,bad] if persistent else [empty,empty,bad,empty,empty,empty]
    h=PowerActionHealth();baseline=h.observe(payload(),100.)
    namespace=dict(PowerActionHealth=PowerActionHealth,power_health=h,
        latest_power_banks=baseline,latest_power_status_at=100.,latest_power_sequence=1,
        last_power_error_at=0.,json=json,time=SimpleNamespace(monotonic=lambda:clock[0],time=lambda:clock[0]),
        datetime=datetime.datetime,UTC=datetime.UTC,uuid=uuid,
        SlotActionVerifier=SlotActionVerifier,confirmation_samples=3,warning_seconds=30.,timeout_seconds=60.,
        POWER_STATUS_TIMEOUT_SECONDS=10.,active_task_id='test',cancellation_requests={},brief=str,
        navigator=SimpleNamespace(get_logger=lambda:SimpleNamespace(error=lambda *_:None,info=lambda *_:None)),
        set_motion_inhibited=inhibits.append,
        progress=lambda *args,**kwargs:None,publish_task_state=lambda *args,**kwargs:None,
        task_state_publisher=None,
        queue_result=lambda target,fsm,status,note:results.append((status,note,clock[0])))
    def spin(*args,**kwargs):
        clock[0]+=.5
        assert events,'Action did not finish after the required samples'
        text=events.pop(0);samples.append(text)
        namespace['power_status_callback'](SimpleNamespace(data=text))
    namespace['rclpy']=SimpleNamespace(ok=lambda:True,spin_once=spin)
    exec(code,namespace)
    namespace['wait_for_slot_action'](dict(type='borrow',slot_number=1,task_id='test'),
                                    SimpleNamespace(state='waiting_action'),baseline)
    assert inhibits==[True,False] and len(results)==1
    if persistent:
        assert results[0][0]=='failed' and '供電不足' in results[0][1]
    else:
        assert results[0][0]=='done' and len(samples)==6
