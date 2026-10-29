"""Actual logger severity transitions and bounded fatal cleanup; no motor I/O."""

from types import SimpleNamespace

import pytest
from rclpy.logging import get_logger

from smart_delivery_core.optional_idle_runtime import OptionalIdleRuntime
from smart_delivery_core import delivery_failure


def test_real_rcutils_idle_moving_idle_severity_transitions():
    runtime = object.__new__(OptionalIdleRuntime)
    runtime.home = None
    runtime.map_matches, runtime.inhibited = True, False
    runtime.ready = lambda: True
    runtime.last_odom = None
    runtime._pose = lambda: None
    runtime.pose_report = {}
    runtime._localization_phase = lambda: 'LOCALIZED'
    logger = get_logger('native_delivery_severity_regression')
    runtime.navigator = SimpleNamespace(get_logger=lambda: logger,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)))
    for moving in (False, True, False, True, False):
        runtime.motion = moving
        runtime.state = 'observing' if moving else 'idle'
        runtime.safety_log_at = -float('inf')
        assert runtime._safe() is False
    runtime._pose=lambda:dict(x=0.,y=0.,yaw=0.)
    runtime.last_odom=(100.,SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=100,nanosec=0))))
    runtime._stamp_fresh=lambda *_:True
    assert runtime._safe() is True
    assert runtime.safety_console_waiting is False


@pytest.mark.parametrize('pending,stopped,drained', [(False,True,True),(True,True,True),
                                                    (False,False,True),(False,True,False)])
def test_failure_drains_action_and_requires_independent_stop(monkeypatch, pending, stopped, drained):
    clock = SimpleNamespace(now=100.)
    monkeypatch.setattr(delivery_failure.time, 'monotonic', lambda: clock.now)
    events = []
    result = SimpleNamespace(done=lambda: drained and clock.now >= 100.5)
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: result,
        cancel_goal_async=lambda: events.append('cancel') or SimpleNamespace(done=lambda: True))
    request = SimpleNamespace(done=lambda: clock.now >= 100.2, result=lambda: handle)
    runtime = SimpleNamespace(observation_collecting=True, cancel_requested=False,
        suspend_lease=lambda: events.append('suspend'), goal_request=request if pending else None,
        navigator=SimpleNamespace(goal_handle=handle, result_future=result),
        _stationary=lambda: stopped)
    def spin():
        clock.now += .05
    report = delivery_failure.stop_after_failure(runtime, timeout=3., spin_once=spin)
    assert events[0] == 'suspend' and 'cancel' in events
    assert report['stopped'] == (stopped and drained)
    assert not runtime.observation_collecting and runtime.cancel_requested
    assert clock.now <= 103.1


def test_entrypoint_retains_original_exception_after_cleanup(monkeypatch):
    from smart_delivery_core import smart_delivery
    messages, cleaned = [], []
    runtime = SimpleNamespace(navigator=SimpleNamespace(get_logger=lambda: SimpleNamespace(error=messages.append)))
    original = ValueError('injected optional failure')
    def run(holder):
        holder.append(runtime)
        raise original
    monkeypatch.setattr(smart_delivery, '_run_delivery', run)
    monkeypatch.setattr(delivery_failure, 'stop_after_failure', lambda r: cleaned.append(r) or {'stopped':False})
    with pytest.raises(ValueError) as caught:
        smart_delivery.main()
    assert caught.value is original and cleaned == [runtime]
    assert 'ValueError: injected optional failure' in messages[0]


@pytest.mark.parametrize('stopped',[True,False])
def test_requested_stop_has_no_traceback_and_restores_signal_handlers(monkeypatch,stopped):
    import signal
    from smart_delivery_core import smart_delivery
    from smart_delivery_core.delivery_signals import DeliveryStop
    before=signal.getsignal(signal.SIGINT)
    messages=[]
    runtime=SimpleNamespace(navigator=SimpleNamespace(get_logger=lambda:SimpleNamespace(
        info=messages.append,warning=messages.append,error=lambda _:pytest.fail('normal stop logged as error'))))
    def run(holder):
        holder.append(runtime)
        raise DeliveryStop()
    monkeypatch.setattr(smart_delivery,'_run_delivery',run)
    monkeypatch.setattr(delivery_failure,'stop_after_failure',lambda _:dict(stopped=stopped))
    smart_delivery.main()
    assert len(messages)==1 and 'Traceback' not in messages[0]
    assert signal.getsignal(signal.SIGINT)==before


def test_invalid_ros_context_returns_without_retrying_executor(monkeypatch):
    import rclpy
    calls=[]
    monkeypatch.setattr(rclpy,'ok',lambda:False)
    monkeypatch.setattr(rclpy,'spin_once',lambda *_args,**_kwargs:pytest.fail('invalid context spun'))
    runtime=SimpleNamespace(observation_collecting=True,cancel_requested=False,
                            suspend_lease=lambda:calls.append('suspend'))
    report=delivery_failure.stop_after_failure(runtime)
    assert report['reason']=='ros_context_unavailable' and not report['stopped']
    assert calls==['suspend']
