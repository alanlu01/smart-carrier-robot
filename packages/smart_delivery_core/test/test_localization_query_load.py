"""Executor checks must not block, queue duplicate requests or bypass safety."""
from types import SimpleNamespace as NS

from rclpy.task import Future
from smart_delivery_core import localization_manager as module


def test_sensor_tf_checks_are_nonblocking_and_fail_closed():
    calls=[];node=object.__new__(module.LocalizationManager)
    node.latest_scan=NS(header=NS(frame_id='laser'))
    def lookup(*args,**kw):
        calls.append((args,kw['timeout'].nanoseconds))
        if len(calls)==2:raise module.TransformException('not available')
    node.tf_buffer=NS(lookup_transform=lookup)
    assert not node._sensor_tf_is_ready()
    assert [timeout for _,timeout in calls]==[0,0]
    assert '感測器 TF 尚未就緒' in node.last_map_match_error


def test_nomotion_service_has_one_pending_request_and_bounded_timeout(monkeypatch):
    now=[100.];monkeypatch.setattr(module.time,'monotonic',lambda:now[0])
    node=object.__new__(module.LocalizationManager);sent=[];removed=[]
    def send(_request):
        future=Future();sent.append(future);return future
    node.nomotion_client=NS(service_is_ready=lambda:True,call_async=send,
        remove_pending_request=removed.append)
    assert node._request_nomotion_update()
    now[0]=100.5
    assert not node._request_nomotion_update() and len(sent)==1
    now[0]=102.1
    assert node._request_nomotion_update() and len(sent)==2
    assert removed==[sent[0]] and sent[0].cancelled()
    sent[1].set_result(NS());now[0]=102.6
    assert node._request_nomotion_update() and len(sent)==3
