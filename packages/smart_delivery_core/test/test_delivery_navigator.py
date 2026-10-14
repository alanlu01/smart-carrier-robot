"""Actual startup method with synthetic time/clients, no ROS node constructor."""
from types import SimpleNamespace
import pytest
from smart_delivery_core import delivery_navigator as module


class Future:
    def __init__(self, value=None):
        self.value=value
        self.cancelled=False
    def done(self):
        return self.value is not None
    def result(self):
        return self.value
    def cancel(self):
        self.cancelled=True


def harness(monkeypatch, modes):
    clock=SimpleNamespace(now=0., running=True)
    monkeypatch.setattr(module,'time',SimpleNamespace(monotonic=lambda:clock.now))
    monkeypatch.setattr(module.rclpy,'ok',lambda:clock.running)
    monkeypatch.setattr(module.rclpy,'spin_once',lambda *_a,**_k:setattr(clock,'now',clock.now+.1))
    node=object.__new__(module.DeliveryNavigator)
    node.fake_clients=[];node.messages=[];node.destroyed=[]
    node.startup_pub=SimpleNamespace(publish=node.messages.append)
    node.get_logger=lambda:SimpleNamespace(info=lambda *_:None)
    def create(_type,_service):
        mode=modes[min(len(node.fake_clients),len(modes)-1)]
        future=Future()
        if mode in ('active','inactive'):
            future.value=SimpleNamespace(current_state=SimpleNamespace(
                id=3 if mode=='active' else 1,label=mode))
        client=SimpleNamespace(service_is_ready=lambda:mode!='absent',
            call_async=lambda _:future,remove_pending_request=lambda _:None,future=future)
        node.fake_clients.append(client)
        return client
    node.create_client=create
    node.destroy_client=node.destroyed.append
    return node,clock


def test_rpc_timeout_rebuilds_client_and_can_recover_without_resetting_nav2(monkeypatch):
    node,clock=harness(monkeypatch,['timeout','active'])
    node._waitForNodeToActivate('bt_navigator')
    assert len(node.fake_clients)==2 and len(node.destroyed)==2
    assert node.fake_clients[0].future.cancelled
    assert clock.now < 5.
    assert '"state": "ready"' in node.messages[-1].data


@pytest.mark.parametrize('mode',['absent','timeout','inactive'])
def test_unavailable_activation_exits_within_total_deadline(monkeypatch,mode):
    node,clock=harness(monkeypatch,[mode])
    with pytest.raises(module.Nav2StartupUnavailable,match='保留訂單'):
        node._waitForNodeToActivate('bt_navigator')
    assert 30. <= clock.now < 30.2
    assert len(node.destroyed)==len(node.fake_clients)
    assert '"state": "failed"' in node.messages[-1].data


def test_context_shutdown_is_interruptible_and_cleans_client(monkeypatch):
    node,clock=harness(monkeypatch,['timeout'])
    monkeypatch.setattr(module.rclpy,'spin_once',lambda *_a,**_k:setattr(clock,'running',False))
    with pytest.raises(module.Nav2StartupUnavailable,match='ROS context stopped'):
        node._waitForNodeToActivate('bt_navigator')
    assert len(node.destroyed)==1 and node.fake_clients[0].future.cancelled


def test_client_creation_failure_is_bounded_and_retried(monkeypatch):
    node,clock=harness(monkeypatch,['active'])
    create=node.create_client
    calls=[]
    def flaky(*args):
        calls.append(1)
        if len(calls)==1:
            raise RuntimeError('temporary DDS creation failure')
        return create(*args)
    node.create_client=flaky
    node._waitForNodeToActivate('bt_navigator')
    assert len(calls)==2 and 1. <= clock.now < 2.
    assert len(node.destroyed)==1
