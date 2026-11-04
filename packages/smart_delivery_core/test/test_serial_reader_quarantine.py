"""An I/O scheduling outage must not turn buffered telemetry into fresh evidence."""
from types import SimpleNamespace
from smart_delivery_core.serial_io import SerialReceiver
from smart_delivery_core.serial_recovery import ReconnectingSerial


class RxPort:
    def __init__(self):self.buffer=b'';self.resets=0
    @property
    def in_waiting(self):return len(self.buffer)
    def reset_input_buffer(self):self.buffer=b'';self.resets+=1
    def read(self,n):result=self.buffer[:n];self.buffer=self.buffer[n:];return result
    def close(self):pass


def test_buffer_after_long_read_pause_is_discarded_and_sequence_never_rewinds(monkeypatch):
    from smart_delivery_core import serial_io as module
    now=[100.];monkeypatch.setattr(module.time,'monotonic',lambda:now[0])
    port=RxPort();logger=SimpleNamespace(info=lambda _:None,warning=lambda _:None)
    link=ReconnectingSerial('fake',115200,logger,factory=lambda *a,**k:port,identity=lambda _:1)
    rx=SerialReceiver('fake',115200,link=link);rx.step(now[0])
    def group(at,text=b'A_RT: 0\nB_RT: 0\nC_RT: 0\nD_RT: 0\n'):
        now[0]=at;port.buffer=text;return rx.step(at)
    first=group(100.5);second=group(101.)
    assert second['sequence']>first['sequence']
    assert group(102.) is None and not port.buffer and rx.backlog_resets==1
    assert not rx.health(102.)['healthy'] and rx.complete_ros_ns==0
    third=group(102.5)
    assert third['sequence']>second['sequence'] and not rx.health(102.5)['stop_proven']
    group(103.);group(103.5)
    assert rx.health(103.5)['stop_proven']
    assert group(104.,b'A_RT: 1e999\nB_RT: 0\nC_RT: 0\nD_RT: 0\n') is None
    assert not rx.health(104.)['healthy'] and rx.complete_ros_ns==0
