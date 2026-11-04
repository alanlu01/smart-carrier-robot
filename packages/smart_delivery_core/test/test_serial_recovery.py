"""Serial hotplug and safety tests use fake ports, never /dev/stm32."""
import json
from types import SimpleNamespace
import pytest
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from smart_delivery_core.serial_recovery import ReconnectingSerial,FeedbackReadiness,fresh_feedback
from smart_delivery_core.serial_io import SerialTransmitter,ZERO


class Port:
    def __init__(self):self.closed=False;self.writes=[];self.short=False
    def reset_input_buffer(self):pass
    def write(self,data):
        self.writes.append(data)
        return len(data)-1 if self.short else len(data)
    def close(self):self.closed=True


def link():
    state=dict(identity=1,present=True);ports=[];lines=[]
    def identity(_):
        if not state['present']:raise OSError('unplugged')
        return state['identity']
    def factory(*args,**kwargs):
        assert kwargs==dict(timeout=0,write_timeout=.10)
        port=Port();ports.append(port);return port
    logger=SimpleNamespace(info=lines.append,warning=lines.append)
    return ReconnectingSerial('fake',115200,logger,factory=factory,identity=identity),state,ports,lines


def test_unplug_backoff_and_new_device_close_old_fd_and_create_new_session():
    l,s,ports,lines=link();assert l.poll(100.)
    first=l.session;s['present']=False
    assert not l.poll(100.1) and ports[0].closed
    for i in range(200):l.poll(100.1+i*.05)
    assert len(lines)<=5 and len(ports)==1
    s['present']=True;s['identity']=2
    assert l.poll(120.) and l.session!=first and l.open_count==2


def test_same_path_replacement_and_short_write_fail_closed():
    l,s,ports,_=link();assert l.poll(100.)
    s['identity']=2
    assert not l.poll(100.1) and ports[0].closed
    assert l.poll(101.)
    ports[-1].short=True
    assert not l.write(b'zero',101.) and l.port is None


def groups(f,at,value=0.):
    for wheel in 'ABCD':f.observe(f'{wheel}_RT: {value}',at)


def test_only_complete_independent_recent_wheel_groups_can_authorize_motion():
    f=FeedbackReadiness(100.)
    groups(f,100.);assert not f.snapshot(100.)['healthy']
    groups(f,100.5);assert f.snapshot(100.5)['healthy'] and not f.snapshot(100.5)['stop_proven']
    groups(f,101.);assert f.snapshot(101.)['stop_proven']
    assert not f.snapshot(102.)['healthy']
    groups(f,102.);assert not f.snapshot(102.)['healthy']
    groups(f,102.5,value=5.);assert f.snapshot(102.5)['healthy'] and not f.snapshot(102.5)['stop_proven']


def payload(at,session='rx',stopped=True):
    return dict(healthy=True,connected=True,session=session,complete_sample_age_sec=.05,
                sample_ros_stamp_ns=round(at*1e9),complete_ros_stamp_ns=round((at-.05)*1e9),stop_proven=stopped)


@pytest.mark.parametrize('fault',['stale_source','stale_receive','old_complete','disconnected','unhealthy','nan','missing'])
def test_feedback_metadata_cannot_authorize_bad_or_delayed_evidence(fault):
    p=payload(100.);received=.05
    if fault=='stale_source':p['sample_ros_stamp_ns']=99000000000
    elif fault=='stale_receive':received=.5
    elif fault=='old_complete':p['complete_sample_age_sec']=.7
    elif fault=='disconnected':p['connected']=False
    elif fault=='unhealthy':p['healthy']=False
    elif fault=='nan':p['complete_sample_age_sec']=float('nan')
    else:p={}
    assert not fresh_feedback(p,100100000000,received)


def transmitter(monkeypatch):
    from smart_delivery_core import serial_io as module
    now=[100.]
    monkeypatch.setattr(module.time,'monotonic',lambda:now[0])
    l,s,ports,lines=link()
    n=SerialTransmitter('fake',115200,link=l,clock_ns=lambda:round(now[0]*1e9))
    def feedback(message):
        n.feedback=json.loads(message.data);n.feedback_at=now[0]
    def command(message,info=None):
        n.command(dict(values=[message.linear.x,message.linear.y,message.angular.z],
            source_stamp_ns=round(now[0]*1e9) if info is None else info.source_timestamp,
            received_at=now[0]))
    n.feedback_callback=feedback;n.cmd_vel_callback=command
    return n,now,s,ports


def test_real_transmitter_never_replays_motion_across_disconnect(monkeypatch):
    n,t,state,ports=transmitter(monkeypatch)
    n.poll();assert ports[-1].writes==[ZERO] and not n.armed
    m=Twist();m.linear.x=.3
    n.cmd_vel_callback(m);assert all(w==ZERO for w in ports[-1].writes)
    t[0]=100.2;n.feedback_callback(String(data=json.dumps(payload(t[0]))));n.poll()
    assert n.armed
    n.cmd_vel_callback(m);assert ports[-1].writes[-1]==b'$CMD,0.30,0.00,0.00#\n'
    state['present']=False;t[0]=100.3;n.poll();assert not n.armed
    n.cmd_vel_callback(m)
    state['present']=True;state['identity']=2;t[0]=101.;n.poll()
    assert not n.armed and ports[-1].writes==[ZERO]
    t[0]=101.2;n.feedback_callback(String(data=json.dumps(payload(t[0],session='new',stopped=False))));n.poll()
    assert not n.armed and all(w==ZERO for w in ports[-1].writes)
    t[0]=101.4;n.feedback_callback(String(data=json.dumps(payload(t[0],session='new'))));n.poll()
    assert n.armed and all(w==ZERO for w in ports[-1].writes)
    n.cmd_vel_callback(m,SimpleNamespace(source_timestamp=100000000000))
    assert all(w==ZERO for w in ports[-1].writes)
    n.cmd_vel_callback(m,SimpleNamespace(source_timestamp=round(t[0]*1e9)))
    assert ports[-1].writes[-1].startswith(b'$CMD,0.30')


def test_session_change_between_health_callback_and_timer_still_rejects_motion(monkeypatch):
    n,t,_,ports=transmitter(monkeypatch);n.poll();t[0]=100.2
    n.feedback_callback(String(data=json.dumps(payload(t[0]))));n.poll();assert n.armed
    n.feedback_callback(String(data=json.dumps(payload(t[0],session='replacement',stopped=False))))
    m=Twist();m.linear.y=.2;n.cmd_vel_callback(m)
    assert all(w==ZERO for w in ports[-1].writes)


def test_tx_disarm_preserves_exact_feedback_age_and_reason(monkeypatch):
    n,t,_,ports=transmitter(monkeypatch)
    n.poll();t[0]=100.2
    n.feedback_callback(String(data=json.dumps(payload(t[0]))));n.poll()
    assert n.armed
    for at in (100.3,100.4,100.5,100.6):
        t[0]=at;n.poll()
    data=n.health(t[0])
    assert not data['armed'] and data['last_disarm']['reason']=='feedback_source_age'
    assert data['last_disarm']['feedback']['source_age_sec'] > .35
    assert data['disarm_counts']['feedback_source_age']==1
    n.cmd_vel_callback(Twist())
    assert n.health(t[0])['last_rejection']['reason']=='not_armed'
    json.dumps(data,allow_nan=False)
    assert ports[-1].writes[-1]==ZERO


def test_frontend_preserves_jazzy_dictionary_source_timestamp_and_rejects_nan():
    from smart_delivery_core.cmd_vel_to_serial import CmdVelToSerialNode
    from smart_delivery_core.pipeline_health import DurationMetrics
    n=object.__new__(CmdVelToSerialNode);records=[]
    n.metrics=DurationMetrics();n.commands=SimpleNamespace(put=records.append)
    m=Twist();m.linear.x=.3
    n.cmd_vel_callback(m,dict(source_timestamp=123456789))
    assert records[-1]['source_stamp_ns']==123456789 and records[-1]['values'][0]==.3
    m.linear.x=float('nan');n.cmd_vel_callback(m,dict(source_timestamp=123456790))
    assert records[-1]['source_stamp_ns']==0 and records[-1]['values']==[0.,0.,0.]
