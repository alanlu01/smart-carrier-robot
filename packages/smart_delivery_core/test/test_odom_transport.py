"""Relay a paired snapshot without restamping or publishing motor commands."""
from types import SimpleNamespace
import json
from nav_msgs.msg import Odometry
from smart_delivery_core import odom_transport as module
from smart_delivery_core.pipeline_health import DurationMetrics


def harness(monkeypatch):
    moment=[100.]
    monkeypatch.setattr(module.time,'monotonic',lambda:moment[0])
    node=object.__new__(module.OdomTransport)
    node.received=node.published=node.dropped=node.errors=node.last_source_ns=0
    node.last_receive_at=node.last_publish_at=0.
    node.metrics=DurationMetrics()
    node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=int(moment[0]*1e9)))
    node.get_logger=lambda:SimpleNamespace(error=lambda _:None)
    odom=[];tf=[];health=[]
    node.odom_pub=SimpleNamespace(publish=odom.append)
    node.tf=SimpleNamespace(sendTransform=tf.append)
    node.health_pub=SimpleNamespace(publish=health.append)
    return node,moment,odom,tf,health


def sample(at=100.):
    m=Odometry();m.header.stamp.sec=int(at);m.header.stamp.nanosec=int((at-int(at))*1e9)
    m.header.frame_id='odom';m.child_frame_id='base_footprint'
    m.pose.pose.orientation.w=1.;m.pose.pose.position.x=1.
    return m


def test_pair_is_identical_and_old_duplicate_future_invalid_samples_are_dropped(monkeypatch):
    n,t,odom,tf,_=harness(monkeypatch)
    m=sample();n.receive(m)
    assert len(odom)==len(tf)==1 and odom[0].header.stamp==tf[0].header.stamp
    assert tf[0].transform.translation.x==1.
    for bad in (sample(),sample(99.),sample(102.)):
        n.receive(bad)
    bad=sample(100.01);bad.pose.pose.orientation.w=0.;n.receive(bad)
    assert n.dropped==4 and n.published==1


def test_slow_tf_cannot_send_stale_odom_and_health_reports_source_age(monkeypatch):
    n,t,odom,tf,health=harness(monkeypatch)
    def slow(message):
        tf.append(message);t[0]+=3.
    n.tf.sendTransform=slow
    n.receive(sample())
    assert len(tf)==1 and not odom and n.dropped==1
    n.health();assert json.loads(health[-1].data)['state']=='stale'
    assert n.metrics.snapshot(t[0])['tf_publish']['max_sec']==3.


def test_transport_failure_does_not_terminate_the_relay(monkeypatch):
    n,_,_,_,_=harness(monkeypatch)
    def fail(_):raise RuntimeError('synthetic publisher timeout')
    n.tf.sendTransform=fail;n.receive(sample())
    assert n.errors==1 and n.published==0
