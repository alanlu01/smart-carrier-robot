"""Self SIGINT/SIGTERM in domain 231. No navigation actions or motor publishers."""
import os
import signal
import sys
import time
from types import SimpleNamespace
assert os.environ.get('ROS_DOMAIN_ID')=='231'
assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')=='LOCALHOST'
assert not os.environ.get('ROS_STATIC_PEERS')
import rclpy
from rclpy.signals import SignalHandlerOptions
from smart_delivery_core import smart_delivery

events=[]
probe=None
ticks=[]
def run(holder):
    global probe
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    probe=rclpy.create_node('codex_delivery_shutdown_probe')
    probe.goal_handle=probe.result_future=None
    fired=[]
    def tick():
        ticks.append(time.monotonic())
        if not fired:
            fired.append(True)
            os.kill(os.getpid(),getattr(signal,sys.argv[1] if len(sys.argv)>1 else 'SIGINT'))
    probe.create_timer(.02,tick)
    runtime=SimpleNamespace(navigator=probe,goal_request=None,observation_collecting=True,
        suspend_lease=lambda:events.append('suspend'),
        _stationary=lambda:bool(ticks) and time.monotonic()-ticks[-1]<.1)
    holder.append(runtime)
    while rclpy.ok():
        rclpy.spin_once(probe,timeout_sec=.1)
    raise AssertionError('context shut down before cancellation')

smart_delivery._run_delivery=run
started=time.monotonic()
try:
    smart_delivery.main()
    assert not rclpy.ok() and events==['suspend'] and len(ticks)>=20
    assert 1.<=time.monotonic()-started<4.
    print('Normal requested stop: live-context cancellation/stop proof, clean shutdown; zero goals/motor commands.')
finally:
    if probe:probe.destroy_node()
    if rclpy.ok():rclpy.shutdown()
