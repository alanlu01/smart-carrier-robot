"""Input fanout, policy cadence and bounded ownership without hardware."""
from types import SimpleNamespace as NS
import pytest
from std_msgs.msg import String
from smart_delivery_core.delivery_inputs import shared_inputs,message_json,PolicyCadence
from smart_delivery_core.navigation_wait import NavigationWait,NavigationOwnershipUncertain
from smart_delivery_core.delivery_navigator import DeliveryNavigator
from smart_delivery_core import smart_delivery
from test_localization_recovery import manager
from smart_delivery_core.localization_manager import LocalizationManager
from tf2_ros import TransformException
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator


def test_one_subscription_decodes_once_and_preserves_event_payload(monkeypatch):
    callbacks=[];deliveries=[];decoded=[]
    node=NS(create_subscription=lambda kind,topic,cb,qos:callbacks.append(cb))
    inputs=shared_inputs(node)
    inputs.subscribe(String,'/localization/state',lambda m:deliveries.append(message_json(m)),1)
    shared_inputs(node).subscribe(String,'localization/state',lambda m:deliveries.append(message_json(m)),1)
    import smart_delivery_core.delivery_inputs as module
    loads=module.json.loads
    monkeypatch.setattr(module.json,'loads',lambda s:(decoded.append(s) or loads(s)))
    callbacks[0](String(data='{"ready":false,"sample_ros_stamp_ns":123}'))
    assert len(callbacks)==len(decoded)==1 and len(deliveries)==2
    assert deliveries[0] is deliveries[1] and not deliveries[0]['ready']


def test_high_rate_inputs_do_not_repeat_policy_but_urgent_cancel_wakes():
    cadence=PolicyCadence();runs=[]
    for i in range(1000):
        if cadence.due(i/1000):runs.append(i)
    assert 9 <= len(runs) <= 10
    assert not cadence.due(.999)
    cadence.wake()
    assert cadence.due(.999)


def test_cancel_wait_needs_terminal_result_not_just_cancel_ack():
    future=NS(done=lambda:False)
    handle=NS(accepted=True,get_result_async=lambda:future,cancel_goal_async=lambda:NS(done=lambda:True))
    wait=NavigationWait(100.);wait.request=NS(done=lambda:True,result=lambda:handle)
    assert not wait.poll(100.1)
    wait.cancel_once(100.2)
    assert not wait.poll(101.)
    with pytest.raises(NavigationOwnershipUncertain):wait.poll(105.3)
    assert wait.phase=='cancelling'


def test_acceptance_timeout_keeps_pending_request_for_stop_cleanup():
    wait=NavigationWait(100.);wait.request=NS(done=lambda:False)
    with pytest.raises(NavigationOwnershipUncertain):wait.poll(105.)
    assert wait.request is not None


def test_client_retains_native_executor_pumping_for_owned_actions():
    assert DeliveryNavigator.goToPose is BasicNavigator.goToPose
    assert DeliveryNavigator.isTaskComplete is BasicNavigator.isTaskComplete
    assert DeliveryNavigator.cancelTask is BasicNavigator.cancelTask


def test_standby_timeout_never_releases_the_guard(monkeypatch):
    events=[]
    node=NS(get_logger=lambda:NS(info=lambda *_:None),get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:None)),
            goToPose=lambda _:None,isTaskComplete=lambda:(_ for _ in ()).throw(NavigationOwnershipUncertain('lost')))
    with pytest.raises(NavigationOwnershipUncertain):
        smart_delivery.go_to_standby(node,{'x':0.,'y':0.},set_navigation_active=events.append)
    assert events==[True]


def test_map_lookup_is_nonblocking_and_never_marks_stale_tf_healthy(manager):
    node,clock=manager
    node.map_message=object();node.latest_scan=NS(header=NS(frame_id='laser',stamp=NS(sec=100,nanosec=0)))
    # Time.from_msg needs a real ROS time message.
    from builtin_interfaces.msg import Time
    node.latest_scan.header.stamp=Time(sec=100)
    durations=[]
    def lookup(*args,**kwargs):
        durations.append(kwargs['timeout'].nanoseconds)
        raise TransformException('not arrived')
    node.tf_buffer=NS(lookup_transform=lookup);node.map_match_tf_failures=0
    LocalizationManager._update_map_match_score(node,clock.now)
    assert durations==[0] and node.map_match_tf_failures==1
    assert 'not arrived' in node.last_map_match_error
