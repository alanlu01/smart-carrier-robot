"""Real asynchronous navigation client against a localhost fake action server."""
import json,os,time
from types import SimpleNamespace as NS
import rclpy
from rclpy.action import ActionServer,CancelResponse,GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_simple_commander.robot_navigator import TaskResult
from smart_delivery_core.delivery_navigator import DeliveryNavigator
from smart_delivery_core.navigation_wait import NavigationOwnershipUncertain
from smart_delivery_core.delivery_failure import stop_after_failure


def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='231'
    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')=='LOCALHOST'
    assert not os.environ.get('ROS_STATIC_PEERS')
    rclpy.init();client=DeliveryNavigator();server_node=rclpy.create_node('codex_fake_delivery_server')
    executor=MultiThreadedExecutor(num_threads=3)
    executor.add_node(client);executor.add_node(server_node)
    model=dict(mode='success',goals=0,cancels=0,closing=False)
    def accept(goal):
        if model['mode']=='late':time.sleep(.5)
        return GoalResponse.ACCEPT
    def execute(handle):
        model['goals']+=1;started=time.monotonic()
        while not model['closing']:
            if handle.is_cancel_requested:
                model['cancels']+=1;handle.canceled();return NavigateToPose.Result()
            if model['mode']=='success' and time.monotonic()-started>.2:
                handle.succeed();return NavigateToPose.Result()
            time.sleep(.01)
        handle.abort();return NavigateToPose.Result()
    server=ActionServer(server_node,NavigateToPose,'/navigate_to_pose',execute,
        goal_callback=accept,cancel_callback=lambda _:CancelResponse.ACCEPT,
        callback_group=ReentrantCallbackGroup())
    def spin():executor.spin_once(timeout_sec=.01)
    def finish():
        deadline=time.monotonic()+5.
        while not client.isTaskComplete() and time.monotonic()<deadline:spin()
        assert client.isTaskComplete()
    try:
        deadline=time.monotonic()+5.
        while not client.nav_to_pose_client.server_is_ready() and time.monotonic()<deadline:spin()
        assert client.nav_to_pose_client.server_is_ready()
        before=time.monotonic();client.goToPose(PoseStamped())
        assert time.monotonic()-before < .05
        finish();assert client.getResult()==TaskResult.SUCCEEDED
        model['mode']='cancel';client.goToPose(PoseStamped())
        deadline=time.monotonic()+5.
        while client.goal_handle is None and time.monotonic()<deadline:
            client.isTaskComplete();spin()
        assert client.goal_handle is not None
        before=time.monotonic();client.cancelTask()
        assert time.monotonic()-before < .05
        finish();assert client.getResult()==TaskResult.CANCELED
        model['mode']='late';client.goToPose(PoseStamped());client._delivery_wait.ack_timeout=.2
        uncertain=False;deadline=time.monotonic()+2.
        while time.monotonic()<deadline:
            try:client.isTaskComplete()
            except NavigationOwnershipUncertain:uncertain=True;break
            spin()
        assert uncertain and client.pending_goal_request is not None
        runtime=NS(navigator=client,goal_request=None,observation_collecting=False,
                   suspend_lease=lambda:None,_stationary=lambda:True)
        stopped=stop_after_failure(runtime,timeout=3.,spin_once=spin)
        assert stopped['stopped'] and stopped['action_drained']
        until=time.monotonic()+.3
        while time.monotonic()<until:spin()
        print(json.dumps(dict(nonblocking_goal=True,nonblocking_cancel=True,
            terminal_results_verified=True,late_acceptance_safely_drained=True,
            goals=model['goals'],cancels=model['cancels'],hardware_commands=0)))
    finally:
        model['closing']=True
        executor._executor.shutdown(wait=True)
        for future in executor._futures:
            if future.done():future.result()
        executor.shutdown();server.destroy();client.destroy_node();server_node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
