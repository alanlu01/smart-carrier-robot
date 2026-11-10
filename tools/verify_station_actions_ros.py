"""Native action/executor/lease integration with virtual poses, never velocity."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from smart_delivery_core.delivery_goal_guard import DeliveryGoalGuard, LEASE_QOS
from smart_delivery_core.delivery_navigator import DeliveryNavigator
from smart_delivery_core.station_navigation import StationNavigation, coordinates
from ament_index_python.packages import get_package_share_directory


def fixture():
    from rclpy.action import ActionServer, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init();node=rclpy.create_node('codex_station_action_fixture');guard=DeliveryGoalGuard()
    state=dict(pose=(0.,0.,0.),hold=False,goals=[],cancels=0)
    node.create_subscription(Bool,'/codex/station_hold',lambda m:state.update(hold=m.data),10)
    grid=node.create_publisher(OccupancyGrid,'/global_costmap/costmap',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    odom=node.create_publisher(Odometry,'/odom',10);tf=TransformBroadcaster(node)
    def publish():
        stamp=node.get_clock().now().to_msg();x,y,yaw=state['pose']
        tr=TransformStamped();tr.header.frame_id='map';tr.child_frame_id='base_footprint';tr.header.stamp=stamp
        tr.transform.translation.x=x;tr.transform.translation.y=y
        import math
        tr.transform.rotation.z=math.sin(yaw/2);tr.transform.rotation.w=math.cos(yaw/2);tf.sendTransform(tr)
        message=Odometry();message.header.stamp=stamp;message.pose.pose.orientation.w=1.;odom.publish(message)
    def publish_grid():
        message=OccupancyGrid();message.header.frame_id='map';message.header.stamp=node.get_clock().now().to_msg()
        message.info.resolution=.05;message.info.width=message.info.height=100
        message.info.origin.position.x=message.info.origin.position.y=-2.5
        message.info.origin.orientation.w=1.;message.data=[0]*10000;grid.publish(message)
    node.create_timer(.05,publish);node.create_timer(.5,publish_grid)
    def execute(handle):
        state['goals'].append(dict(pose=coordinates(handle.request.pose),tree=handle.request.behavior_tree))
        started=time.monotonic()
        while rclpy.ok() and (state['hold'] or time.monotonic()-started<.3):
            if handle.is_cancel_requested:
                state['cancels']+=1;handle.canceled();return NavigateToPose.Result()
            handle.publish_feedback(NavigateToPose.Feedback());time.sleep(.03)
        state['pose']=coordinates(handle.request.pose)
        handle.succeed();return NavigateToPose.Result()
    server=ActionServer(node,NavigateToPose,'/navigate_to_pose',execute,
        cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=ReentrantCallbackGroup())
    executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node);executor.add_node(guard)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        print(json.dumps(state),flush=True)
        executor.shutdown();server.destroy();node.destroy_node();guard.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='231'
    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')=='LOCALHOST'
    assert not os.environ.get('ROS_STATIC_PEERS')
    if '--fixture' in sys.argv:return fixture()
    root=Path(os.environ['ROS_LOG_DIR']);root.mkdir(parents=True,exist_ok=True)
    with (root/'action_fixture.log').open('w') as log:
        child=subprocess.Popen([sys.executable,__file__,'--fixture'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        rclpy.init();node=DeliveryNavigator();buffer=Buffer();listener=TransformListener(buffer,node)
        manager=StationNavigation(node,buffer,get_package_share_directory('smart_delivery_core'))
        lease=node.create_publisher(Bool,'/smart_carrier/delivery_navigation_active',LEASE_QOS)
        node.create_timer(.2,lambda:lease.publish(Bool(data=True)))
        hold=node.create_publisher(Bool,'/codex/station_hold',10)
        commands=[];node.create_subscription(Twist,'/cmd_vel',commands.append,10)
        def pump(seconds):
            end=time.monotonic()+seconds
            while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        def finish(limit=15.):
            end=time.monotonic()+limit
            while time.monotonic()<end:
                if manager.isTaskComplete():return manager.getResult()
                rclpy.spin_once(node,timeout_sec=.02)
            raise AssertionError('station action did not complete')
        def goal(x,y):
            message=PoseStamped();message.header.frame_id='map'
            message.pose.position.x=x;message.pose.position.y=y;message.pose.orientation.w=1.;return message
        try:
            assert node.nav_to_pose_client.wait_for_server(timeout_sec=10.)
            pump(1.)
            manager.start(goal(1.,0.));assert finish()==TaskResult.SUCCEEDED
            assert manager.parked==(1.,0.,0.)
            manager.start(goal(1.,1.));assert finish()==TaskResult.SUCCEEDED
            assert manager.parked==(1.,1.,0.)
            hold.publish(Bool(data=True));pump(.3)
            permit=[True];manager.start(goal(0.,0.),lambda:permit[0])
            end=time.monotonic()+3.
            while not manager.active_goal and time.monotonic()<end:
                manager.isTaskComplete();rclpy.spin_once(node,timeout_sec=.02)
            assert manager.active_goal
            permit[0]=False;assert finish()==TaskResult.CANCELED
            assert not commands, 'isolated action test must never publish velocity'
            print(json.dumps(dict(passed=True,park_x=True,departure_and_park_y=True,
                permission_cancel_drained=True,nonzero_velocity_commands=0,domain=231)))
        finally:
            if manager.running:
                manager.cancelTask();finish(5.)
            lease.publish(Bool(data=False));pump(.2)
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=8.)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5.)
            node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
