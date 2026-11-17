"""Isolated optional/StationNavigation TF pause test; no plant velocity output."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='231'
    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')=='LOCALHOST'
    assert not os.environ.get('ROS_STATIC_PEERS')
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool,String
    from tf2_ros import Buffer,TransformListener
    from smart_delivery_core.delivery_goal_guard import LEASE_QOS
    from smart_delivery_core.delivery_navigator import DeliveryNavigator
    from smart_delivery_core.optional_idle import load_idle_config
    from smart_delivery_core.optional_idle_runtime import OptionalIdleRuntime
    from smart_delivery_core.station_navigation import StationNavigation
    root=Path(os.environ['ROS_LOG_DIR']);root.mkdir(parents=True,exist_ok=True)
    with (root/'fixture.log').open('w') as log:
        child=subprocess.Popen([sys.executable,str(Path(__file__).with_name('verify_station_actions_ros.py')),
            '--fixture','--slow-cancel'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        rclpy.init();node=DeliveryNavigator();buffer=Buffer();listener=TransformListener(buffer,node)
        share=Path(get_package_share_directory('smart_delivery_core'))
        config=load_idle_config(share/'config/optional_idle_features.yaml');config['home']['enabled']=False
        config['people']['observation_pose']=dict(name='TEST_OBSERVATION',x=1.,y=0.,yaw=0.)
        lease=node.create_publisher(Bool,'/smart_carrier/delivery_navigation_active',LEASE_QOS)
        active=[False]
        runtime=OptionalIdleRuntime(node,buffer,config,share,lambda:True,
            lambda enabled:active.__setitem__(0,enabled),lambda:active.__setitem__(0,False))
        runtime.map_matches=True
        station=StationNavigation(node,buffer,share);node.station_navigation=station
        node.create_timer(.2,lambda:lease.publish(Bool(data=active[0])))
        local=node.create_publisher(String,'/localization/state',LEASE_QOS)
        node.create_timer(.2,lambda:local.publish(String(data=json.dumps(dict(state='LOCALIZED',ready=True,
            sample_ros_stamp_ns=node.get_clock().now().nanoseconds)))))
        scan=node.create_publisher(LaserScan,'/scan_filtered',10)
        def laser():
            m=LaserScan();m.header.frame_id='base_footprint';m.header.stamp=node.get_clock().now().to_msg()
            m.angle_min=-math.pi;m.angle_increment=2*math.pi/720;m.range_min=.15;m.range_max=12.
            m.ranges=[2.]*720;scan.publish(m)
        node.create_timer(.05,laser)
        hold=node.create_publisher(Bool,'/codex/station_hold',10)
        tf_enabled=node.create_publisher(Bool,'/codex/station_tf_enabled',10)
        commands=[];node.create_subscription(Twist,'/cmd_vel',commands.append,10)
        max_poll=0.
        def pump(seconds,poll=True):
            nonlocal max_poll
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                if poll:
                    started=time.monotonic();runtime.poll(False)
                    max_poll=max(max_poll,time.monotonic()-started)
                rclpy.spin_once(node,timeout_sec=.02)
        def until(predicate,limit):
            end=time.monotonic()+limit
            while time.monotonic()<end:
                pump(.03)
                if predicate():return
            raise AssertionError((runtime.state,runtime.reason,runtime.localization_resume_blocker,station.phase))
        try:
            assert node.nav_to_pose_client.wait_for_server(timeout_sec=10.)
            pump(1.,False);hold.publish(Bool(data=True));pump(.3,False)
            assert runtime._start(config['people']['observation_pose'],'going_observation','',60.)
            deadline=runtime.deadline
            until(lambda:station.active_goal,5.)
            tf_enabled.publish(Bool(data=False))
            until(lambda:runtime.state=='waiting_people_localization' and not runtime.settling_after_cancel,8.)
            assert station.goals_submitted==1 and not station.active_goal
            assert runtime.localization_resume_deadline==deadline and runtime.data_pauses==1
            pump(.5);assert runtime.state=='waiting_people_localization' and station.goals_submitted==1
            tf_enabled.publish(Bool(data=True));hold.publish(Bool(data=False))
            until(lambda:runtime.state=='waiting_observation_stop',8.)
            assert runtime.localization_resume_blocker=='ready' and runtime.localization_pauses==0
            assert runtime.data_pauses==1 and max_poll<.5 and not commands
            assert abs(station.target[0]-1.)<1e-9
            print(json.dumps(dict(passed=True,domain=231,optional_tf_pause=True,
                delayed_cancel_ack_sec=3.,max_poll_sec=max_poll,original_deadline_preserved=True,
                post_pause_sources_and_stop_required=True,resumed_original_observation_point=True,
                nonzero_velocity_commands=0)))
        finally:
            runtime.poll(True)
            end=time.monotonic()+5.
            while runtime.motion and time.monotonic()<end:
                runtime.poll(True);rclpy.spin_once(node,timeout_sec=.02)
            active[0]=False;lease.publish(Bool(data=False))
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=8.)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5.)
            node.destroy_node()
            if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
