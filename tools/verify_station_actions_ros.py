"""Native action/executor/lease integration with virtual poses, never velocity."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.duration import Duration
from rcl_interfaces.msg import Log
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from smart_delivery_core.delivery_goal_guard import DeliveryGoalGuard, LEASE_QOS
from smart_delivery_core.delivery_navigator import DeliveryNavigator
from smart_delivery_core.station_navigation import StationNavigation, coordinates
from smart_delivery_core.delivery_inputs import shared_inputs, message_json
from smart_delivery_core.smart_delivery import go_to_standby
from ament_index_python.packages import get_package_share_directory


def fixture():
    from rclpy.action import ActionServer, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init();node=rclpy.create_node('codex_station_action_fixture');guard=DeliveryGoalGuard()
    loaded='--loaded' in sys.argv
    state=dict(pose=(0.,0.,0.),hold=False,grid_enabled=True,goals=[],cancels=0)
    node.create_subscription(Bool,'/codex/station_hold',lambda m:state.update(hold=m.data),10)
    node.create_subscription(Bool,'/codex/station_grid_enabled',lambda m:state.update(grid_enabled=m.data),10)
    state_qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    grid=node.create_publisher(OccupancyGrid,'/global_costmap/costmap',state_qos)
    map_message=OccupancyGrid();map_message.header.frame_id='map'
    map_message.info.resolution=.05;map_message.info.width=map_message.info.height=100
    map_message.info.origin.position.x=map_message.info.origin.position.y=-2.5
    map_message.info.origin.orientation.w=1.;map_message.data=[0]*10000
    if loaded:
        import numpy as np
        import yaml
        from PIL import Image
        share=Path(get_package_share_directory('smart_delivery_core'))
        path=share/'maps/3F_NEW.yaml';meta=yaml.safe_load(path.read_text())
        pixels=np.flipud(np.asarray(Image.open(path.parent/meta['image']).convert('L')))
        occupancy=1.-pixels/255.
        map_message.info.width,map_message.info.height=pixels.shape[1],pixels.shape[0]
        map_message.info.resolution=meta['resolution']
        map_message.info.origin.position.x,map_message.info.origin.position.y=meta['origin'][:2]
        map_message.data=np.where(occupancy>meta['occupied_thresh'],100,
            np.where(occupancy<meta['free_thresh'],0,-1)).ravel().tolist()
        static_map=node.create_publisher(OccupancyGrid,'/map',state_qos)
        map_message.header.stamp=node.get_clock().now().to_msg();static_map.publish(map_message)
    odom=node.create_publisher(Odometry,'/odom',10);tf=TransformBroadcaster(node)
    def publish():
        stamp=node.get_clock().now().to_msg();x,y,yaw=state['pose']
        tr=TransformStamped();tr.header.frame_id='odom' if loaded else 'map';tr.child_frame_id='base_footprint';tr.header.stamp=stamp
        tr.transform.translation.x=x;tr.transform.translation.y=y
        tr.transform.rotation.z=math.sin(yaw/2);tr.transform.rotation.w=math.cos(yaw/2);tf.sendTransform(tr)
        message=Odometry();message.header.stamp=stamp;message.pose.pose.orientation.w=1.;odom.publish(message)
    def publish_grid():
        if not state['grid_enabled']:return
        map_message.header.stamp=node.get_clock().now().to_msg();grid.publish(map_message)
    node.create_timer(.05,publish);node.create_timer(2. if loaded else .5,publish_grid)
    if loaded:
        def map_transform():
            tr=TransformStamped();tr.header.frame_id='map';tr.child_frame_id='odom'
            tr.header.stamp=(node.get_clock().now()+Duration(seconds=1.)).to_msg()
            tr.transform.rotation.w=1.;tf.sendTransform(tr)
        node.create_timer(1./11.,map_transform)
        # Match the production callback load instead of just three easy inputs.
        scan=node.create_publisher(LaserScan,'/scan_filtered',qos_profile_sensor_data)
        def laser():
            m=LaserScan();m.header.frame_id='base_footprint';m.header.stamp=node.get_clock().now().to_msg()
            m.angle_min=-math.pi;m.angle_increment=2*math.pi/720;m.range_min=.15;m.range_max=12.
            m.ranges=[2.]*720;scan.publish(m)
        node.create_timer(1./11.,laser)
        vision=node.create_publisher(String,'/vision/semantic_info',10)
        def semantics():
            stamp=node.get_clock().now().to_msg()
            vision.publish(String(data=json.dumps(dict(stamp=dict(sec=stamp.sec,nanosec=stamp.nanosec),detections=[],padding='x'*1024))))
        node.create_timer(1./16.,semantics)
        for topic,hz,qos,payload in (
            ('/power_status',2.,10,dict(ch1=dict(status='full'),ch2=dict(status='full'),ch3=dict(status='empty'))),
            ('/chassis/serial_rx_health',6.,qos_profile_sensor_data,dict(healthy=True,stop_proven=True)),
            ('/localization/state',1.,state_qos,dict(state='LOCALIZED',ready=True)),
            ('/smart_carrier/dispatch_sync_state',1.,state_qos,dict(state='ready')),
        ):
            publisher=node.create_publisher(String,topic,qos)
            node.create_timer(1./hz,lambda p=publisher,v=payload:p.publish(String(data=json.dumps(v))))
        ready=node.create_publisher(Bool,'/localization/ready',state_qos)
        inhibited=node.create_publisher(Bool,'/localization/motion_inhibited',state_qos)
        node.create_timer(1.,lambda:ready.publish(Bool(data=True)))
        node.create_timer(1.,lambda:inhibited.publish(Bool(data=False)))
        amcl=node.create_publisher(PoseWithCovarianceStamped,'/amcl_pose',state_qos)
        def pose():
            m=PoseWithCovarianceStamped();m.header.frame_id='map';m.header.stamp=node.get_clock().now().to_msg()
            m.pose.pose.position.x,m.pose.pose.position.y=state['pose'][:2]
            m.pose.pose.orientation.z=math.sin(state['pose'][2]/2);m.pose.pose.orientation.w=math.cos(state['pose'][2]/2)
            amcl.publish(m)
        node.create_timer(1.,pose)
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
    publish_grid()
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
    loaded='--loaded' in sys.argv
    legacy='--legacy-wait' in sys.argv
    manager_type=StationNavigation
    if legacy:
        import types
        source=subprocess.check_output(['git','show',
            'a077a52:packages/smart_delivery_core/smart_delivery_core/station_navigation.py'],text=True)
        module=types.ModuleType('legacy_station_navigation')
        exec(compile(source,'legacy_station_navigation.py','exec'),module.__dict__)
        manager_type=module.StationNavigation
    root=Path(os.environ['ROS_LOG_DIR']);root.mkdir(parents=True,exist_ok=True)
    with (root/'action_fixture.log').open('w') as log:
        child=subprocess.Popen([sys.executable,__file__,'--fixture']+(['--loaded'] if loaded else []),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        rclpy.init();node=DeliveryNavigator();buffer=Buffer();listener=TransformListener(buffer,node)
        share=get_package_share_directory('smart_delivery_core')
        manager=manager_type(node,buffer,share);node.station_navigation=manager
        received={};optional=None
        if loaded:
            from smart_delivery_core.optional_idle import load_idle_config
            from smart_delivery_core.optional_idle_runtime import OptionalIdleRuntime
            def consume(topic,message):
                received[topic]=received.get(topic,0)+1
                if isinstance(message,String):message_json(message)
            shared=shared_inputs(node)
            shared.subscribe(Odometry,'/odom',lambda m:consume('/odom',m),10)
            shared.subscribe(LaserScan,'/scan_filtered',lambda m:consume('/scan_filtered',m),qos_profile_sensor_data)
            for topic,qos in (('/chassis/serial_rx_health',qos_profile_sensor_data),
                    ('/smart_carrier/dispatch_sync_state',LEASE_QOS),('/power_status',10),
                    ('/order',10),('/smart_carrier/task_cancel',10),('/smart_carrier/task_result_ack',LEASE_QOS)):
                node.create_subscription(String,topic,lambda m,t=topic:consume(t,m),qos)
            shared.subscribe(String,'/localization/state',lambda m:consume('/localization/state',m),LEASE_QOS)
            node.create_subscription(Log,'/rosout',lambda m:consume('/rosout',m),50)
            node.create_subscription(Bool,'/localization/ready',lambda m:consume('/localization/ready',m),LEASE_QOS)
            optional=OptionalIdleRuntime(node,buffer,load_idle_config(Path(share)/'config/optional_idle_features.yaml'),
                share,lambda:True,lambda _active:None,lambda:None)
        lease=node.create_publisher(Bool,'/smart_carrier/delivery_navigation_active',LEASE_QOS)
        node.create_timer(.2,lambda:lease.publish(Bool(data=True)))
        hold=node.create_publisher(Bool,'/codex/station_hold',10)
        grid_enabled=node.create_publisher(Bool,'/codex/station_grid_enabled',10)
        commands=[];node.create_subscription(Twist,'/cmd_vel',commands.append,10)
        def pump(seconds):
            end=time.monotonic()+seconds
            while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        def finish(limit=15.):
            end=time.monotonic()+limit
            while time.monotonic()<end:
                if manager.isTaskComplete():return manager.getResult()
                # Also cover the former production caller's fixed sleep; the
                # callback pump must keep up even if a legacy caller retains it.
                if loaded:time.sleep(.1)
                rclpy.spin_once(node,timeout_sec=.02)
            raise AssertionError('station action did not complete')
        def goal(x,y,yaw=0.):
            message=PoseStamped();message.header.frame_id='map'
            message.pose.position.x=x;message.pose.position.y=y
            message.pose.orientation.z=math.sin(yaw/2);message.pose.orientation.w=math.cos(yaw/2);return message
        try:
            assert node.nav_to_pose_client.wait_for_server(timeout_sec=10.)
            pump(1.)
            first=(-3.,5.,0.) if loaded else (1.,0.,0.)
            second=(6.6,-.3,math.pi) if loaded else (1.,1.,0.)
            manager.start(goal(*first));first_result=finish()
            if legacy:
                print(json.dumps(dict(legacy_wait=True,loaded=loaded,result=str(first_result),phase=manager.phase,
                    reason=manager.last_report[1],
                    reproduced_wait_failure=first_result==TaskResult.FAILED and manager.stage is None
                        and '階段逾時' in manager.last_report[1],
                    nonzero_velocity_commands=0,received=received)))
                return
            assert first_result==TaskResult.SUCCEEDED
            assert manager.parked==first
            manager.start(goal(*second));assert finish()==TaskResult.SUCCEEDED
            assert manager.parked==second
            if loaded:
                point=dict(name='seat_light',x=1.4,y=-5.3,yaw=0.)
                position=go_to_standby(node,dict(x=second[0],y=second[1]),standby_point=point)
                assert position==dict(x=point['x'],y=point['y'])
            hold.publish(Bool(data=True));pump(.3)
            permit=[True];manager.start(goal(0.,0.),lambda:permit[0])
            end=time.monotonic()+3.
            while not manager.active_goal and time.monotonic()<end:
                manager.isTaskComplete();rclpy.spin_once(node,timeout_sec=.02)
            assert manager.active_goal
            permit[0]=False;assert finish()==TaskResult.CANCELED
            if loaded:
                # A real source outage, not a mocked healthy predicate. Test a
                # short deadline here; production retains its 30-second cap.
                hold.publish(Bool(data=False));grid_enabled.publish(Bool(data=False));pump(3.)
                assert not manager._data()
                manager.preflight_wait_sec=1.5
                manager.start(goal(0.,0.));assert finish(5.)==TaskResult.FAILED
                assert manager.preflight_timeout and manager.goals_submitted==0
                assert 'costmap_stale' in manager.wait_blockers
                manager.preflight_wait_sec=30.
                manager.start(goal(0.,0.),lambda:False)
                assert finish()==TaskResult.CANCELED and manager.goals_submitted==0
                grid_enabled.publish(Bool(data=True));pump(2.2)
                manager.start(goal(1.4,-5.3))
                assert finish()==TaskResult.SUCCEEDED and not manager.preflight_timeout
            assert not commands, 'isolated action test must never publish velocity'
            print(json.dumps(dict(passed=True,park_x=True,departure_and_park_y=True,
                permission_cancel_drained=True,nonzero_velocity_commands=0,domain=231,
                production_callback_load=loaded,map_shape=[827,1070] if loaded else [100,100],
                bounded_missing_data_and_restart_verified=loaded,
                real_standby_caller_verified=loaded,received=received)))
        finally:
            if manager.running:
                manager.cancelTask();finish(5.)
            lease.publish(Bool(data=False));pump(.2)
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=8.)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5.)
            node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
