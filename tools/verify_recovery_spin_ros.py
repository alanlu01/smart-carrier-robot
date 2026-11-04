"""Real ROS action cancellation with simulated sensors; no hardware or Nav2 goals."""
import json
import math
import os
import tempfile
import time
from types import SimpleNamespace
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import Spin
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from smart_delivery_core.localization_manager import LocalizationManager


def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='231'
    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')=='LOCALHOST'
    assert not os.environ.get('ROS_STATIC_PEERS')
    with tempfile.TemporaryDirectory(prefix='codex_fake_spin_') as directory:
        rclpy.init(args=['--ros-args','-p','auto_initialize:=false',
                        '-p','automatic_motion_recovery:=false','-p','pose_cache_path:='+directory+'/pose.json'])
        manager=LocalizationManager()
        probe=rclpy.create_node('codex_simulated_spin_server')
        executor=MultiThreadedExecutor(num_threads=3)
        executor.add_node(manager);executor.add_node(probe)
        model=dict(yaw=0.,rate=0.,busy=False,goals=[],cancels=0,blocked=False,closing=False)
        completed=[]
        manager._recovery_spin_clear=lambda _:True  # geometry is tested separately
        manager._complete_spin_sequence=lambda:completed.append(manager.last_recovery_spin)
        manager.recovery_feedback=SimpleNamespace(last_complete_at=0.,is_stale=lambda *_:False)
        manager.recovery_stop=SimpleNamespace(ready=lambda _:not model['busy'])

        def execute(handle):
            target=abs(handle.request.target_yaw)
            model['goals'].append(target)
            start=model['yaw'];previous=time.monotonic();model['busy']=True
            while abs(model['yaw']-start)<target:
                if model['closing']:
                    handle.abort();return Spin.Result()
                if handle.is_cancel_requested:
                    model['busy']=False;model['rate']=0.;model['cancels']+=1
                    handle.canceled();return Spin.Result()
                now=time.monotonic()
                cap=manager.managed_spin.speed_limit() if manager.managed_spin else 0.
                model['rate']=min(1.,cap)
                model['yaw']+=model['rate']*(now-previous);previous=now
                time.sleep(.01)
            model['busy']=False;model['rate']=0.
            handle.succeed();return Spin.Result()

        server=ActionServer(probe,Spin,'/spin',execute,
            cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=ReentrantCallbackGroup())
        def sensors():
            now=time.monotonic();stamp=manager.get_clock().now().to_msg()
            odom=Odometry();odom.header.stamp=stamp
            odom.pose.pose.orientation.z=math.sin(model['yaw']/2)
            odom.pose.pose.orientation.w=math.cos(model['yaw']/2)
            odom.twist.twist.angular.z=model['rate']
            manager._odom_callback(odom)
            command=Twist();command.angular.z=model['rate'];manager._cmd_vel_callback(command)
            scan=LaserScan();scan.header.stamp=stamp
            manager.latest_scan=manager.latest_raw_scan=scan
            manager.latest_scan_at=manager.latest_raw_scan_at=now
            manager.latest_scan_stamp_ns=stamp.sec*1_000_000_000+stamp.nanosec
            manager.navigation_health={'state':'ready'};manager.navigation_health_at=now
            manager.recovery_feedback.last_complete_at=now
            if not model['blocked'] and model['yaw']>3.:
                model['blocked']=True
                message=CollisionMonitorState();message.action_type=1;message.polygon_name='NearFootprintStop'
                manager._spin_collision_callback(message)
        timer=manager.create_timer(.02,sensors)
        try:
            deadline=time.monotonic()+5.
            while not manager.spin_client.server_is_ready() and time.monotonic()<deadline:
                executor.spin_once(timeout_sec=.02)
            assert manager.spin_client.server_is_ready()
            sensors()
            manager.state='RECOVERING_GLOBAL';manager.ready=False;manager.restore_heading=False
            manager.spin_sequence=[2*math.pi]
            manager._send_next_spin()
            deadline=time.monotonic()+25.
            while not completed and time.monotonic()<deadline:
                executor.spin_once(timeout_sec=.02)
            assert completed, (model,manager.state,manager.state_reason)
            assert len(model['goals'])==2 and 0 < model['goals'][1]<2*math.pi-2.
            assert completed[0]['resumes']==1 and completed[0]['remaining_deg']<=3.
            assert model['cancels']>=1 and not manager.ready
            # Drain terminal result/cancel replies before destroying action entities.
            drained=time.monotonic()+1.
            while time.monotonic()<drained:executor.spin_once(timeout_sec=.02)
            print(json.dumps(dict(real_ros_action=True,goals_deg=[math.degrees(x) for x in model['goals']],
                cancellations=model['cancels'],final=completed[0],hardware_commands=0)))
        finally:
            model['closing']=True
            for entity in manager.timers:entity.cancel()
            # This pinned rclpy version inherits the base shutdown without
            # joining its ThreadPoolExecutor. Join before destroying ROS handles.
            executor._executor.shutdown(wait=True)
            for future in executor._futures:
                if future.done():future.result()
            executor.shutdown(timeout_sec=3.)
            manager.destroy_timer(timer)
            server.destroy();manager.pose_cache_writer.close()
            manager.destroy_node();probe.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
