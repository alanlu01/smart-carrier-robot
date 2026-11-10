"""Exercise the installed CollisionMonitor with reversed virtual lidar; no hardware."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
import yaml
from geometry_msgs.msg import Point32, PolygonStamped, TransformStamped, Twist
from lifecycle_msgs.srv import ChangeState
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from ament_index_python.packages import get_package_share_directory
from smart_delivery_core.station_nav_profile import station_parameters


def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='231'
    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')=='LOCALHOST'
    assert not os.environ.get('ROS_STATIC_PEERS')
    root=Path(os.environ['ROS_LOG_DIR']);root.mkdir(parents=True,exist_ok=True)
    share=Path(get_package_share_directory('smart_delivery_core'))
    parameters=station_parameters(yaml.safe_load((share/'config/my_nav2_params.yaml').read_text()))
    cfg=root/'collision_parameters.yaml';cfg.write_text(yaml.safe_dump(parameters))
    rclpy.init();node=rclpy.create_node('codex_station_collision_probe')
    outputs=[]
    node.create_subscription(Twist,'/cmd_vel',lambda m:outputs.append((time.monotonic(),m)),10)
    scans=node.create_publisher(LaserScan,'/scan_filtered',qos_profile_sensor_data)
    commands=node.create_publisher(Twist,'/cmd_vel_smoothed',10)
    footprints=node.create_publisher(PolygonStamped,'/local_costmap/published_footprint',10)
    broadcaster=StaticTransformBroadcaster(node);transforms=[]
    for parent,child,x,y,yaw in [('odom','base_footprint',0.,0.,0.),('base_footprint','laser',.15,-.025,math.pi)]:
        tr=TransformStamped();tr.header.frame_id=parent;tr.child_frame_id=child
        tr.header.stamp=node.get_clock().now().to_msg()
        tr.transform.translation.x=x;tr.transform.translation.y=y
        tr.transform.rotation.z=math.sin(yaw/2);tr.transform.rotation.w=math.cos(yaw/2)
        transforms.append(tr)
    broadcaster.sendTransform(transforms)
    client=node.create_client(ChangeState,'/collision_monitor/change_state')
    results={}
    with (root/'collision.log').open('w') as log:
        child=subprocess.Popen(['ros2','run','nav2_collision_monitor','collision_monitor',
            '--ros-args','--params-file',str(cfg)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            if not client.wait_for_service(timeout_sec=12.):raise RuntimeError('collision lifecycle missing')
            for transition in (1,3):
                request=ChangeState.Request();request.transition.id=transition
                future=client.call_async(request)
                rclpy.spin_until_future_complete(node,future,timeout_sec=10.)
                assert future.done() and future.result().success, 'lifecycle failed'
            def exercise(name,point,velocity,allowed,duration=1.2,reset=True):
                # CollisionMonitor intentionally stops repeating zero after
                # stop_pub_timeout. Separate cases with a verified clear run
                # so a long prior STOP cannot be mistaken for missing output.
                if reset:exercise('_clear',None,(.05,0.,0.),True,.6,False)
                message=Twist();message.linear.x,message.linear.y,message.angular.z=velocity
                ranges=[float('inf')]*1440
                # Use the real reversed lidar mounting, not a convenient
                # identity laser transform that could conceal an axis error.
                if point:
                    x,y=point
                    lx,ly=-(x-.15),-(y+.025)
                    angle=math.atan2(ly,lx);distance=math.hypot(lx,ly)
                    center=int(round((angle+math.pi)*1440/(2*math.pi)))%1440
                    for i in (-1,0,1):ranges[(center+i)%1440]=distance
                outputs.clear();end=time.monotonic()+duration
                while time.monotonic()<end:
                    scan=LaserScan();scan.header.frame_id='laser';scan.header.stamp=node.get_clock().now().to_msg()
                    scan.angle_min=-math.pi;scan.angle_max=math.pi-2*math.pi/1440
                    scan.angle_increment=2*math.pi/1440;scan.range_min=.02;scan.range_max=12.;scan.ranges=ranges
                    footprint=PolygonStamped();footprint.header.frame_id='base_footprint';footprint.header.stamp=scan.header.stamp
                    footprint.polygon.points=[Point32(x=x,y=y) for x,y in ((.22,.16),(.22,-.16),(-.22,-.16),(-.22,.16))]
                    footprints.publish(footprint)
                    scans.publish(scan);commands.publish(message)
                    rclpy.spin_once(node,timeout_sec=.04)
                    time.sleep(.04)
                recent=[m for stamp,m in outputs if stamp>end-.45]
                assert recent, name+': no monitored command'
                magnitudes=[math.hypot(m.linear.x,m.linear.y)+abs(m.angular.z) for m in recent]
                if allowed:assert min(magnitudes)>.01, (name,magnitudes)
                else:assert max(magnitudes)<1e-6, (name,magnitudes)
                if name!='_clear':results[name]=dict(samples=len(recent),allowed=allowed)
            exercise('left_wall_move_right',(0.,.235),(0.,-.15,0.),True)
            exercise('left_wall_move_left',(0.,.235),(0.,.15,0.),False)
            exercise('right_wall_move_right',(0.,-.235),(0.,-.15,0.),False)
            exercise('front_wall_reverse',(.29,0.),(-.15,0.,0.),True)
            exercise('front_wall_forward',(.29,0.),(.15,0.,0.),False)
            exercise('corner_rotation',(.25,.17),(0.,0.,.5),False)
            exercise('clear_rotation',None,(0.,0.,.5),True)
            print(json.dumps(dict(passed=True,cases=results,reversed_lidar=True,domain=231),indent=2))
        finally:
            commands.publish(Twist())
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=8.)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5.)
            node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
