"""Exercise real MPPI plugins against an isolated software plant, never hardware."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped, Twist, PoseStamped
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import Path as NavPath, Odometry
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from smart_delivery_core.station_nav_profile import station_parameters


def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='231'
    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')=='LOCALHOST'
    assert not os.environ.get('ROS_STATIC_PEERS')
    root=Path(tempfile.mkdtemp(prefix='station_controller_',dir=os.environ['ROS_LOG_DIR']))
    share=Path(get_package_share_directory('smart_delivery_core'))
    params=station_parameters(yaml.safe_load((share/'config/my_nav2_params.yaml').read_text()))
    c=params['controller_server']['ros__parameters']
    c['general_goal_checker']['xy_goal_tolerance']=.04  # Software-plant probe only.
    costmap=params['local_costmap']['local_costmap']['ros__parameters']
    costmap['voxel_layer']['observation_sources']='scan'
    params_path=root/'controller.yaml';params_path.write_text(yaml.safe_dump(params))
    rclpy.init();node=rclpy.create_node('codex_station_controller_plant')
    state=dict(x=0.,y=0.,yaw=0.,command=Twist(),last=time.monotonic())
    samples=[];limits=[];commands=[]
    broadcaster=TransformBroadcaster(node);static=StaticTransformBroadcaster(node)
    tr=TransformStamped();tr.header.frame_id='map';tr.child_frame_id='odom'
    tr.transform.translation.x=1.;tr.transform.translation.y=2.;tr.transform.rotation.w=1.
    tr.header.stamp=node.get_clock().now().to_msg();static.sendTransform(tr)
    odom=node.create_publisher(Odometry,'/odom',10)
    laser=node.create_publisher(LaserScan,'/scan_filtered',qos_profile_sensor_data)
    def command(message):
        state['command']=message
        commands.append((state['x'],math.hypot(message.linear.x,message.linear.y)))
    node.create_subscription(Twist,'/codex/controller_command',command,10)
    node.create_subscription(String,'/delivery/station_controller_state',
        lambda m:samples.append(json.loads(m.data)),10)
    from nav2_msgs.msg import SpeedLimit
    node.create_subscription(SpeedLimit,'/speed_limit',limits.append,10)
    def sensors():
        now=time.monotonic();dt=min(.1,now-state['last']);state['last']=now
        v=state['command'];a=state['yaw']
        state['x']+=(math.cos(a)*v.linear.x-math.sin(a)*v.linear.y)*dt
        state['y']+=(math.sin(a)*v.linear.x+math.cos(a)*v.linear.y)*dt
        state['yaw']+=v.angular.z*dt
        stamp=node.get_clock().now().to_msg()
        tr=TransformStamped();tr.header.frame_id='odom';tr.child_frame_id='base_footprint';tr.header.stamp=stamp
        tr.transform.translation.x=state['x'];tr.transform.translation.y=state['y']
        tr.transform.rotation.z=math.sin(state['yaw']/2);tr.transform.rotation.w=math.cos(state['yaw']/2)
        broadcaster.sendTransform(tr)
        m=Odometry();m.header.frame_id='odom';m.child_frame_id='base_footprint';m.header.stamp=stamp
        m.pose.pose.position.x=state['x'];m.pose.pose.position.y=state['y']
        m.pose.pose.orientation=tr.transform.rotation;m.twist.twist=v;odom.publish(m)
        scan=LaserScan();scan.header.frame_id='base_footprint';scan.header.stamp=stamp
        scan.angle_min=-math.pi;scan.angle_max=math.pi;scan.angle_increment=2*math.pi/720
        scan.range_min=.15;scan.range_max=12.;scan.ranges=[float('inf')]*721;laser.publish(scan)
    node.create_timer(.025,sensors)
    def pump_until(predicate,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)
            if predicate():return True
        return bool(predicate())
    def resolve(future,seconds=8.):
        assert pump_until(future.done,seconds),'ROS future timeout'
        return future.result()
    log=(root/'controller.log').open('w')
    child=subprocess.Popen(['ros2','run','nav2_controller','controller_server','--ros-args',
        '--params-file',str(params_path),'-r','cmd_vel:=/codex/controller_command'],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    handle=None
    try:
        lifecycle=node.create_client(ChangeState,'/controller_server/change_state')
        assert lifecycle.wait_for_service(timeout_sec=10.)
        for transition in (1,3):
            request=ChangeState.Request();request.transition.id=transition
            assert resolve(lifecycle.call_async(request),15.).success
        client=ActionClient(node,FollowPath,'/follow_path')
        assert client.wait_for_server(timeout_sec=5.)
        def send(controller,goal_x,goal_y=None,yaw=0.,checker='general_goal_checker'):
            request=FollowPath.Goal();request.controller_id=controller
            request.goal_checker_id=checker
            if goal_y is None:goal_y=state['y']
            request.path=NavPath();request.path.header.frame_id='map'
            request.path.header.stamp=node.get_clock().now().to_msg()
            for i in range(41):
                p=PoseStamped();p.header=request.path.header
                p.pose.position.x=1.+state['x']+(goal_x-state['x'])*i/40
                p.pose.position.y=2.+state['y']+(goal_y-state['y'])*i/40
                p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2)
                request.path.poses.append(p)
            answer=resolve(client.send_goal_async(request));assert answer.accepted
            return answer
        handle=send('StationX',.65);result=resolve(handle.get_result_async(),30.)
        assert result.status==4,(result.status,result.result)
        assert len(samples)>=4,samples
        assert any(m['remaining_m']>.4 for m in samples),samples
        assert any(m['remaining_m']<.2 for m in samples),samples
        assert all(m['command_speed_mps']<=m['speed_cap_mps']+1e-6 for m in samples)
        assert all(m['speed_cap_mps']<=.060001 for m in samples if m['remaining_m']<=.2)
        assert any(.06001<m['speed_cap_mps']<.14999 for m in samples),samples
        handle=send('StationY',state['x'],state['y']+.55)
        assert resolve(handle.get_result_async(),30.).status==4
        assert any(m['controller']=='StationY' and m['remaining_m']<.2 for m in samples)
        assert all(m['command_speed_mps']<=m['speed_cap_mps']+1e-6 for m in samples)
        anchor=(state['x'],state['y']);desired=state['yaw']+.125
        handle=send('StationAlign',state['x'],state['y'],desired,'station_alignment_checker')
        assert resolve(handle.get_result_async(),15.).status==4
        alignment_drift=math.hypot(state['x']-anchor[0],state['y']-anchor[1])
        assert alignment_drift<=.05
        assert abs(state['yaw']-desired)<=.03
        # A cancelled station session must not leave a shared cap behind.
        handle=send('StationX',state['x']+.6)
        assert pump_until(lambda:math.hypot(state['command'].linear.x,state['command'].linear.y)>.02,5.)
        resolve(handle.cancel_goal_async());resolve(handle.get_result_async())
        commands.clear();handle=send('FollowPath',state['x']+1.)
        assert pump_until(lambda:any(speed>.20 for _,speed in commands),10.),commands[-15:]
        resolve(handle.cancel_goal_async());resolve(handle.get_result_async())
        assert not limits,'station envelope leaked to shared speed_limit'
        report=dict(passed=True,domain=231,plant_only=True,map_to_odom_offset=[1,2],
            station_reached=True,diagnostics=samples,ordinary_peak_speed=max(v for _,v in commands),
            station_y_reached=True,alignment_drift_m=alignment_drift,
            ordinary_after_station_cancel=True,shared_limits_published=len(limits))
        (root/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
    finally:
        if handle is not None:
            try:resolve(handle.cancel_goal_async(),2.)
            except Exception:pass
        child.send_signal(signal.SIGINT)
        try:child.wait(timeout=8.)
        except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=3.)
        log.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
