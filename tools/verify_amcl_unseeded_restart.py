"""Real AMCL reconfigure/bootstrap in localhost domain; synthetic laser, no motion."""
import json,math,os,signal,subprocess,time
from pathlib import Path
import rclpy
from geometry_msgs.msg import TransformStamped,PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from lifecycle_msgs.srv import GetState,ChangeState
from std_srvs.srv import Empty
from tf2_ros import StaticTransformBroadcaster
from ament_index_python.packages import get_package_share_directory


def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='231'
    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')=='LOCALHOST'
    assert not os.environ.get('ROS_STATIC_PEERS')
    share=Path(get_package_share_directory('smart_delivery_core'))
    rclpy.init();node=rclpy.create_node('codex_amcl_bootstrap_probe');poses=[]
    node.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',poses.append,10)
    broadcaster=StaticTransformBroadcaster(node);tr=TransformStamped()
    tr.header.frame_id='odom';tr.child_frame_id='base_footprint';tr.transform.rotation.w=1.
    tr.header.stamp=node.get_clock().now().to_msg();broadcaster.sendTransform(tr)
    scan=node.create_publisher(LaserScan,'/scan_filtered',10)
    def pump(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            m=LaserScan();m.header.stamp=node.get_clock().now().to_msg();m.header.frame_id='base_footprint'
            m.angle_min=-math.pi;m.angle_increment=2*math.pi/120;m.angle_max=m.angle_min+119*m.angle_increment
            m.range_min=.1;m.range_max=12.;m.ranges=[5.]*120;scan.publish(m)
            rclpy.spin_once(node,timeout_sec=.05)
    def call(client,request):
        assert client.wait_for_service(timeout_sec=10.)
        future=client.call_async(request);deadline=time.monotonic()+5.
        while not future.done() and time.monotonic()<deadline:pump(.05)
        assert future.done() and future.exception() is None
        return future.result()
    log=Path(os.environ['ROS_LOG_DIR'])/'amcl_bootstrap.log'
    with log.open('w') as output:
        child=subprocess.Popen(['ros2','launch','nav2_bringup','localization_launch.py',
            'map:='+str(share/'maps/3F_NEW.yaml'),'params_file:='+str(share/'config/my_nav2_params.yaml'),
            'use_composition:=False','autostart:=true'],stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            get=node.create_client(GetState,'/amcl/get_state')
            deadline=time.monotonic()+20.
            while time.monotonic()<deadline:
                if call(get,GetState.Request()).current_state.id==3:break
                pump(.2)
            assert call(get,GetState.Request()).current_state.id==3
            change=node.create_client(ChangeState,'/amcl/change_state')
            for transition in (4,2,1,3):
                req=ChangeState.Request();req.transition.id=transition
                assert call(change,req).success
            poses.clear();pump(1.)
            assert not poses,'Test must start without an initial pose'
            initialize=node.create_client(Empty,'/reinitialize_global_localization')
            call(initialize,Empty.Request());pump(2.)
            assert poses,'Global initialization must restore pose publication without old pose seed'
            print(json.dumps(dict(real_amcl_reconfigured=True,no_old_pose_seed=True,
                                  published_poses_after_global_init=len(poses),hardware_commands=0)))
        finally:
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(10.)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
            node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
