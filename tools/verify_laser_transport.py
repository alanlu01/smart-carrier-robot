"""Exercise installed laser_filters with synthetic scans in domain 231 only."""
import json, math, os, signal, subprocess, time
from pathlib import Path
assert os.environ.get('ROS_DOMAIN_ID')=='231'
assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')=='LOCALHOST'
assert os.environ.get('RMW_FASTRTPS_USE_QOS_FROM_XML')=='1'
import rclpy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster
from rclpy.qos import qos_profile_sensor_data


def main():
    rclpy.init();node=rclpy.create_node('codex_laser_transport_probe')
    received=[]
    node.create_subscription(LaserScan,'/scan_filtered',received.append,qos_profile_sensor_data)
    pub=node.create_publisher(LaserScan,'/scan',10)
    static=StaticTransformBroadcaster(node)
    tr=TransformStamped();tr.header.stamp=node.get_clock().now().to_msg()
    tr.header.frame_id='base_footprint';tr.child_frame_id='laser'
    tr.transform.translation.x=.15;tr.transform.translation.y=-.025
    tr.transform.rotation.z=1.;tr.transform.rotation.w=0.;static.sendTransform(tr)
    config=Path(__file__).resolve().parents[1]/'packages/smart_delivery_core/config/my_laser_filter.yaml'
    child=subprocess.Popen(['/opt/ros/jazzy/lib/laser_filters/scan_to_scan_filter_chain',
        '--ros-args','-r','__node:=scan_to_scan_filter_chain','--params-file',str(config)],
        stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    try:
        end=time.monotonic()+8.
        while time.monotonic()<end and pub.get_subscription_count()<1:
            rclpy.spin_once(node,timeout_sec=.02)
        assert pub.get_subscription_count()
        for _ in range(30):
            msg=LaserScan();msg.header.frame_id='laser';msg.header.stamp=node.get_clock().now().to_msg()
            msg.angle_min=-math.pi;msg.angle_max=math.pi;msg.angle_increment=2*math.pi/1800
            msg.range_min=.15;msg.range_max=12.;msg.ranges=[2.]*1800
            pub.publish(msg)
            until=time.monotonic()+.09
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.005)
        assert len(received)>=20,len(received)
        assert all(m.header.frame_id=='laser' and len(m.ranges)==1800 for m in received)
        assert all(abs(m.ranges[900]-2.)<1e-5 for m in received)
        print(json.dumps({'domain':231,'laser_filter_received':len(received),'inverted_laser_mask_test':True,
                          'motor_commands':0,'goals':0}))
    finally:
        child.send_signal(signal.SIGINT)
        try:out,err=child.communicate(timeout=5.)
        except subprocess.TimeoutExpired:
            child.terminate();out,err=child.communicate(timeout=3.)
        node.destroy_node();rclpy.shutdown()
        if child.returncode not in (0,-signal.SIGINT):
            raise RuntimeError((out+err).decode(errors='replace')[-2000:])


if __name__=='__main__':main()
