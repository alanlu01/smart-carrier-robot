"""Isolated DDS smoke test: synthetic odometry only, never motor/goal topics.

Run in domain 231 / LOCALHOST with fastdds_sensor_publish.xml configured.
"""
import json
import os
import time
assert os.environ.get('ROS_DOMAIN_ID')=='231'
assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')=='LOCALHOST'
assert os.environ.get('RMW_FASTRTPS_USE_QOS_FROM_XML')=='1'
import rclpy
from rclpy.executors import SingleThreadedExecutor
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from smart_delivery_core.odom_transport import OdomTransport, LOCAL_ODOM_QOS


def main():
    rclpy.init()
    probe=rclpy.create_node('codex_sensor_transport_probe')
    relay=OdomTransport()
    exe=SingleThreadedExecutor();exe.add_node(probe);exe.add_node(relay)
    output=[];transforms=[]
    publisher=probe.create_publisher(Odometry,'/chassis/odom_local',LOCAL_ODOM_QOS)
    probe.create_subscription(Odometry,'/odom',output.append,10)
    probe.create_subscription(TFMessage,'/tf',transforms.append,100)
    try:
        end=time.monotonic()+3.
        while time.monotonic()<end and (publisher.get_subscription_count()<1
                or relay.odom_pub.get_subscription_count()<1):
            exe.spin_once(timeout_sec=.05)
        assert publisher.get_subscription_count() and relay.odom_pub.get_subscription_count()
        for i in range(20):
            msg=Odometry();msg.header.stamp=probe.get_clock().now().to_msg()
            msg.header.frame_id='odom';msg.child_frame_id='base_footprint'
            msg.pose.pose.orientation.w=1.;msg.pose.pose.position.x=i*.001
            publisher.publish(msg)
            until=time.monotonic()+.05
            while time.monotonic()<until:exe.spin_once(timeout_sec=.005)
        assert len(output)>=15 and transforms
        assert all(x.header.frame_id=='odom' and x.child_frame_id=='base_footprint' for x in output)
        assert all(x.transforms[0].header.stamp.sec>0 for x in transforms)
        assert relay.errors==0
        print(json.dumps({'domain':231,'relay_messages':len(output),'tf_messages':len(transforms),
            'write_errors':relay.errors,'public_odom_reliability':str(relay.odom_pub.qos_profile.reliability),
            'motor_commands':0,'goals':0}))
    finally:
        exe.shutdown();probe.destroy_node();relay.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
