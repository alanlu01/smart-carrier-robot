"""Localhost/domain 231: actual DDS zero-limit path, no goals or nonzero commands."""
import os
import time
import json
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from hailo_vision.fusion_node import SensorFusionNode


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '231'
    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') == 'LOCALHOST'
    assert not os.environ.get('ROS_STATIC_PEERS')
    rclpy.init()
    fusion=SensorFusionNode()
    probe=rclpy.create_node('codex_zero_localization_limit_probe')
    executor=SingleThreadedExecutor()
    executor.add_node(fusion);executor.add_node(probe)
    observed=[]
    probe.create_subscription(Twist,'/chassis_cmd_vel',observed.append,10)
    publisher=probe.create_publisher(Float32,'/localization/speed_limit',1)
    try:
        end=time.monotonic()+3.
        while publisher.get_subscription_count()<1 and time.monotonic()<end:
            executor.spin_once(timeout_sec=.02)
        assert publisher.get_subscription_count()>=1
        until=time.monotonic()+.5
        while time.monotonic()<until:
            publisher.publish(Float32(data=0.))
            step=time.monotonic()+.05
            while time.monotonic()<step:
                executor.spin_once(timeout_sec=.005)
        assert fusion.localization_speed_limit.value==0. and observed
        assert all(all(value==0. for value in (m.linear.x,m.linear.y,m.linear.z,
                       m.angular.x,m.angular.y,m.angular.z)) for m in observed)
        print(json.dumps(dict(domain=231,dds_zero_limit_received=True,
                              zero_motor_messages=len(observed),goals=0,nonzero_motor_commands=0)))
    finally:
        executor.shutdown();fusion.destroy_node();probe.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
