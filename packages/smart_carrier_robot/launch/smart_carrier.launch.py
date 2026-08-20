import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("smart_carrier_robot"), "config", "robot.yaml"
    )
    return LaunchDescription(
        [
            Node(package="smart_carrier_robot", executable="ina3221_node", parameters=[config]),
            Node(
                package="smart_carrier_robot",
                executable="dispatch_bridge_node",
                parameters=[config],
            ),
            Node(package="smart_carrier_robot", executable="navigator_node"),
        ]
    )
