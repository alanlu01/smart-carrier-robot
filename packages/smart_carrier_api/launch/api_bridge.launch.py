import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("smart_carrier_api"), "config", "api.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="smart_carrier_api",
                executable="api_bridge",
                name="smart_carrier_api_bridge",
                parameters=[config],
                output="screen",
            )
        ]
    )
