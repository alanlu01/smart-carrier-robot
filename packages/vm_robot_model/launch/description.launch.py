import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Publish the robot description and all fixed robot transforms."""
    package_path = get_package_share_directory("vm_robot_model")
    urdf_file_path = os.path.join(package_path, "urdf", "smart_carrier.urdf")

    with open(urdf_file_path, "r", encoding="utf-8") as infp:
        robot_description = infp.read()

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": False,
            }],
        )
    ])
