import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start RViz only; the robot computer owns robot_description and TF."""
    package_path = get_package_share_directory("vm_robot_model")
    default_rviz_config = os.path.join(package_path, "rviz", "nav_test.rviz")

    rviz_config_argument = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config,
        description="Absolute path to the RViz configuration file",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": False}],
    )

    return LaunchDescription([
        rviz_config_argument,
        rviz_node,
    ])
