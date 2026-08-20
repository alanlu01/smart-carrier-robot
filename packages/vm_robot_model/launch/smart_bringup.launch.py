import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    core_package_path = get_package_share_directory("smart_delivery_core")
    model_package_path = get_package_share_directory("vm_robot_model")

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(core_package_path, "launch", "my_lidar.launch.py")
        )
    )

    # The robot computer owns robot_description and TF. RViz runs separately
    # on the VMware workstation through display.launch.py.
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(model_package_path, "launch", "description.launch.py")
        )
    )

    cmd_to_vel_node = Node(
        package="smart_delivery_core",
        executable="cmd_vel_to_serial",
        name="cmd_vel_to_serial",
        arguments=["ROS"],
        output="screen",
    )

    stm_to_ros_node = Node(
        package="smart_delivery_core",
        executable="serial_to_ros",
        name="serial_to_ros",
        arguments=["STM32"],
        output="screen",
    )

    odom_node = Node(
        package="smart_delivery_core",
        executable="mecanum_odom_real",
        name="mecanum_odom_real",
        output="screen",
    )

    return LaunchDescription([
        lidar_launch,
        description_launch,
        cmd_to_vel_node,
        stm_to_ros_node,
        odom_node,
    ])
