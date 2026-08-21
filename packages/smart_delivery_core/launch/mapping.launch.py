import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Run SLAM on the robot; RViz is intentionally hosted on the VMware PC."""
    core_share = get_package_share_directory("smart_delivery_core")
    slam_share = get_package_share_directory("slam_toolbox")

    default_params = os.path.join(core_share, "config", "slam_mapping.yaml")
    params_argument = DeclareLaunchArgument(
        "slam_params_file",
        default_value=default_params,
        description="Absolute path to the slam_toolbox mapping parameters",
    )
    use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use wall time on the physical robot",
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "slam_params_file": LaunchConfiguration("slam_params_file"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "autostart": "true",
        }.items(),
    )

    return LaunchDescription([
        params_argument,
        use_sim_time_argument,
        slam_toolbox,
    ])
