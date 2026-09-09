import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """Start Nav2 together with the robot-side localization safety manager."""
    core_share = get_package_share_directory("smart_delivery_core")
    nav2_share = get_package_share_directory("nav2_bringup")
    default_map = os.path.join(core_share, "maps", "3f_platform.yaml")
    default_params = os.path.join(core_share, "config", "my_nav2_params.yaml")
    default_behavior_tree = os.path.join(
        core_share, "behavior_trees", "smart_delivery_navigate.xml"
    )

    map_argument = DeclareLaunchArgument(
        "map", default_value=default_map, description="Absolute path to the map yaml"
    )
    params_argument = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Absolute path to Nav2 and localization manager parameters",
    )
    behavior_tree_argument = DeclareLaunchArgument(
        "behavior_tree",
        default_value=default_behavior_tree,
        description="Absolute path to the NavigateToPose behavior tree XML",
    )
    use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation clock"
    )
    auto_initialize_argument = DeclareLaunchArgument(
        "auto_initialize",
        default_value="true",
        description="Seed AMCL from the fixed (0,0,0) power-on pose",
    )

    configured_nav2_params = RewrittenYaml(
        source_file=LaunchConfiguration("params_file"),
        param_rewrites={
            "default_nav_to_pose_bt_xml": LaunchConfiguration("behavior_tree")
        },
        convert_types=True,
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "params_file": configured_nav2_params,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "autostart": "true",
        }.items(),
    )

    localization_manager = Node(
        package="smart_delivery_core",
        executable="localization_manager",
        name="localization_manager",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "auto_initialize": ParameterValue(
                    LaunchConfiguration("auto_initialize"), value_type=bool
                )
            },
        ],
    )

    return LaunchDescription(
        [
            map_argument,
            params_argument,
            behavior_tree_argument,
            use_sim_time_argument,
            auto_initialize_argument,
            nav2,
            localization_manager,
        ]
    )
