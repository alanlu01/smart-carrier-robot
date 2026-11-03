import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, GroupAction, SetEnvironmentVariable
from launch.event_handlers import OnShutdown, OnProcessIO, OnProcessExit
from launch.logging import launch_config
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml
from smart_delivery_core.optional_nav_profile import OptionalNavParameters
from smart_delivery_core.navigation_launch_guard import NavigationLaunchLease
from smart_delivery_core.navigation_transcript import NavigationTranscript


def generate_launch_description():
    """Start Nav2 together with the robot-side localization safety manager."""
    core_share = get_package_share_directory("smart_delivery_core")
    nav2_share = get_package_share_directory("nav2_bringup")
    default_map = os.path.join(core_share, "maps", "3F_NEW.yaml")
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
    optional_idle_argument = DeclareLaunchArgument(
        "optional_idle_config",
        default_value=os.path.join(core_share, "config", "optional_idle_features.yaml"),
        description="Idle feature policy (home trigger disabled; crowd selection enabled)",
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
        source_file=OptionalNavParameters(),
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
            'use_composition': PythonExpression(["'",LaunchConfiguration('use_composition'),"'.lower() == 'true'"]),
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
                "map_yaml_file": LaunchConfiguration("map"),
                "auto_initialize": ParameterValue(
                    LaunchConfiguration("auto_initialize"), value_type=bool
                )
            },
        ],
    )

    delivery_goal_guard = Node(
        package="smart_delivery_core",
        executable="delivery_goal_guard",
        name="delivery_goal_guard",
        output="screen",
    )

    def guarded_start(context):
        mode = LaunchConfiguration('nav_transport_profile').perform(context)
        if mode not in ('current','async'):
            raise ValueError('nav_transport_profile must be current or async')
        if LaunchConfiguration('use_composition').perform(context).lower() not in ('true','false'):
            raise ValueError('use_composition must be true or false')
        lease = NavigationLaunchLease().acquire()
        transcript = NavigationTranscript(launch_config.log_dir, multi_process=True)
        def release(context):
            lease.close()
            return []
        nav_actions = [nav2]
        if mode == 'async':
            profile = os.path.join(core_share,'config','fastdds_nav_async.xml')
            nav_actions = [SetEnvironmentVariable(name=name,value=value) for name,value in (
                ('FASTRTPS_DEFAULT_PROFILES_FILE',profile),('FASTDDS_DEFAULT_PROFILES_FILE',profile),
                ('RMW_FASTRTPS_USE_QOS_FROM_XML','1'),('RMW_FASTRTPS_PUBLICATION_MODE','ASYNCHRONOUS'))]+nav_actions
        return [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=release)])),
                RegisterEventHandler(OnProcessIO(on_stdout=transcript.output,on_stderr=transcript.output)),
                RegisterEventHandler(OnProcessExit(on_exit=transcript.exited)),
                GroupAction(actions=nav_actions), localization_manager, delivery_goal_guard,
                Node(package='smart_delivery_core',executable='navigation_health',output='screen')]

    return LaunchDescription(
        [
            map_argument,
            params_argument,
            optional_idle_argument,
            behavior_tree_argument,
            use_sim_time_argument,
            auto_initialize_argument,
            DeclareLaunchArgument('use_composition',default_value='False',
                                  description='Separate Nav2 processes by default; true is the container comparison'),
            DeclareLaunchArgument('nav_transport_profile',default_value='async',
                                  description='async uses bounded asynchronous UDP; current is the legacy comparison'),
            OpaqueFunction(function=guarded_start),
        ]
    )
