"""Compose Nav2 while keeping callback code loaded until process exit."""
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from ament_index_python.resources import get_resource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    upstream = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    composed = LaunchConfiguration('use_composition')
    params = LaunchConfiguration('params_file')
    sim = LaunchConfiguration('use_sim_time')
    # This installed Jazzy version retains plugin-owned callbacks in rclcpp's
    # context until process exit. Pin their code for the same lifetime, rather
    # than permitting component/plugin unloading to leave dangling callbacks.
    packages = ('nav2_map_server','nav2_amcl','nav2_controller','nav2_smoother',
        'nav2_planner','nav2_route','nav2_behaviors','nav2_bt_navigator',
        'nav2_waypoint_follower','nav2_velocity_smoother','nav2_collision_monitor',
        'nav2_lifecycle_manager','opennav_docking')
    libraries = set()
    for package in packages:
        contents, prefix = get_resource('rclcpp_components', package)
        for line in contents.splitlines():
            plugin, relative = line.split(';')
            if plugin.endswith('CollisionDetector'):
                continue
            libraries.add(os.path.join(prefix, relative))
    prefix = get_package_prefix('nav2_mppi_controller')
    libraries.update(os.path.join(prefix, 'lib', name) for name in
                     ('libmppi_controller.so','libmppi_critics.so'))
    if not all(os.path.isfile(path) for path in libraries):
        raise RuntimeError('Installed Nav2 component libraries are incomplete')
    preload = ':'.join(sorted(libraries))
    if os.environ.get('LD_PRELOAD'):
        preload += ':'+os.environ['LD_PRELOAD']
    arguments = dict(map=LaunchConfiguration('map'), params_file=params,
                     use_sim_time=sim, autostart='true', use_composition=composed)
    return LaunchDescription([
        DeclareLaunchArgument('map'), DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_composition', default_value='True'),
        DeclareLaunchArgument('autostart', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(upstream, 'bringup_launch.py')),
            condition=UnlessCondition(composed), launch_arguments=arguments.items()),
        GroupAction(condition=IfCondition(composed), actions=[
            SetParameter('use_sim_time', sim),
            Node(package='rclcpp_components', executable='component_container_isolated',
                name='nav2_container', output='screen', parameters=[params, {'autostart': True}],
                remappings=[('/tf','tf'),('/tf_static','tf_static')],
                additional_env={'LD_PRELOAD': preload}),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(upstream, 'localization_launch.py')),
                launch_arguments={**arguments, 'use_composition': 'True', 'container_name': 'nav2_container'}.items()),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(upstream, 'navigation_launch.py')),
                launch_arguments={**arguments, 'use_composition': 'True', 'container_name': 'nav2_container'}.items()),
        ]),
    ])
