import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # === 路徑設定 ===
    # 指向尼剛剛存好的地圖
    map_file = os.path.expanduser('~/rplidar_ws/lab_map.yaml') 
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # === 1. 硬體與感覺器官 ===
    # 雷達
    rplidar_node = Node(package='rplidar_ros', executable='rplidar_node', parameters=[{
            'serial_port': '/dev/rplidar',  # 🌟 換成固定綁定名稱
            'frame_id': 'laser',
            'serial_baudrate': 256000,
            'angle_compensate': True
        }])
    
    # 假底盤 TF
    fake_footprint_node = Node(package='tf2_ros', executable='static_transform_publisher', arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'])
    fake_base_node = Node(package='tf2_ros', executable='static_transform_publisher', arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'])
    
    # 雷射里程計 (rf2o)
    rf2o_node = Node(
        package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node', name='rf2o_laser_odometry',
        parameters=[{'laser_scan_topic': '/scan', 'odom_topic': '/odom', 'publish_tf': True, 'base_frame_id': 'base_footprint', 'odom_frame_id': 'odom', 'init_pose_from_topic': '', 'freq': 20.0}]
    )

    # === 2. 🧠 導航大腦 (Nav2 Bringup) ===
    # 明確指定官方的預設參數檔路徑！
    nav2_params_path = os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')]),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'false',
            'params_file': nav2_params_path  # 補上這行保命符！
        }.items(),
    )

    # === 3. RViz2 (使用 Nav2 官方的超豐富介面) ===
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    rviz_node = Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config_dir], output='screen')

    # 打包發射！
    return LaunchDescription([
        rplidar_node, fake_footprint_node, fake_base_node, rf2o_node,
        nav2_launch, rviz_node
    ])
