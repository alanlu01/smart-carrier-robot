import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 雷達節點
    rplidar_node = Node(package='rplidar_ros', executable='rplidar_node', parameters=[{
            'serial_port': '/dev/rplidar',  # 🌟 換成固定綁定名稱
            'frame_id': 'laser',
            'serial_baudrate': 256000,
            'angle_compensate': True
        }])

    # 2. RViz2 節點
    rviz_config_path = os.path.expanduser('~/rplidar_ws/slam_hand.rviz')
    rviz_node = Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config_path])

    # 3. 🌟 新武器：雷射里程計 (取代原本的假 odom！)
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 20.0
        }]
    )

    # 4. 保留車體內部的靜態連結 (Footprint -> Link -> Laser)
    fake_footprint_node = Node(package='tf2_ros', executable='static_transform_publisher', arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'])
    fake_base_node = Node(package='tf2_ros', executable='static_transform_publisher', arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'])

    # 5. 建圖大腦 slam_toolbox
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')])
    )

    return LaunchDescription([
        rplidar_node, rviz_node, 
        rf2o_node,  # 加入雷射里程計
        fake_footprint_node, fake_base_node, 
        slam_toolbox_launch
    ])
