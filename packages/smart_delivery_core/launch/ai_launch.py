import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    power_config = os.path.join(
        get_package_share_directory('power_monitor'), 'config', 'power_monitor.yaml'
    )
    return LaunchDescription([
        # 1. 啟動相機節點 (這裡沒有 &，由 Launch 系統統一接管)
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera_node',
            parameters=[{
                'width': 640,
                'height': 360,
                'af_mode': 2,
                'fps': 15.0
            }]
        ),

        # 2. 啟動 Hailo 語意辨識節點
        Node(
            package='hailo_vision',
            executable='semantic_node',
            name='semantic_node',
            remappings=[
                ('/camera/image_raw', '/camera_node/image_raw'),
            ]
        ),

        # 3. 啟動 Hailo 感測融合節點
        Node(
            package='hailo_vision',
            executable='fusion_node',
            name='fusion_node',
            parameters=[{
                # Ignore isolated camera/LiDAR timestamp misses. Persistent sync
                # loss remains fail-safe at half speed.
                'semantic_scan_miss_grace_sec': 0.60,
            }]
        ),

        # 4. 啟動 INA3221 電源監控節點
        Node(
            package='power_monitor',
            executable='ina3221_node',
            name='ina3221_node',
            parameters=[power_config]
        )
    ])
