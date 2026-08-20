from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
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
            name='semantic_node'
        ),

        # 3. 啟動 Hailo 感測融合節點
        Node(
            package='hailo_vision',
            executable='fusion_node',
            name='fusion_node'
        ),

        # 4. 啟動 INA3221 電源監控節點
        Node(
            package='power_monitor',
            executable='ina3221_node',
            name='ina3221_node'
        )
    ])
