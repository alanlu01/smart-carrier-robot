import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # 1. 啟動雷達節點 (給它眼睛)
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/rplidar',  # 🌟 換成固定綁定名稱
            'frame_id': 'laser',
            'serial_baudrate': 256000,
            'angle_compensate': True
        }],
        output='screen'
    )

    # 2. 啟動避障大腦 (直接執行尼寫好的 Python 腳本)
    safety_node = ExecuteProcess(
        cmd=['python3', os.path.expanduser('~/rplidar_ws/safety_interceptor.py')],
        output='screen'
    )

    # 打包一起發射！
    return LaunchDescription([
        rplidar_node,
        safety_node
    ])
