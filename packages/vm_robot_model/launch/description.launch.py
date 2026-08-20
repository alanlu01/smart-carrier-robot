import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 🌟 修改點：將套件名稱對齊我們剛才新建的資料夾名稱 'vm_robot_model'
    pkg_path = get_package_share_directory('vm_robot_model')
    urdf_file_path = os.path.join(pkg_path, 'urdf', 'smart_carrier.urdf')
    
    # 讀取 URDF 內容
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # 🤖 啟動 Robot State Publisher 節點，發布靜態 TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': False
            }]
        )
    ])
