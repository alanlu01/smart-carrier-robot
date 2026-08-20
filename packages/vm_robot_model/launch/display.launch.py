import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 取得套件安裝共享目錄路徑 
    # (🌟 請確認尼在樹莓派上的套件名稱是 vm_robot_model 還是 smart_carrier_description)
    pkg_path = get_package_share_directory('vm_robot_model')
    urdf_file_path = os.path.join(pkg_path, 'urdf', 'smart_carrier.urdf')
    
    # 2. 讀取 URDF 的 XML 3D 模型描述內容
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    # 📌 關鍵修正：宣告 rviz_config 變數
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='',
        description='Absolute path to RViz config file'
    )

    # 🤖 節點 A：Robot State Publisher (發布 TF 座標與模型數據，這個必須留著！)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False
        }]
    )

    # 🛞 節點 B：Joint State Publisher (防止 TF 遺失警告，這個也必須留著！)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}]
    )

    # ==========================================
    # 📺 節點 C：RViz 2 本體 (🌟 封印它！交給西風筆電去開)
    # ==========================================
    # rviz_config_file = LaunchConfiguration('rviz_config')
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config_file]
    # )

    return LaunchDescription([
        rviz_config_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        # rviz_node   # 🌟 這裡也要記得註解掉！
    ])
