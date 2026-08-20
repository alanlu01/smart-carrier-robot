import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
# 🌟 引入專業的 Node 模組來取代舊的 ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    
    # 取得我們兩個核心套件的動態路徑
    core_pkg_path = get_package_share_directory('smart_delivery_core')
    model_pkg_path = get_package_share_directory('vm_robot_model')

    # ==========================================
    # 1. 載入光達啟動檔
    # ==========================================
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(core_pkg_path, 'launch', 'my_lidar.launch.py')
        )
    )

    # ==========================================
    # 🌟 2. 封印 3D 模型與 RViz2 (移交給西風 G16 負責)
    # 為了徹底解放樹莓派 CPU，我們在這裡將畫面與骨架渲染全部註解掉
    # ==========================================
    rviz_config_path = os.path.join(model_pkg_path, 'rviz', 'nav_test.rviz')
    # 
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(model_pkg_path, 'launch', 'display.launch.py')
        ),
        launch_arguments={'rviz_config': rviz_config_path}.items()
    )

    # ==========================================
    # 3. 執行底層通訊與里程計
    # ==========================================
    
    # 執行 cmd_vel_to_serial
    cmd_to_vel_node = Node(
        package='smart_delivery_core',
        executable='cmd_vel_to_serial',
        name='cmd_vel_to_serial',
        arguments=['ROS'],
        output='screen'
    )

    # 執行 serial_to_ros
    stm_to_ros_node = Node(
        package='smart_delivery_core',
        executable='serial_to_ros',
        name='serial_to_ros',
        arguments=['STM32'],
        output='screen'
    )

    # 執行 mecanum_odom_real
    odom_node = Node(
        package='smart_delivery_core',
        executable='mecanum_odom_real',
        name='mecanum_odom_real',
        output='screen'
    )

    # ==========================================
    # 4. 打包所有任務，一鍵發射！
    # ==========================================
    return LaunchDescription([
        lidar_launch,
        # display_launch,  # 🌟 這裡記得要同步註解掉，不讓它啟動
        cmd_to_vel_node,
        stm_to_ros_node,
        odom_node
    ])
