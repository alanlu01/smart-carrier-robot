import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction  # 👈 確保引入這兩個新道具
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap

def generate_launch_description():
    
    # 1. 🌟 建圖大腦 slam_toolbox
    # 透過 GroupAction 建立結界，強制將官方預設的 /scan 話題重新導向 /scan_filtered
    slam_toolbox_launch = GroupAction(
        actions=[
            SetRemap(src='/scan', dst='/scan_filtered'),  # 👈 在這裡進行話題的攔截與替換
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
                ])
            )
        ]
    )

    # 2. RViz2 節點 (建議存一個專門用來看地圖的設定檔 mapping.rviz)
    # 🌟 讓系統自動找出我們新建的 smart_delivery_core 套件安裝在哪裡
    pkg_dir = get_package_share_directory('smart_delivery_core')
    
    # 🌟 將原本寫死的路徑，改為動態組合路徑，指向 rviz 資料夾裡的 mapping.rviz
    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'mapping.rviz')
    
    rviz_node = Node(
        package='rviz2', 
        executable='rviz2', 
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # 🌟 砍掉了 rf2o 和 fake_tf，因為尼的 smart_carrier.urdf 和 mecanum_odom_real 已經提供最真實的數據了！
    # 🌟 雷達節點也不需要寫在這裡，讓尼原本的系統統一啟動硬體即可。

    return LaunchDescription([
        slam_toolbox_launch,
        rviz_node
    ])
