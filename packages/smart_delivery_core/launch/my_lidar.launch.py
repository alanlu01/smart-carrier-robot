import os
from launch import LaunchDescription
from launch_ros.actions import Node
# 🌟 引入 ROS 2 的動態尋路魔法套件
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 🌟 讓系統自動找出我們新建的 smart_delivery_core 套件安裝在哪裡
    pkg_dir = get_package_share_directory('smart_delivery_core')

    # 🌟 定義 laser_filter 淨水器的設定檔路徑 
    # (⚠️ 請確保尼有在 smart_delivery_core 裡面建立 config 資料夾與這個 yaml 檔)
    filter_config = os.path.join(pkg_dir, 'config', 'my_laser_filter.yaml')

    # 1. 雷達節點 (專心驅動硬體)
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/rplidar',  # 🌟 已經換成固定綁定名稱
            'frame_id': 'laser',
            'serial_baudrate': 256000,
            'angle_compensate': True,       # 👈 這裡原本少了一個逗號，已經幫尼補上！
            'range_min': 0.15               # 👈 讓硬體先粗略濾掉 15cm 內的極近雜訊 (保險機制)
        }],
        output='screen'
    )

    # 2. 雷達淨水器節點 (吃進 /scan，過濾掉車身排線後，吐出乾淨的 /scan_filtered)
    filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain',  # 🌟 關鍵修改：明確指定名字，確保吃到 yaml 設定
        parameters=[filter_config],
        remappings=[
            ('scan', '/scan'),                  
            ('scan_filtered', '/scan_filtered') 
        ],
        output='screen'
    )

    # 3. 定義 RViz2 的設定檔路徑 (🌟 已經幫尼改成動態組合路徑，指向 rviz 資料夾)
    # rviz_config_path = os.path.join(pkg_dir, 'rviz', 'rplidar_rviz.rviz')

    # 4. RViz2 節點 (🌟 全部註解掉，將畫面呈現任務交給主系統的 3D 模型)
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_path], 
    #     output='screen'
    # )

    # 5. 打包發射 (🌟 把 filter_node 一併加入啟動清單)
    return LaunchDescription([
        rplidar_node,
        filter_node
    ])
