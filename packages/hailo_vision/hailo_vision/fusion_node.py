import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField 
from std_msgs.msg import String, Header
from geometry_msgs.msg import Twist
import json
import math
import numpy as np
from collections import deque
import struct 
import time

from hailo_vision.semantic_protocol import (
    parse_semantic_payload,
    semantic_data_age,
    semantic_safety_multiplier,
    stamps_are_synchronized,
)

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        self.latest_semantic_data = []
        self.latest_semantic_stamp_ns = None
        self.latest_semantic_received_at = None
        self.last_semantic_source_stamp_ns = None
        self.semantic_started_at = time.monotonic()
        self.semantic_health_state = 'starting'
        self.last_semantic_error_at = 0.0
        self.last_sync_warning_at = 0.0
        self.history = {} 

        self.declare_parameter('semantic_timeout_sec', 0.50)
        self.declare_parameter('semantic_max_sync_skew_sec', 0.20)
        self.declare_parameter('semantic_stale_speed_multiplier', 0.50)
        self.declare_parameter('semantic_hard_stop_timeout_sec', 2.00)
        self.semantic_timeout_sec = float(
            self.get_parameter('semantic_timeout_sec').value
        )
        self.semantic_max_sync_skew_sec = float(
            self.get_parameter('semantic_max_sync_skew_sec').value
        )
        self.semantic_stale_speed_multiplier = float(
            self.get_parameter('semantic_stale_speed_multiplier').value
        )
        self.semantic_hard_stop_timeout_sec = float(
            self.get_parameter('semantic_hard_stop_timeout_sec').value
        )
        if not 0 < self.semantic_timeout_sec < self.semantic_hard_stop_timeout_sec:
            raise ValueError('語意 soft timeout 必須大於 0 且小於 hard-stop timeout')
        if self.semantic_max_sync_skew_sec < 0:
            raise ValueError('語意與雷達最大時間差不得小於 0')
        if not 0 <= self.semantic_stale_speed_multiplier <= 1:
            raise ValueError('語意逾時速度乘數必須介於 0 到 1')
        
        # 🌟 速度控制乘數
        self.speed_multiplier = 1.0

        # 🌟 玻璃專屬防護機制 (連續計數器)
        self.glass_hit_count = 0
        self.GLASS_HIT_MAX = 5    # 最高累積信任度
        self.GLASS_HIT_THRESH = 3 # 達標門檻：連續看到才發布

        # 1. 訂閱情報與點雲
        self.sub_vision = self.create_subscription(String, '/vision/semantic_info', self.vision_callback, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan_filtered', self.scan_callback, 10)
        
        # 2. 訂閱與發布速度 (確保 cmd_vel_to_serial 訂閱的是 /chassis_cmd_vel！)
        self.sub_nav2_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.pub_safe_cmd = self.create_publisher(Twist, '/chassis_cmd_vel', 10)

        # 3. 發布虛擬玻璃點雲給 Nav2 代價地圖
        self.pub_virtual_glass = self.create_publisher(PointCloud2, '/visual_glass', 10)
        self.semantic_watchdog_timer = self.create_timer(
            0.1, self.semantic_watchdog_callback
        )

        self.get_logger().info('🧠 融合節點啟動：極速煞車攔截器與虛擬牆壁建造者已上線！')

    def vision_callback(self, msg):
        try:
            packet = parse_semantic_payload(msg.data)
            if (
                packet.stamp_ns is not None
                and self.last_semantic_source_stamp_ns is not None
                and packet.stamp_ns <= self.last_semantic_source_stamp_ns
            ):
                return
            self.latest_semantic_data = packet.detections
            self.latest_semantic_stamp_ns = packet.stamp_ns
            self.latest_semantic_received_at = time.monotonic()
            if packet.stamp_ns is not None:
                self.last_semantic_source_stamp_ns = packet.stamp_ns
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            now = time.monotonic()
            if now - self.last_semantic_error_at >= 10.0:
                self.get_logger().warning(f'無效的語意資料，已忽略: {exc}')
                self.last_semantic_error_at = now

    def semantic_age(self, now=None):
        now = time.monotonic() if now is None else now
        reference = self.latest_semantic_received_at
        if reference is None:
            reference = self.semantic_started_at
        receipt_age = max(0.0, now - reference)
        return semantic_data_age(
            receipt_age,
            source_stamp_ns=self.latest_semantic_stamp_ns,
            now_ns=self.get_clock().now().nanoseconds,
        )

    def semantic_speed_limit(self, now=None):
        return semantic_safety_multiplier(
            self.semantic_age(now),
            self.semantic_timeout_sec,
            self.semantic_stale_speed_multiplier,
            self.semantic_hard_stop_timeout_sec,
        )

    def semantic_watchdog_callback(self):
        speed_limit = self.semantic_speed_limit()
        if self.latest_semantic_received_at is None and speed_limit == 1.0:
            next_state = 'starting'
        elif speed_limit == 1.0:
            next_state = 'healthy'
        elif speed_limit > 0.0:
            next_state = 'stale'
        else:
            next_state = 'stopped'

        if next_state == self.semantic_health_state:
            return
        if next_state == 'healthy':
            self.get_logger().info('語意資料已恢復為即時狀態')
        elif next_state == 'stale':
            speed_percent = round(self.semantic_stale_speed_multiplier * 100)
            self.get_logger().warning(
                f'語意資料已逾時，停止使用舊偵測並限制車速為 {speed_percent}%'
            )
        else:
            self.get_logger().error('語意資料持續逾時，已輸出零速命令')
            self.pub_safe_cmd.publish(Twist())
        self.semantic_health_state = next_state

    def cmd_vel_callback(self, msg):
        effective_multiplier = min(
            self.speed_multiplier,
            self.semantic_speed_limit(),
        )
        real_cmd = Twist()
        real_cmd.linear.x = msg.linear.x * effective_multiplier
        real_cmd.linear.y = msg.linear.y * effective_multiplier
        real_cmd.angular.z = msg.angular.z * effective_multiplier
        self.pub_safe_cmd.publish(real_cmd)

    def scan_callback(self, scan_msg):
        if self.semantic_age() > self.semantic_timeout_sec:
            self.latest_semantic_data = []
            self.speed_multiplier = 1.0
            self._update_glass_tracker([], scan_msg.header)
            return

        scan_stamp_ns = (
            scan_msg.header.stamp.sec * 1_000_000_000
            + scan_msg.header.stamp.nanosec
        )
        if not stamps_are_synchronized(
            self.latest_semantic_stamp_ns,
            scan_stamp_ns,
            self.semantic_max_sync_skew_sec,
        ):
            now = time.monotonic()
            if now - self.last_sync_warning_at >= 10.0:
                self.get_logger().warning(
                    '語意影像與雷達時間差過大，本次不進行融合'
                )
                self.last_sync_warning_at = now
            self.speed_multiplier = 1.0
            self._update_glass_tracker([], scan_msg.header)
            return

        if not self.latest_semantic_data:
            self.speed_multiplier = 1.0 
            self._update_glass_tracker([]) # 沒看到東西，衰減玻璃計數
            return

        current_multiplier = 1.0
        glass_points_this_frame = [] # 記錄這幀看到的有效玻璃座標

        for target in self.latest_semantic_data:
            cls_id = target['id']
            display_angle_deg = target['angle']

            if cls_id not in self.history:
                self.history[cls_id] = {'angle': deque(maxlen=5), 'dist': deque(maxlen=5)}

            if len(self.history[cls_id]['angle']) > 0:
                last_angle = self.history[cls_id]['angle'][-1]
                if abs(display_angle_deg - last_angle) > 20.0:
                    self.history[cls_id]['angle'].clear()
                    self.history[cls_id]['dist'].clear()

            # 鏡像修正與 Index 計算
            LIDAR_DIR_MULTIPLIER = -1.0  
            lidar_target_angle_deg = 180.0 + (display_angle_deg * LIDAR_DIR_MULTIPLIER)
            lidar_target_angle_rad = math.radians(lidar_target_angle_deg % 360.0)
            
            angle_offset = (lidar_target_angle_rad - scan_msg.angle_min + 2 * math.pi) % (2 * math.pi)
            center_index = int(angle_offset / scan_msg.angle_increment)
            
            target_width_deg = (target['bbox_width'] / 640.0) * 68.0
            search_window_deg = max((target_width_deg / 2) * 1.2, 3.0) 
            window_idx = int(math.radians(search_window_deg) / scan_msg.angle_increment)

            valid_distances = []
            for i in range(-window_idx, window_idx + 1):
                idx = (center_index + i) % len(scan_msg.ranges)
                dist = scan_msg.ranges[idx]
                if 0.35 < dist < scan_msg.range_max and not math.isinf(dist) and not math.isnan(dist):
                    valid_distances.append(dist)

            if not valid_distances:
                continue

            valid_distances.sort()
            raw_dist = valid_distances[1] if len(valid_distances) > 1 else valid_distances[0]
            raw_dist = raw_dist - 0.0

            # ==========================================
            # 🛡️ 玻璃專屬處理 (ID: 10)
            # ==========================================
            if cls_id == 10:
                # 防護一：面積與深度交集 (太遠或 Bbox 太小，視為幻覺)
                if raw_dist > 2.0 or target['bbox_width'] < 100:
                    continue 
                
                # 計算出玻璃相對於雷達的 X, Y 座標
                glass_x = raw_dist * math.cos(lidar_target_angle_rad)
                glass_y = raw_dist * math.sin(lidar_target_angle_rad)
                glass_points_this_frame.append((glass_x, glass_y))
                continue # 玻璃不參與減速邏輯，交給 Costmap 處理

            # 紀錄其他目標的歷史
            self.history[cls_id]['angle'].append(display_angle_deg)
            self.history[cls_id]['dist'].append(raw_dist)
            
            # 👇 🌟 關鍵修改 1：捨棄中位數 (np.median)，改用 min 確保對最近危險立刻反應！
            smooth_dist = float(min(self.history[cls_id]['dist']))

            # 禮讓邏輯 (人、車等)
            if cls_id in [0, 2, 3, 4]:
                if smooth_dist < 0.6:
                    current_multiplier = 0.0  
                    self.get_logger().warn(f"🛑 緊急煞停！距離 {smooth_dist:.2f}m 發現優先禮讓目標！")
                    
                    # 👇 🌟 關鍵修改 2：立刻主動發布全 0 的停止指令給底盤，不等待 Nav2！
                    stop_msg = Twist()
                    self.pub_safe_cmd.publish(stop_msg)
                    
                elif smooth_dist < 1.5:
                    current_multiplier = min(current_multiplier, 0.5) 
                    self.get_logger().info(f"⚠️ 減速通過！距離 {smooth_dist:.2f}m 發現優先禮讓目標。")

        # 更新全局速度與玻璃追蹤
        self.speed_multiplier = current_multiplier
        self._update_glass_tracker(glass_points_this_frame, scan_msg.header)

    def _update_glass_tracker(self, glass_points, header=None):
        """🛡️ 防護二與三：連續計數器與動態權重衰減"""
        if glass_points:
            # Hit (命中)：一次加 2 分，快速建立信任
            self.glass_hit_count = min(self.glass_hit_count + 2, self.GLASS_HIT_MAX)
        else:
            # Miss (丟失)：一次扣 1 分，容許模型偶爾眨眼
            self.glass_hit_count = max(self.glass_hit_count - 1, 0)

        # 如果沒有 header (例如沒收到語意資訊時)，建立一個預設的
        if header is None:
            header = Header()
            header.frame_id = "laser"
            header.stamp = self.get_clock().now().to_msg()

        # 當計數器達標，發布虛擬牆壁
        if self.glass_hit_count >= self.GLASS_HIT_THRESH and glass_points:
            self._publish_pointcloud(glass_points, header)
        else:
            # 未達標或衰減歸零，發布「空點雲」
            self._publish_pointcloud([], header)

    def _publish_pointcloud(self, points_2d, header):
        """將 2D 座標轉換為一堵寬度 40cm 的虛擬 3D 牆壁發布"""
        msg = PointCloud2()
        msg.header = header 
        msg.height = 1
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.is_dense = True

        if not points_2d:
            msg.width = 0
            msg.row_step = 0
            msg.data = b''
            self.pub_virtual_glass.publish(msg)
            return

        packed_points = []
        for (x, y) in points_2d:
            # 在中心點的左右各延伸 20cm，確保 Costmap 能畫出一道實體牆壁，不會漏縫
            angle = math.atan2(y, x)
            for offset in np.linspace(-0.2, 0.2, 5): 
                px = x - offset * math.sin(angle)
                py = y + offset * math.cos(angle)
                pz = 0.0 # 高度貼齊雷達平面
                packed_points.append(struct.pack('fff', px, py, pz))
        
        msg.width = len(packed_points)
        msg.row_step = msg.point_step * msg.width
        msg.data = b''.join(packed_points)
        
        self.pub_virtual_glass.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
