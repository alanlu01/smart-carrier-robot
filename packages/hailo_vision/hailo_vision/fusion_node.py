import json
import math
import struct 
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Float32, Header, String

from hailo_vision.semantic_protocol import (
    closest_timestamped_item,
    parse_semantic_payload,
    scan_sync_safety_multiplier,
    semantic_data_age,
    semantic_health_state,
)
from hailo_vision.pipeline_metrics import PipelineMetrics
from hailo_vision.recovery_lease import RecoveryLease

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
        self.scan_unmatched_since = None
        self.scan_history = deque()
        self.last_person_limit_at = None
        self.held_person_multiplier = 1.0
        self.person_safety_state = 'clear'
        self.history = {} 
        self.pipeline_metrics = PipelineMetrics(time.monotonic())
        self.last_pipeline_report_at = time.monotonic()
        self.semantic_sequence = None
        self.recovery_lease = RecoveryLease()

        self.declare_parameter('semantic_timeout_sec', 0.80)
        self.declare_parameter('semantic_recovery_timeout_sec', 0.40)
        self.declare_parameter('semantic_max_sync_skew_sec', 0.20)
        self.declare_parameter('semantic_stale_speed_multiplier', 0.50)
        self.declare_parameter('semantic_hard_stop_timeout_sec', 2.00)
        self.declare_parameter('semantic_scan_history_sec', 1.00)
        self.declare_parameter('semantic_scan_miss_grace_sec', 0.60)
        self.declare_parameter('person_clear_hold_sec', 0.50)
        self.semantic_timeout_sec = float(
            self.get_parameter('semantic_timeout_sec').value
        )
        self.semantic_recovery_timeout_sec = float(
            self.get_parameter('semantic_recovery_timeout_sec').value
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
        self.semantic_scan_history_sec = float(
            self.get_parameter('semantic_scan_history_sec').value
        )
        self.semantic_scan_miss_grace_sec = float(
            self.get_parameter('semantic_scan_miss_grace_sec').value
        )
        self.person_clear_hold_sec = float(
            self.get_parameter('person_clear_hold_sec').value
        )
        if not (
            0 < self.semantic_recovery_timeout_sec
            < self.semantic_timeout_sec
            < self.semantic_hard_stop_timeout_sec
        ):
            raise ValueError(
                '語意逾時門檻須符合 0 < recovery < soft timeout < hard-stop timeout'
            )
        if self.semantic_max_sync_skew_sec < 0:
            raise ValueError('語意與雷達最大時間差不得小於 0')
        if not 0 <= self.semantic_stale_speed_multiplier <= 1:
            raise ValueError('語意逾時速度乘數必須介於 0 到 1')
        if self.semantic_scan_history_sec <= self.semantic_max_sync_skew_sec:
            raise ValueError('雷達歷史長度必須大於語意與雷達最大時間差')
        if self.semantic_scan_miss_grace_sec < 0:
            raise ValueError('語意與雷達同步缺漏寬限不得小於 0')
        if self.person_clear_hold_sec < 0:
            raise ValueError('人員減速保持時間不得小於 0')
        
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
        self.recovery_guard_pub = self.create_publisher(
            String, '/localization/recovery_motion_guard', 1
        )
        self.create_subscription(
            String, '/localization/recovery_motion_lease', self.recovery_lease_callback, 1
        )
        self.pub_speed_multiplier = self.create_publisher(
            Float32, '/semantic/speed_multiplier', 10
        )

        # 3. 發布虛擬玻璃點雲給 Nav2 代價地圖
        self.pub_virtual_glass = self.create_publisher(PointCloud2, '/visual_glass', 10)
        self.pipeline_pub = self.create_publisher(String, '/semantic/pipeline_health', 1)
        self.semantic_watchdog_timer = self.create_timer(
            0.1, self.semantic_watchdog_callback
        )

        self.get_logger().info('🧠 融合節點啟動：極速煞車攔截器與虛擬牆壁建造者已上線！')

    def vision_callback(self, msg):
        self.pipeline_metrics.observe('semantic', time.monotonic())
        try:
            packet = parse_semantic_payload(msg.data)
            self.semantic_sequence = packet.sequence
            if packet.stamp_ns is not None:
                self.pipeline_metrics.measure(
                    'source_age_at_receive_sec',
                    (self.get_clock().now().nanoseconds - packet.stamp_ns) / 1e9,
                )
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
            matching_scan = self._matching_scan(packet.stamp_ns)
            if matching_scan is None:
                now = time.monotonic()
                if self.scan_unmatched_since is None:
                    self.scan_unmatched_since = now
                unmatched_age = now - self.scan_unmatched_since
                next_multiplier = scan_sync_safety_multiplier(
                    self.speed_multiplier,
                    unmatched_age,
                    self.semantic_scan_miss_grace_sec,
                    self.semantic_stale_speed_multiplier,
                )
                if (
                    unmatched_age >= self.semantic_scan_miss_grace_sec
                    and now - self.last_sync_warning_at >= 10.0
                ):
                    self.get_logger().warning(
                        '語意影像與雷達持續無法同步 '
                        f'{unmatched_age:.2f}s，採保守半速'
                    )
                    self.last_sync_warning_at = now
                if next_multiplier < self.speed_multiplier:
                    self._set_speed_multiplier(
                        next_multiplier,
                        reason='scan_unmatched_persistent',
                    )
                return
            self.scan_unmatched_since = None
            self._process_semantic_data(packet.detections, matching_scan)
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
        state = semantic_health_state(
            self.semantic_age(now),
            self.semantic_health_state,
            self.semantic_timeout_sec,
            self.semantic_recovery_timeout_sec,
            self.semantic_hard_stop_timeout_sec,
        )
        if state == 'stopped':
            return 0.0
        if state == 'stale':
            return self.semantic_stale_speed_multiplier
        return 1.0

    def semantic_watchdog_callback(self):
        now = time.monotonic()
        if self.recovery_lease.enabled:
            if self.recovery_lease.blocked(now):
                self.pub_safe_cmd.publish(Twist())
            self.publish_recovery_guard(now)
        self.pipeline_metrics.observe('watchdog', now)
        if now - self.last_pipeline_report_at >= 1.0:
            payload = self.pipeline_metrics.snapshot(now)
            payload['sequence'] = self.semantic_sequence
            payload['source_age_sec'] = round(self.semantic_age(now), 3)
            payload['effective_speed_multiplier'] = min(
                self.speed_multiplier, self.semantic_speed_limit(now)
            )
            message = String()
            message.data = json.dumps(payload)
            self.pipeline_pub.publish(message)
            self.last_pipeline_report_at = now
        next_state = semantic_health_state(
            self.semantic_age(),
            self.semantic_health_state,
            self.semantic_timeout_sec,
            self.semantic_recovery_timeout_sec,
            self.semantic_hard_stop_timeout_sec,
        )
        if self.latest_semantic_received_at is None and next_state == 'healthy':
            next_state = 'starting'

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
        lease = self.recovery_lease
        if lease.blocked(time.monotonic()):
            self.pub_safe_cmd.publish(Twist())
            return
        effective_multiplier = min(
            self.speed_multiplier,
            self.semantic_speed_limit(),
        )
        real_cmd = Twist()
        real_cmd.linear.x = msg.linear.x * effective_multiplier
        real_cmd.linear.y = msg.linear.y * effective_multiplier
        real_cmd.angular.z = msg.angular.z * effective_multiplier
        if lease.enabled:
            # Translation permission never authorizes other motion or a speed
            # above the agreed cap; semantic stop/reduction still applies.
            real_cmd.linear.x = real_cmd.angular.z = 0.0
            real_cmd.linear.y = max(-0.25, min(0.25, msg.linear.y)) * effective_multiplier
        self.pub_safe_cmd.publish(real_cmd)

    def recovery_lease_callback(self, message):
        try:
            payload = json.loads(message.data)
            was_enabled = self.recovery_lease.enabled
            if self.recovery_lease.receive(payload, time.monotonic()):
                # Flush any preceding spin/navigation velocity before arming.
                if payload.get('active') is True and not was_enabled:
                    self.pub_safe_cmd.publish(Twist())
                self.publish_recovery_guard(time.monotonic())
        except (ValueError, TypeError, AttributeError):
            self.pub_safe_cmd.publish(Twist())

    def publish_recovery_guard(self, now):
        message = String()
        message.data = json.dumps({
            'token': self.recovery_lease.token,
            'active': self.recovery_lease.enabled,
            'blocked': self.recovery_lease.blocked(now),
        })
        self.recovery_guard_pub.publish(message)

    def scan_callback(self, scan_msg):
        self.pipeline_metrics.observe('scan_filtered', time.monotonic())
        scan_stamp_ns = (
            scan_msg.header.stamp.sec * 1_000_000_000
            + scan_msg.header.stamp.nanosec
        )
        self.scan_history.append((scan_stamp_ns, scan_msg))
        oldest_allowed = scan_stamp_ns - int(self.semantic_scan_history_sec * 1e9)
        while self.scan_history and self.scan_history[0][0] < oldest_allowed:
            self.scan_history.popleft()

    def _matching_scan(self, semantic_stamp_ns):
        match = closest_timestamped_item(
            self.scan_history,
            semantic_stamp_ns,
            self.semantic_max_sync_skew_sec,
        )
        return None if match is None else match[1]

    def _apply_person_clear_hold(self, detected_multiplier):
        now = time.monotonic()
        if detected_multiplier < 1.0:
            self.last_person_limit_at = now
            self.held_person_multiplier = detected_multiplier
            return detected_multiplier
        if (
            self.last_person_limit_at is not None
            and now - self.last_person_limit_at <= self.person_clear_hold_sec
        ):
            return self.held_person_multiplier
        self.last_person_limit_at = None
        self.held_person_multiplier = 1.0
        return 1.0

    def _set_speed_multiplier(self, multiplier, reason='clear'):
        multiplier = max(0.0, min(1.0, float(multiplier)))
        previous_state = self.person_safety_state
        if multiplier <= 0.0:
            next_state = 'stopped'
        elif multiplier < 1.0:
            next_state = 'slowed'
        else:
            next_state = 'clear'

        self.speed_multiplier = multiplier
        status = Float32()
        status.data = multiplier
        self.pub_speed_multiplier.publish(status)

        if next_state == previous_state:
            return
        if next_state == 'stopped':
            self.get_logger().warning(f'🛑 語意安全煞停（{reason}）')
            self.pub_safe_cmd.publish(Twist())
        elif next_state == 'slowed':
            self.get_logger().info(
                f'⚠️ 語意安全減速至 {round(multiplier * 100)}%（{reason}）'
            )
        else:
            self.get_logger().info('✅ 語意安全區域已清空，恢復導航速度')
        self.person_safety_state = next_state

    def _process_semantic_data(self, detections, scan_msg):
        if self.semantic_age() > self.semantic_timeout_sec:
            self.latest_semantic_data = []
            self._set_speed_multiplier(
                min(self.speed_multiplier, self.semantic_stale_speed_multiplier),
                reason='semantic_stale',
            )
            self._update_glass_tracker([], scan_msg.header)
            return

        if not detections:
            self._set_speed_multiplier(self._apply_person_clear_hold(1.0))
            self._update_glass_tracker([], scan_msg.header)
            return

        current_multiplier = 1.0
        person_limit_reason = 'clear'
        glass_points_this_frame = [] # 記錄這幀看到的有效玻璃座標

        for target in detections:
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
                    person_limit_reason = f'優先禮讓目標距離 {smooth_dist:.2f} m'
                elif smooth_dist < 1.5:
                    current_multiplier = min(current_multiplier, 0.5)
                    if current_multiplier > 0.0:
                        person_limit_reason = f'優先禮讓目標距離 {smooth_dist:.2f} m'

        # 更新全局速度與玻璃追蹤
        current_multiplier = self._apply_person_clear_hold(current_multiplier)
        self._set_speed_multiplier(current_multiplier, person_limit_reason)
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
