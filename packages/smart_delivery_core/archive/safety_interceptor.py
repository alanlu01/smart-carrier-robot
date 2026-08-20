import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class SafetyInterceptor(Node):
    def __init__(self):
        super().__init__('safety_interceptor')
        
        # 訂閱雷達數據
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
            
        # 發布安全速度給底盤
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 參數設定
        self.max_speed = 0.5        # 預設最高前進速度 (m/s)
        self.slow_zone = 0.8        # 開始減速的距離 (80cm)
        self.stop_zone = 0.3        # 強制煞停的距離 (30cm)
        self.strafe_speed = 0.2     # 橫移閃避的速度 (m/s，僅限麥克納姆輪)

    def get_min_distance(self, ranges, angle_min, angle_max, scan_min, scan_max):
        """取出特定視角範圍內的最小有效距離"""
        # RPLIDAR 的 0 度通常在正前方，逆時針增加
        # 為了處理跨越 0 度的情況 (例如 345度 ~ 15度)
        selected_ranges = []
        for i, dist in enumerate(ranges):
            # 計算當前點的角度 (度)
            angle = math.degrees(scan_min + i * (scan_max - scan_min) / len(ranges))
            angle = angle % 360
            
            # 判斷角度是否在指定範圍內
            if angle_min > angle_max: # 跨越 0 度的情況 (如前方)
                if angle >= angle_min or angle <= angle_max:
                    selected_ranges.append(dist)
            else:
                if angle_min <= angle <= angle_max:
                    selected_ranges.append(dist)

        # 過濾掉無效的 0 或無限大數值
        valid_ranges = [r for r in selected_ranges if 0.1 < r < 10.0]
        
        if not valid_ranges:
            return float('inf') # 如果沒掃到東西，視為無限遠
        return min(valid_ranges)

    def scan_callback(self, msg):
        cmd = Twist()
        cmd.linear.x = self.max_speed # 預設全速前進
        
        # 1. 劃分四大防區 (抓取各區 +- 15 度的最小距離)
        front_dist = self.get_min_distance(msg.ranges, 345, 15, msg.angle_min, msg.angle_max)
        left_dist  = self.get_min_distance(msg.ranges, 75, 105, msg.angle_min, msg.angle_max)
        right_dist = self.get_min_distance(msg.ranges, 255, 285, msg.angle_min, msg.angle_max)
        rear_dist  = self.get_min_distance(msg.ranges, 165, 195, msg.angle_min, msg.angle_max)

        # 2. 前方邏輯：線性減速與煞停
        if front_dist < self.stop_zone:
            cmd.linear.x = 0.0
            self.get_logger().warn(f'前方極度危險 ({front_dist:.2f}m)！強制煞停！')
        elif front_dist < self.slow_zone:
            # 線性減速公式：(當前距離 - 煞停區) / (減速區 - 煞停區)
            scale = (front_dist - self.stop_zone) / (self.slow_zone - self.stop_zone)
            cmd.linear.x = self.max_speed * scale
            self.get_logger().info(f'前方進入緩衝區 ({front_dist:.2f}m)，減速至 {cmd.linear.x:.2f} m/s')

        # 3. 側向邏輯：如果前方被擋住，且側邊有空間，則進行橫移 (僅限麥克納姆輪)
        if cmd.linear.x < self.max_speed: # 代表前方有障礙物
            if left_dist > 0.5 and right_dist < 0.5:
                # 右邊有障礙，左邊空曠 -> 向左橫移 (正 Y 軸)
                cmd.linear.y = self.strafe_speed
                self.get_logger().info('右側有障礙，向左橫移閃避中...')
            elif right_dist > 0.5 and left_dist < 0.5:
                # 左邊有障礙，右邊空曠 -> 向右橫移 (負 Y 軸)
                cmd.linear.y = -self.strafe_speed
                self.get_logger().info('左側有障礙，向右橫移閃避中...')

        # 4. 後方邏輯：警示音或 Log
        if rear_dist < 0.4:
             self.get_logger().warn(f'注意！後方有物體逼近 ({rear_dist:.2f}m)！')

        # 5. 發布指令給 STM32 底盤
        self.publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyInterceptor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 關閉前發布停止指令確保安全
        stop_cmd = Twist()
        node.publisher.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
