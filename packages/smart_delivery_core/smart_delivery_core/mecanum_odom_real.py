import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros
import math
import time
import json

from smart_delivery_core.wheel_feedback import WheelSampleAssembler
from smart_delivery_core.pipeline_health import DurationMetrics, ReceiptMetrics

class MecanumOdomReal(Node):
    def __init__(self):
        super().__init__('mecanum_odom_real')
        
        # 1. 訂閱隊友發布的 STM32 原始字串話題
        self.subscription = self.create_subscription(
            String,
            'stm32_data',
            self.stm32_data_callback,
            10)
            
        # 2. 初始化發布器 (Publisher) 與 TF 廣播器
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # 3. 機器人物理座標變數
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        # 4. 🎛️ 硬體引腳對應設定
        self.motor_mapping = {
            'A': 'FL',  # Front Left  (左前)
            'B': 'FR',  # Front Right (右前)
            'C': 'RL',  # Rear Left   (左後)
            'D': 'RR'   # Rear Right  (右後)
        }
        
        self.current_speeds = {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0}

        self.declare_parameter('wheel_batch_window_sec', 0.25)
        self.declare_parameter('wheel_state_timeout_sec', 0.80)
        self.declare_parameter('max_integration_dt_sec', 0.20)
        self.wheel_state_timeout_sec = float(
            self.get_parameter('wheel_state_timeout_sec').value
        )
        self.max_integration_dt_sec = float(
            self.get_parameter('max_integration_dt_sec').value
        )
        batch_window_sec = float(
            self.get_parameter('wheel_batch_window_sec').value
        )
        if self.wheel_state_timeout_sec <= 0 or self.max_integration_dt_sec <= 0:
            raise ValueError('里程計 timeout 與最大積分週期必須大於 0')
        self.feedback_assembler = WheelSampleAssembler(
            batch_window_sec=batch_window_sec,
            started_at=time.monotonic(),
        )
        self.feedback_state = 'waiting'
        self.receipt_metrics = ReceiptMetrics()
        self.duration_metrics = DurationMetrics()
        self.feedback_health_pub = self.create_publisher(
            String, '/chassis/feedback_health', 1
        )
        self.feedback_health_timer = self.create_timer(1.0, self.publish_feedback_health)
        
        # 🌟 5. 修正後的精準車體幾何參數
        self.wheel_radius = 0.08  # 真實輪子半徑 8cm
        self.wheel_base = 0.106   # 前後輪距的一半 (21.2 / 2)
        self.track_width = 0.101  # 左右輪距的一半 (20.2 / 2)
        self.geometry_factor = self.wheel_base + self.track_width
        
        # 🌟 6. 透過實車反推的黃金轉換係數
        self.rt_to_rad_s = 0.000389
        
        # 🌟 7. 新增：自轉專屬微調係數
        # 如果實體轉 360 度，虛擬轉了 420 度，就把這裡調小 (例如 360/420 = 0.85)
        self.yaw_ratio = 1.0
        
        # 🌟 8. 新增：反向補償係數 (抹平 RViz 中的虛擬偏右現象)
        self.compensation = {
            'FL': 1.0 / 1.005,
            'RL': 1.0 / 1.0,
            'FR': 1.0 / 0.998,
            'RR': 1.0 / 0.994
        }
        
        # 🌟 9. 因為是反向偏移 (里程計少算)，所以數值要大於 1.0
        # self.angular_scale = 1.10  # 建議先從 1.05 到 1.15 之間開始測

        self.last_time = time.monotonic()
        self.timer = self.create_timer(0.05, self.update_odometry)
        
        self.get_logger().info('真實硬體里程計解析節點已成功啟動！監聽中...')

    def stm32_data_callback(self, msg):
        with self.duration_metrics.measure('stm32_callback'):
            self._consume_stm32_data(msg)

    def _consume_stm32_data(self, msg):
        self.receipt_metrics.observe('stm32_data', time.monotonic())
        raw_str = msg.data
        try:
            snapshot = self.feedback_assembler.add_line(raw_str, time.monotonic())
            if snapshot is None:
                return

            next_speeds = {}
            for letter, raw_val in snapshot.items():
                wheel_position = self.motor_mapping[letter]
                real_rad_s = (
                    raw_val
                    * self.rt_to_rad_s
                    * self.compensation[wheel_position]
                )
                next_speeds[wheel_position] = (
                    -real_rad_s if letter in ['B', 'D'] else real_rad_s
                )
            self.current_speeds = next_speeds
            if self.feedback_state != 'healthy':
                self.get_logger().info('STM32 四輪回授已完整，里程計 watchdog 正常')
            self.feedback_state = 'healthy'
        except Exception as e:
            self.get_logger().error(f'字串解析出錯: {e}，原始字串為: {raw_str}')

    def euler_to_quaternion(self, yaw):
        q = Quaternion()
        q.x = 0.0; q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def update_odometry(self):
        with self.duration_metrics.measure('odom_callback'):
            self._update_odometry()

    def _update_odometry(self):
        current_time = time.monotonic()
        self.receipt_metrics.observe('odom_timer', current_time)
        dt = min(
            max(0.0, current_time - self.last_time),
            self.max_integration_dt_sec,
        )
        self.last_time = current_time

        if self.feedback_assembler.is_stale(
            current_time, self.wheel_state_timeout_sec
        ):
            self.current_speeds = {
                'FL': 0.0,
                'FR': 0.0,
                'RL': 0.0,
                'RR': 0.0,
            }
            if self.feedback_state != 'stale':
                age = self.feedback_assembler.sample_age(current_time)
                self.get_logger().warning(
                    f'STM32 四輪回授已逾時 {age:.2f}s，凍結里程計並輸出零速度'
                )
            self.feedback_state = 'stale'
        
        # 1. 抓取當前四個輪子的最新轉速快照
        w_fl = self.current_speeds['FL']
        w_fr = self.current_speeds['FR']
        w_rl = self.current_speeds['RL']
        w_rr = self.current_speeds['RR']
        
        # 2. 麥克納姆輪正運動學 (Forward Kinematics)
        R = self.wheel_radius
        vx = (w_fl + w_fr + w_rl + w_rr) * (R / 4.0)
        vy = (-w_fl + w_fr + w_rl - w_rr) * (R / 4.0)
        
        # 🌟 在這裡乘上自轉微調係數
        vth = (-w_fl + w_fr - w_rl + w_rr) * (R / (4.0 * self.geometry_factor)) * self.yaw_ratio
        
        # 3. 航向角坐標積分
        delta_x = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
        delta_y = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt
        delta_th = vth * dt
        
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        
        # 4. 發布標準 TF 坐標系轉換
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.euler_to_quaternion(self.th)
        with self.duration_metrics.measure('tf_publish'):
            self.tf_broadcaster.sendTransform(t)
        
        # 5. 發布標準 Odometry 話題
        odom = Odometry()
        odom.header.stamp = t.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = t.transform.rotation
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth
        with self.duration_metrics.measure('odom_publish'):
            self.odom_pub.publish(odom)

    def publish_feedback_health(self):
        """Report receipt/group/timer timing without altering odometry control."""
        now = time.monotonic()
        payload = {
            'sample_ros_stamp_ns': self.get_clock().now().nanoseconds,
            'state': self.feedback_state,
            'complete_sample_count': self.feedback_assembler.complete_sample_count,
            'complete_sample_age_sec': round(self.feedback_assembler.sample_age(now), 3),
            'receipts': self.receipt_metrics.snapshot(now),
            'wall_durations': self.duration_metrics.snapshot(now),
        }
        message = String()
        message.data = json.dumps(payload)
        with self.duration_metrics.measure('health_publish'):
            self.feedback_health_pub.publish(message)

def main(args=None):
    rclpy.init(args=args)
    node = MecanumOdomReal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
