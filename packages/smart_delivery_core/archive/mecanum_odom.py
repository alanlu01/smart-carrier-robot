import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros
import math
import time

class MecanumOdomNode(Node):
    def __init__(self):
        super().__init__('mecanum_odom_node')
        
        # 1. 初始化發布器 (Publisher) 與 TF 廣播器
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # 2. 機器人狀態變數 (位置與姿態)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        # 車體參數 (請根據實車尺寸修改)
        self.wheel_radius = 0.05  # 輪子半徑 5cm
        self.wheel_base = 0.20    # 前後輪軸距的一半
        self.track_width = 0.20   # 左右輪距的一半
        self.geometry_factor = self.wheel_base + self.track_width

        # 3. 設定計時器，以 20Hz (0.05秒) 的頻率執行運算
        self.last_time = time.time()
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        self.get_logger().info('麥克納姆輪里程計節點已啟動 (原型測試模式)！')

    def read_serial_data(self):
        """
        模擬從 STM32 讀取資料。
        未來這裡要換成使用 pyserial 讀取 USB 封包 (例如: readline)
        目前我們先回傳假的輪速資料 (單位: rad/s)
        """
        # 測試情境：讓車子慢慢往前走 (四個輪子皆正轉)
        w_fl = 2.0  # 左前
        w_fr = 2.0  # 右前
        w_rl = 2.0  # 左後
        w_rr = 2.0  # 右後
        return w_fl, w_fr, w_rl, w_rr

    def euler_to_quaternion(self, yaw):
        """將 2D 旋轉角度轉換為 ROS2 需要的四元數"""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def timer_callback(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # 1. 取得馬達數據 (目前是假的)
        w_fl, w_fr, w_rl, w_rr = self.read_serial_data()

        # 2. 正運動學公式 (Forward Kinematics)
        # 將四個輪子的角速度轉換為底盤的 Vx, Vy, Vth
        R = self.wheel_radius
        vx = (w_fl + w_fr + w_rl + w_rr) * (R / 4.0)
        vy = (-w_fl + w_fr + w_rl - w_rr) * (R / 4.0)
        vth = (-w_fl + w_fr - w_rl + w_rr) * (R / (4.0 * self.geometry_factor))

        # 3. 積分計算在地圖上的真實座標 (考量車頭當前朝向)
        delta_x = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
        delta_y = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt
        delta_th = vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # 4. 發布 TF 座標轉換 (odom -> base_footprint)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.euler_to_quaternion(self.th)
        self.tf_broadcaster.sendTransform(t)

        # 5. 發布 Odometry 訊息
        odom = Odometry()
        odom.header.stamp = t.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # 填寫位置
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = t.transform.rotation
        
        # 填寫速度
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth
        
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = MecanumOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
