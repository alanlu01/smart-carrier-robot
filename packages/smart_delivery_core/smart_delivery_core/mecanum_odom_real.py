import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros
import math
import time

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
        
        # 🌟 5. 修正後的精準車體幾何參數
        self.wheel_radius = 0.08  # 真實輪子半徑 8cm
        self.wheel_base = 0.106   # 前後輪距的一半 (21.2 / 2)
        self.track_width = 0.101  # 左右輪距的一半 (20.2 / 2)
        self.geometry_factor = self.wheel_base + self.track_width
        
        # 🌟 6. 透過實車反推的黃金轉換係數
        self.rt_to_rad_s = 0.000389
        
        # 🌟 7. 新增：自轉專屬微調係數
        # 如果實體轉 360 度，虛擬轉了 420 度，就把這裡調小 (例如 360/420 = 0.85)
        self.yaw_ratio = 1.02
        
        # 🌟 8. 新增：反向補償係數 (抹平 RViz 中的虛擬偏右現象)
        self.compensation = {
            'FL': 1.0 / 1.005,
            'RL': 1.0 / 1.0,
            'FR': 1.0 / 0.998,
            'RR': 1.0 / 0.994
        }
        
        # 🌟 9. 因為是反向偏移 (里程計少算)，所以數值要大於 1.0
        self.angular_scale = 1.10  # 建議先從 1.05 到 1.15 之間開始測

        self.last_time = time.time()
        self.timer = self.create_timer(0.05, self.update_odometry)
        
        self.get_logger().info('真實硬體里程計解析節點已成功啟動！監聽中...')

    def stm32_data_callback(self, msg):
        raw_str = msg.data
        try:
            for letter in ['A', 'B', 'C', 'D']:
                rt_label = f'{letter}_RT:'
                if rt_label in raw_str:
                    parts = raw_str.split(rt_label)
                    val_str = parts[1].split(',')[0].strip() if ',' in parts[1] else parts[1].strip()
                    raw_val = float(val_str)
                    
                    wheel_position = self.motor_mapping[letter]

                    # 🌟 乘上標準轉換係數，並且套用「反向補償」抹平偏右誤差
                    real_rad_s = raw_val * self.rt_to_rad_s * self.compensation[wheel_position]

                    if letter in ['B', 'D']:
                        self.current_speeds[wheel_position] = -real_rad_s
                    else:
                        self.current_speeds[wheel_position] = real_rad_s
        except Exception as e:
            self.get_logger().error(f'字串解析出錯: {e}，原始字串為: {raw_str}')

    def euler_to_quaternion(self, yaw):
        q = Quaternion()
        q.x = 0.0; q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def update_odometry(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
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
        self.odom_pub.publish(odom)

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
