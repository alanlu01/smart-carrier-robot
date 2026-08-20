import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import struct # 用來打包二進位數據

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        
        # 訂閱 Nav2 或避障程式發出的速度指令
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
            
        self.get_logger().info('Serial Bridge 節點已準備就緒，等待對接 STM32...')

    def cmd_vel_callback(self, msg):
        # 1. 取得 Nav2 算出來的三軸速度
        vx = msg.linear.x
        vy = msg.linear.y
        vz = msg.angular.z
        
        # 2. 這裡就是未來與隊友對接的地方！
        # 我們要把這些 float 數值打包成隊友 STM32 聽得懂的 byte 格式
        # 範例：將 vx, vy, vz 打包成 3 個 float (4 bytes each)
        # packet = struct.pack('fff', vx, vy, vz)
        
        # 目前先印出來確認 Nav2 有在發指令
        self.get_logger().info(f'收到導航指令 -> 前後:{vx:.2f}, 左右:{vy:.2f}, 旋轉:{vz:.2f}')
        
        # 3. 未來加上：ser.write(packet) 發送給 STM32

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
