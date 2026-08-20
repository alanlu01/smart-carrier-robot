import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SerialToROS(Node):
    def __init__(self):
        super().__init__('serial_to_ros')
        self.publisher_ = self.create_publisher(String, 'stm32_data', 10)
        self.ser = serial.Serial('/dev/stm32', 115200, timeout=1) # 🌟 換成固定綁定名稱
        self.timer = self.create_timer(0.05, self.read_serial)

    def read_serial(self):
        try:
            line = self.ser.readline().decode(errors='ignore').strip()
            if line:
                msg = String()
                msg.data = line
                self.publisher_.publish(msg)
                self.get_logger().info(f'Publish: {line}')
        except Exception as e:
            self.get_logger().error(str(e))

def main(args=None):
    rclpy.init(args=args)
    node = SerialToROS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
