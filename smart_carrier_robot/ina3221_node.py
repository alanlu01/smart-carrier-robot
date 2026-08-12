import collections
import json

import rclpy
import smbus2
from rclpy.node import Node
from std_msgs.msg import String

from smart_carrier_robot.power_status import classify_current


class INA3221Node(Node):
    def __init__(self):
        super().__init__("ina3221_node")
        self.publisher = self.create_publisher(String, "power_status", 10)
        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_address", 0x40)
        self.declare_parameter("current_lsb", 0.0004)
        self.declare_parameter("window_size", 6)
        self.bus_number = self.get_parameter("i2c_bus").value
        self.address = self.get_parameter("i2c_address").value
        self.current_lsb = self.get_parameter("current_lsb").value
        window_size = self.get_parameter("window_size").value
        self.histories = [collections.deque(maxlen=window_size) for _ in range(3)]
        self.bus = smbus2.SMBus(self.bus_number)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info("INA3221 three-slot monitor started")

    def read_current(self, register: int) -> float:
        try:
            data = self.bus.read_i2c_block_data(self.address, register, 2)
            raw = (data[0] << 8) | data[1]
            negative = bool(raw & 0x8000)
            current = ((raw >> 3) & 0x0FFF) * self.current_lsb
            return -current if negative else current
        except OSError as exc:
            self.get_logger().error(f"INA3221 I2C read failed: {exc}")
            return 0.0

    def timer_callback(self) -> None:
        channels = {}
        for index, register in enumerate((0x01, 0x03, 0x05)):
            history = self.histories[index]
            history.append(self.read_current(register))
            average = sum(history) / len(history)
            channels[f"ch{index + 1}"] = {
                "current": round(average, 3),
                "status": classify_current(average),
            }
        message = String()
        message.data = json.dumps(channels)
        self.publisher.publish(message)

    def destroy_node(self):
        self.bus.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = INA3221Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
