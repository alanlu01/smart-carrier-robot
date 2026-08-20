import collections
import json

import rclpy
import smbus2
from rclpy.node import Node
from std_msgs.msg import String

from power_monitor.power_status import build_slot


def decode_shunt_current(raw_val, current_lsb=0.0004):
    """解析 INA3221 16-bit 二補數分流電流寄存器。"""
    if not 0 <= raw_val <= 0xFFFF:
        raise ValueError("raw_val 必須是 16-bit 無號整數")

    signed_raw = raw_val - 0x10000 if raw_val & 0x8000 else raw_val
    shunt_steps = signed_raw >> 3
    return shunt_steps * current_lsb


class INA3221Node(Node):
    def __init__(self):
        super().__init__("ina3221_node")

        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_address", 0x40)
        self.declare_parameter("current_lsb", 0.0004)
        self.declare_parameter("window_size", 6)
        self.declare_parameter("sample_period", 0.5)

        self.i2c_bus = int(self.get_parameter("i2c_bus").value)
        self.i2c_address = int(self.get_parameter("i2c_address").value)
        self.current_lsb = float(self.get_parameter("current_lsb").value)
        window_size = int(self.get_parameter("window_size").value)
        sample_period = float(self.get_parameter("sample_period").value)

        if window_size < 1:
            raise ValueError("window_size 必須大於 0")
        if sample_period <= 0:
            raise ValueError("sample_period 必須大於 0")

        self.publisher = self.create_publisher(String, "power_status", 10)
        self.histories = [collections.deque(maxlen=window_size) for _ in range(3)]
        self.bus = smbus2.SMBus(self.i2c_bus)
        self.timer = self.create_timer(sample_period, self.timer_callback)
        self.get_logger().info("INA3221 電源監控與行充狀態節點已啟動")

    def read_raw_current(self, register):
        try:
            data = self.bus.read_i2c_block_data(self.i2c_address, register, 2)
            raw_value = (data[0] << 8) | data[1]
            return decode_shunt_current(raw_value, self.current_lsb)
        except OSError as exc:
            self.get_logger().error(f"INA3221 I2C 讀取錯誤: {exc}")
            return None

    def timer_callback(self):
        channels = {}
        for index, register in enumerate((0x01, 0x03, 0x05)):
            history = self.histories[index]
            sample = self.read_raw_current(register)
            sensor_ok = sample is not None
            if sensor_ok:
                history.append(sample)

            average = sum(history) / len(history) if history else None
            channels[f"ch{index + 1}"] = build_slot(
                index + 1,
                average,
                sensor_ok=sensor_ok,
            )

        summary = " | ".join(
            f"CH{number}: {channel['current']} A, {channel['status']}"
            for number, channel in enumerate(channels.values(), start=1)
        )
        self.get_logger().info(summary)

        message = String()
        message.data = json.dumps(channels, ensure_ascii=False)
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
