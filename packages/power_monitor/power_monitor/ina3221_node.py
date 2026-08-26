import collections
import json

import rclpy
import smbus2
from rclpy.node import Node
from std_msgs.msg import String

from power_monitor.power_status import SlotStateTracker, build_slot


def decode_shunt_current(raw_val, current_lsb=0.0004):
    """解析 INA3221 16-bit 二補數分流電流寄存器。"""
    if not 0 <= raw_val <= 0xFFFF:
        raise ValueError("raw_val 必須是 16-bit 無號整數")

    signed_raw = raw_val - 0x10000 if raw_val & 0x8000 else raw_val
    shunt_steps = signed_raw >> 3
    return shunt_steps * current_lsb


def decode_bus_voltage(raw_val, voltage_lsb=0.008):
    """Decode an INA3221 bus-voltage register."""
    if not 0 <= raw_val <= 0xFFFF:
        raise ValueError("raw_val 必須是 16-bit 無號整數")

    bus_steps = (raw_val & 0x7FFF) >> 3
    return bus_steps * voltage_lsb


class INA3221Node(Node):
    def __init__(self):
        super().__init__("ina3221_node")

        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_address", 0x40)
        self.declare_parameter("current_lsb", 0.0004)
        self.declare_parameter("bus_voltage_lsb", 0.008)
        self.declare_parameter("window_size", 6)
        self.declare_parameter("sample_period", 0.5)
        self.declare_parameter("empty_current_max_a", 0.008)
        self.declare_parameter("empty_voltage_max_v", 1.0)
        self.declare_parameter("present_current_min_a", 0.020)
        self.declare_parameter("full_current_max_a", 0.080)
        self.declare_parameter("state_confirm_samples", 6)
        self.declare_parameter("slot_enabled", [True, True, False])

        self.i2c_bus = int(self.get_parameter("i2c_bus").value)
        self.i2c_address = int(self.get_parameter("i2c_address").value)
        self.current_lsb = float(self.get_parameter("current_lsb").value)
        self.bus_voltage_lsb = float(self.get_parameter("bus_voltage_lsb").value)
        window_size = int(self.get_parameter("window_size").value)
        sample_period = float(self.get_parameter("sample_period").value)
        self.empty_current_max_a = float(
            self.get_parameter("empty_current_max_a").value
        )
        self.empty_voltage_max_v = float(
            self.get_parameter("empty_voltage_max_v").value
        )
        self.present_current_min_a = float(
            self.get_parameter("present_current_min_a").value
        )
        self.full_current_max_a = float(
            self.get_parameter("full_current_max_a").value
        )
        state_confirm_samples = int(
            self.get_parameter("state_confirm_samples").value
        )
        self.slot_enabled = [
            bool(value) for value in self.get_parameter("slot_enabled").value
        ]

        if window_size < 1:
            raise ValueError("window_size 必須大於 0")
        if sample_period <= 0:
            raise ValueError("sample_period 必須大於 0")
        if self.empty_current_max_a < 0 or self.empty_voltage_max_v < 0:
            raise ValueError("空槽門檻不得小於 0")
        if not self.empty_current_max_a < self.present_current_min_a:
            raise ValueError("空槽門檻必須小於行充存在門檻")
        if not self.present_current_min_a < self.full_current_max_a:
            raise ValueError("行充存在門檻必須小於滿電門檻")
        if state_confirm_samples < 1:
            raise ValueError("狀態確認樣本數必須大於 0")
        if len(self.slot_enabled) != 3:
            raise ValueError("slot_enabled 必須包含三個布林值")

        self.publisher = self.create_publisher(String, "power_status", 10)
        self.current_histories = [
            collections.deque(maxlen=window_size) for _ in range(3)
        ]
        self.voltage_histories = [
            collections.deque(maxlen=window_size) for _ in range(3)
        ]
        self.state_trackers = [
            SlotStateTracker(
                empty_current_max_a=self.empty_current_max_a,
                present_current_min_a=self.present_current_min_a,
                full_current_max_a=self.full_current_max_a,
                confirm_samples=state_confirm_samples,
            )
            for _ in range(3)
        ]
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

    def read_bus_voltage(self, register):
        try:
            data = self.bus.read_i2c_block_data(self.i2c_address, register, 2)
            raw_value = (data[0] << 8) | data[1]
            return decode_bus_voltage(raw_value, self.bus_voltage_lsb)
        except OSError as exc:
            self.get_logger().error(f"INA3221 I2C 讀取錯誤: {exc}")
            return None

    def timer_callback(self):
        channels = {}
        registers = ((0x01, 0x02), (0x03, 0x04), (0x05, 0x06))
        for index, (shunt_register, bus_register) in enumerate(registers):
            current_history = self.current_histories[index]
            voltage_history = self.voltage_histories[index]
            current_sample = self.read_raw_current(shunt_register)
            voltage_sample = self.read_bus_voltage(bus_register)
            sensor_ok = current_sample is not None and voltage_sample is not None
            if sensor_ok:
                current_history.append(current_sample)
                voltage_history.append(voltage_sample)

            current_average = (
                sum(current_history) / len(current_history)
                if current_history
                else None
            )
            voltage_average = (
                sum(voltage_history) / len(voltage_history) if voltage_history else None
            )
            enabled = self.slot_enabled[index]
            status = (
                self.state_trackers[index].update(
                    current_average if sensor_ok else None,
                    sensor_ok=sensor_ok,
                )
                if enabled
                else "disabled"
            )
            channels[f"ch{index + 1}"] = build_slot(
                index + 1,
                current_average if sensor_ok else None,
                voltage_v=voltage_average if sensor_ok else None,
                sensor_ok=sensor_ok,
                status=status,
                enabled=enabled,
                empty_current_max_a=self.empty_current_max_a,
                empty_voltage_max_v=self.empty_voltage_max_v,
                present_current_min_a=self.present_current_min_a,
                full_current_max_a=self.full_current_max_a,
            )

        summary = " | ".join(
            f"CH{number}: {channel['voltage']} V, "
            f"{channel['current']} A, {channel['status']}"
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
