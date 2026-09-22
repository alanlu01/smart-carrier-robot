import collections
import json
import time

import rclpy
from rclpy.node import Node
import smbus2

from power_monitor.power_status import (
    build_slot,
    build_vehicle_battery,
    input_power_w,
    SlotStateTracker,
)
from std_msgs.msg import String

INA3221_MANUFACTURER_ID = 0x5449
INA3221_DIE_ID = 0x3220


def ordered_probe_addresses(preferred, candidates):
    """Return valid 7-bit I2C addresses with the configured address first."""
    addresses = []
    for value in (preferred, *candidates):
        address = int(value)
        if not 0x03 <= address <= 0x77:
            raise ValueError(f"無效的 7-bit I2C 位址: {address}")
        if address not in addresses:
            addresses.append(address)
    return addresses


def read_device_identification(bus, address):
    """Read the INA3221 manufacturer and die identification registers."""
    manufacturer = bus.read_i2c_block_data(address, 0xFE, 2)
    die = bus.read_i2c_block_data(address, 0xFF, 2)
    return (
        (manufacturer[0] << 8) | manufacturer[1],
        (die[0] << 8) | die[1],
    )


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
        self.declare_parameter("auto_detect_i2c_address", True)
        self.declare_parameter("i2c_probe_addresses", [0x40, 0x41, 0x42, 0x43])
        self.declare_parameter("i2c_reprobe_interval_sec", 2.0)
        self.declare_parameter("i2c_reprobe_after_failures", 2)
        self.declare_parameter("i2c_error_log_period_sec", 5.0)
        self.declare_parameter("current_lsb", 0.0004)
        self.declare_parameter("bus_voltage_lsb", 0.008)
        self.declare_parameter("window_size", 6)
        self.declare_parameter("sample_period", 0.5)
        self.declare_parameter("empty_current_max_a", 0.008)
        self.declare_parameter("empty_voltage_max_v", 1.0)
        self.declare_parameter("present_current_min_a", 0.020)
        self.declare_parameter("full_power_max_w", 5.0)
        self.declare_parameter("ready_power_max_w", 10.0)
        self.declare_parameter("power_hysteresis_w", 0.5)
        self.declare_parameter("state_confirm_samples", 6)
        self.declare_parameter("summary_log_period_sec", 10.0)
        self.declare_parameter("vehicle_full_voltage_v", 12.368)
        self.declare_parameter("vehicle_low_voltage_v", 10.5)
        self.declare_parameter("vehicle_reserve_voltage_v", 10.2)
        self.declare_parameter("vehicle_critical_voltage_v", 9.8)
        self.declare_parameter("vehicle_cutoff_voltage_v", 9.5)
        self.declare_parameter("slot_enabled", [True, True, True])

        self.i2c_bus = int(self.get_parameter("i2c_bus").value)
        self.configured_i2c_address = int(self.get_parameter("i2c_address").value)
        self.auto_detect_i2c_address = bool(
            self.get_parameter("auto_detect_i2c_address").value
        )
        configured_probe_addresses = list(
            self.get_parameter("i2c_probe_addresses").value
        )
        self.i2c_probe_addresses = ordered_probe_addresses(
            self.configured_i2c_address,
            configured_probe_addresses if self.auto_detect_i2c_address else [],
        )
        self.i2c_reprobe_interval = max(
            0.5, float(self.get_parameter("i2c_reprobe_interval_sec").value)
        )
        self.i2c_reprobe_after_failures = max(
            1, int(self.get_parameter("i2c_reprobe_after_failures").value)
        )
        self.i2c_error_log_period = max(
            1.0, float(self.get_parameter("i2c_error_log_period_sec").value)
        )
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
        self.full_power_max_w = float(
            self.get_parameter("full_power_max_w").value
        )
        self.ready_power_max_w = float(
            self.get_parameter("ready_power_max_w").value
        )
        self.power_hysteresis_w = float(
            self.get_parameter("power_hysteresis_w").value
        )
        state_confirm_samples = int(
            self.get_parameter("state_confirm_samples").value
        )
        self.summary_log_period = max(
            1.0, float(self.get_parameter("summary_log_period_sec").value)
        )
        self.vehicle_full_voltage = float(
            self.get_parameter("vehicle_full_voltage_v").value
        )
        self.vehicle_low_voltage = float(
            self.get_parameter("vehicle_low_voltage_v").value
        )
        self.vehicle_reserve_voltage = float(
            self.get_parameter("vehicle_reserve_voltage_v").value
        )
        self.vehicle_critical_voltage = float(
            self.get_parameter("vehicle_critical_voltage_v").value
        )
        self.vehicle_cutoff_voltage = float(
            self.get_parameter("vehicle_cutoff_voltage_v").value
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
        if self.full_power_max_w <= 0:
            raise ValueError("滿電功率門檻必須大於 0")
        if not self.full_power_max_w < self.ready_power_max_w:
            raise ValueError("滿電功率門檻必須小於就緒功率門檻")
        if self.power_hysteresis_w < 0:
            raise ValueError("功率遲滯不得小於 0")
        if state_confirm_samples < 1:
            raise ValueError("狀態確認樣本數必須大於 0")
        if len(self.slot_enabled) != 3:
            raise ValueError("slot_enabled 必須包含三個布林值")

        self.publisher = self.create_publisher(String, "power_status", 10)
        self.vehicle_battery_publisher = self.create_publisher(
            String, "vehicle_battery_status", 10
        )
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
                full_power_max_w=self.full_power_max_w,
                ready_power_max_w=self.ready_power_max_w,
                power_hysteresis_w=self.power_hysteresis_w,
                confirm_samples=state_confirm_samples,
            )
            for _ in range(3)
        ]
        self.bus = None
        self.i2c_address = None
        self.last_i2c_error = ""
        self.last_i2c_error_log_at = 0.0
        self.last_i2c_probe_at = 0.0
        self.i2c_failure_cycles = 0
        self.last_i2c_recovery_log_at = 0.0
        self.last_summary_log_at = 0.0
        self.last_summary_signature = None
        self._probe_sensor(force=True)
        self.timer = self.create_timer(sample_period, self.timer_callback)
        self.get_logger().info("INA3221 電源監控與行充狀態節點已啟動")

    def _reopen_bus(self):
        if self.bus is not None:
            try:
                self.bus.close()
            except OSError:
                pass
        try:
            self.bus = smbus2.SMBus(self.i2c_bus)
            return True
        except OSError as exc:
            self.bus = None
            self.last_i2c_error = str(exc)
            return False

    def _log_i2c_error(self, message):
        now = time.monotonic()
        if now - self.last_i2c_error_log_at >= self.i2c_error_log_period:
            self.get_logger().error(message)
            self.last_i2c_error_log_at = now

    def _probe_sensor(self, *, force=False):
        now = time.monotonic()
        if not force and now - self.last_i2c_probe_at < self.i2c_reprobe_interval:
            return False
        self.last_i2c_probe_at = now
        if not self._reopen_bus():
            self._log_i2c_error(
                f"無法開啟 I2C bus {self.i2c_bus}: {self.last_i2c_error}"
            )
            return False

        previous_address = self.i2c_address
        for address in self.i2c_probe_addresses:
            try:
                manufacturer_id, die_id = read_device_identification(
                    self.bus, address
                )
            except (OSError, IndexError) as exc:
                self.last_i2c_error = str(exc)
                continue
            if (
                manufacturer_id != INA3221_MANUFACTURER_ID
                or die_id != INA3221_DIE_ID
            ):
                continue

            self.i2c_address = address
            self.i2c_failure_cycles = 0
            if address != previous_address:
                for history in self.current_histories + self.voltage_histories:
                    history.clear()
                level = (
                    self.get_logger().warning
                    if address != self.configured_i2c_address
                    else self.get_logger().info
                )
                level(
                    "已偵測 INA3221："
                    f"bus={self.i2c_bus}, address=0x{address:02X}"
                )
            return True

        self.i2c_address = None
        addresses = ", ".join(
            f"0x{address:02X}" for address in self.i2c_probe_addresses
        )
        self._log_i2c_error(
            f"找不到 INA3221（bus={self.i2c_bus}, probes={addresses}）；"
            "將持續重新探測"
        )
        return False

    def _read_register(self, register):
        if self.bus is None or self.i2c_address is None:
            return None
        try:
            data = self.bus.read_i2c_block_data(self.i2c_address, register, 2)
        except OSError as exc:
            self.last_i2c_error = str(exc)
            return None
        return (data[0] << 8) | data[1]

    def read_raw_current(self, register):
        raw_value = self._read_register(register)
        if raw_value is None:
            return None
        return decode_shunt_current(raw_value, self.current_lsb)

    def read_bus_voltage(self, register):
        raw_value = self._read_register(register)
        if raw_value is None:
            return None
        return decode_bus_voltage(raw_value, self.bus_voltage_lsb)

    def _read_cycle(self):
        """Read one atomic three-channel sample, discarding partial cycles."""
        samples = []
        registers = ((0x01, 0x02), (0x03, 0x04), (0x05, 0x06))
        for shunt_register, bus_register in registers:
            current_sample = self.read_raw_current(shunt_register)
            voltage_sample = self.read_bus_voltage(bus_register)
            if current_sample is None or voltage_sample is None:
                return None
            samples.append((current_sample, voltage_sample))
        return samples

    def _retry_failed_cycle(self):
        """Reopen, identify, and retry once before exposing an I2C glitch."""
        if not self._probe_sensor(force=True):
            return None
        samples = self._read_cycle()
        if samples is not None:
            now = time.monotonic()
            if now - self.last_i2c_recovery_log_at >= self.i2c_error_log_period:
                self.get_logger().warning(
                    "INA3221 短暫讀取失敗，重新開啟 I2C 後已於同一週期恢復"
                )
                self.last_i2c_recovery_log_at = now
        return samples

    def timer_callback(self):
        if self.i2c_address is None:
            self._probe_sensor()

        samples = self._read_cycle()
        first_error = self.last_i2c_error
        if samples is None:
            samples = self._retry_failed_cycle()

        channels = {}
        cycle_failed = samples is None
        for index in range(3):
            current_history = self.current_histories[index]
            voltage_history = self.voltage_histories[index]
            sensor_ok = samples is not None
            if sensor_ok:
                current_sample, voltage_sample = samples[index]
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
                    voltage_average if sensor_ok else None,
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
                full_power_max_w=self.full_power_max_w,
                ready_power_max_w=self.ready_power_max_w,
            )

        if cycle_failed:
            self.i2c_failure_cycles += 1
            address = (
                "none"
                if self.i2c_address is None
                else f"0x{self.i2c_address:02X}"
            )
            self._log_i2c_error(
                "INA3221 I2C 讀取失敗："
                f"bus={self.i2c_bus}, address={address}, "
                f"error={self.last_i2c_error or first_error or 'unknown'}"
            )
            if self.i2c_failure_cycles >= self.i2c_reprobe_after_failures:
                self.i2c_address = None
        else:
            self.i2c_failure_cycles = 0

        summary_parts = []
        for number, channel in enumerate(channels.values(), start=1):
            power_w = input_power_w(channel["current"], channel["voltage"])
            power_text = "unknown" if power_w is None else f"{power_w:.2f} W"
            summary_parts.append(
                f"CH{number}: {channel['voltage']} V, "
                f"{channel['current']} A, {power_text}, {channel['status']}"
            )
        vehicle_battery = build_vehicle_battery(
            [channel["voltage"] for channel in channels.values()],
            full_voltage_v=self.vehicle_full_voltage,
            low_voltage_v=self.vehicle_low_voltage,
            reserve_voltage_v=self.vehicle_reserve_voltage,
            critical_voltage_v=self.vehicle_critical_voltage,
            cutoff_voltage_v=self.vehicle_cutoff_voltage,
        )
        summary_parts.append(
            "Vehicle: "
            f"{vehicle_battery['voltage']} V, {vehicle_battery['status']}"
        )
        summary = " | ".join(summary_parts)
        summary_signature = (
            tuple(channel["status"] for channel in channels.values()),
            vehicle_battery["status"],
            cycle_failed,
        )
        now = time.monotonic()
        if (
            summary_signature != self.last_summary_signature
            or now - self.last_summary_log_at >= self.summary_log_period
        ):
            if vehicle_battery["status"] in {"critical", "cutoff"}:
                self.get_logger().error(summary)
            elif vehicle_battery["status"] in {"low", "reserve"}:
                self.get_logger().warning(summary)
            else:
                self.get_logger().info(summary)
            self.last_summary_signature = summary_signature
            self.last_summary_log_at = now

        message = String()
        payload = dict(channels)
        payload["vehicle_battery"] = vehicle_battery
        message.data = json.dumps(payload, ensure_ascii=False)
        self.publisher.publish(message)

        battery_message = String()
        battery_message.data = json.dumps(vehicle_battery, ensure_ascii=False)
        self.vehicle_battery_publisher.publish(battery_message)

    def destroy_node(self):
        if self.bus is not None:
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
