import pytest
import collections
import json
import uuid
from types import SimpleNamespace

from power_monitor.ina3221_node import (
    INA3221_DIE_ID,
    INA3221_MANUFACTURER_ID,
    INA3221Node,
    ordered_probe_addresses,
    read_device_identification,
)
from power_monitor.power_status import SlotStateTracker
from rclpy.impl.rcutils_logger import RcutilsLogger


class FakeBus:
    def read_i2c_block_data(self, address, register, length):
        assert address == 0x41
        assert length == 2
        values = {
            0xFE: [0x54, 0x49],
            0xFF: [0x32, 0x20],
        }
        return values[register]


def test_probe_addresses_prefer_configured_and_remove_duplicates():
    assert ordered_probe_addresses(0x41, [0x40, 0x41, 0x42]) == [
        0x41,
        0x40,
        0x42,
    ]


def test_probe_addresses_reject_invalid_7_bit_address():
    with pytest.raises(ValueError):
        ordered_probe_addresses(0x40, [0x80])


def test_read_device_identification_decodes_big_endian_registers():
    assert read_device_identification(FakeBus(), 0x41) == (
        INA3221_MANUFACTURER_ID,
        INA3221_DIE_ID,
    )


class RecoverableBus:
    """An in-memory I2C device; no real bus is opened by these tests."""

    def __init__(self):
        self.available = {0x40}
        self.short_read = False

    def close(self):
        pass

    def read_i2c_block_data(self, address, register, length):
        if address not in self.available:
            raise OSError('simulated unplugged sensor')
        if register in (0xFE, 0xFF):
            return [0x54, 0x49] if register == 0xFE else [0x32, 0x20]
        if self.short_read:
            return [0]
        # 0.1 A shunt / 12 V bus, all channels.
        raw = 250 << 3 if register in (1, 3, 5) else 1500 << 3
        return [raw >> 8, raw & 0xFF]


def sensor(monkeypatch):
    from power_monitor import ina3221_node

    bus = RecoverableBus()
    monkeypatch.setattr(ina3221_node.smbus2, 'SMBus', lambda _: bus)
    node = object.__new__(INA3221Node)
    logger = RcutilsLogger(name='ina_transport_test_' + uuid.uuid4().hex)
    node.get_logger = lambda: logger
    node.bus = None
    node.i2c_bus = 1
    node.i2c_address = None
    node.configured_i2c_address = 0x40
    node.i2c_probe_addresses = [0x40, 0x41]
    node.i2c_reprobe_interval = 2.0
    node.i2c_reprobe_after_failures = 2
    node.i2c_error_log_period = 5.0
    node.last_i2c_error = ''
    node.last_i2c_error_log_at = node.last_i2c_probe_at = 0.0
    node.last_i2c_recovery_log_at = node.i2c_failure_cycles = 0
    node.current_histories = [collections.deque(maxlen=6) for _ in range(3)]
    node.voltage_histories = [collections.deque(maxlen=6) for _ in range(3)]
    node.state_trackers = [SlotStateTracker() for _ in range(3)]
    node.current_lsb, node.bus_voltage_lsb = 0.0004, 0.008
    node.slot_enabled = [True, True, True]
    node.empty_current_max_a, node.empty_voltage_max_v = 0.008, 1.0
    node.present_current_min_a = 0.020
    node.full_power_max_w, node.ready_power_max_w = 5.0, 10.0
    node.vehicle_full_voltage, node.vehicle_low_voltage = 12.368, 10.5
    node.vehicle_reserve_voltage = 10.2
    node.vehicle_critical_voltage, node.vehicle_cutoff_voltage = 9.8, 9.5
    node.last_summary_log_at, node.summary_log_period = 0.0, 10.0
    node.last_summary_signature = None
    node.messages, node.batteries = [], []
    node.publisher = SimpleNamespace(publish=node.messages.append)
    node.vehicle_battery_publisher = SimpleNamespace(publish=node.batteries.append)
    return node, bus


def test_real_rclpy_logger_survives_configured_fallback_configured_address(monkeypatch):
    node, bus = sensor(monkeypatch)
    for address in (0x40, 0x41, 0x40):
        bus.available = {address}
        assert node._probe_sensor(force=True)
        assert node.i2c_address == address


def test_unplugged_sensor_publishes_unknown_and_recovers_without_restart(monkeypatch):
    node, bus = sensor(monkeypatch)
    assert node._probe_sensor(force=True)
    for _ in range(6):
        node.timer_callback()
    assert json.loads(node.messages[-1].data)['ch1']['status'] == 'full'
    bus.available = set()
    for _ in range(3):
        node.timer_callback()
        payload = json.loads(node.messages[-1].data)
        for channel in ('ch1', 'ch2', 'ch3'):
            assert payload[channel]['status'] == 'unknown'
            assert not payload[channel]['sensor_ok']
            assert payload[channel]['current'] is None
            assert payload[channel]['voltage'] is None
        assert all(not history for history in node.current_histories + node.voltage_histories)
    # Restore on the alternate address; logger and state machine both survive.
    bus.available = {0x41}
    for _ in range(6):
        node.timer_callback()
    assert node.i2c_address == 0x41
    payload = json.loads(node.messages[-1].data)
    assert payload['ch1']['sensor_ok'] and payload['ch1']['status'] == 'full'


def test_short_register_read_is_invalid_not_an_exception(monkeypatch):
    node, bus = sensor(monkeypatch)
    assert node._probe_sensor(force=True)
    bus.short_read = True
    node.timer_callback()
    assert json.loads(node.messages[-1].data)['ch1']['status'] == 'unknown'


def test_same_cycle_retry_does_not_publish_false_empty(monkeypatch):
    node, _ = sensor(monkeypatch)
    assert node._probe_sensor(force=True)
    samples = [(0.1, 12.0)] * 3
    attempts = iter((None, samples))
    node._read_cycle = lambda: next(attempts)
    node.timer_callback()
    payload = json.loads(node.messages[-1].data)
    assert payload['ch1']['sensor_ok']
    assert payload['ch1']['status'] != 'empty'
