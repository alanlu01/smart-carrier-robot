import pytest

from power_monitor.ina3221_node import (
    INA3221_DIE_ID,
    INA3221_MANUFACTURER_ID,
    ordered_probe_addresses,
    read_device_identification,
)


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
