import json

import pytest
from power_monitor.power_status import (
    build_slot,
    classify_current_status,
    classify_power_status,
    payload_to_slots,
)


def test_current_classification():
    assert classify_current_status(0.5) == "low"
    assert classify_current_status(0.1) == "ready"
    assert classify_current_status(0.01) == "full"
    assert classify_current_status(0.002) == "empty"
    assert classify_current_status(0.0) == "empty"


def test_voltage_distinguishes_empty_from_full_at_zero_current():
    assert classify_power_status(0.0, 0.0) == "empty"
    assert classify_power_status(0.002, 5.0) == "full"
    assert classify_power_status(0.1, 5.0) == "ready"
    assert classify_power_status(0.5, 5.0) == "low"


def test_payload_converts_three_channels():
    payload = json.dumps(
        {
            "ch1": {"current": 0.5, "voltage": 5.0, "status": "charging"},
            "ch2": {"current": 0.1, "voltage": 5.0, "status": "almost_full"},
            "ch3": {"current": 0.0, "voltage": 0.0, "status": "not_inserted"},
        }
    )
    slots = payload_to_slots(payload)
    assert [slot["status"] for slot in slots] == ["low", "ready", "empty"]
    assert [slot["bank_id"] for slot in slots] == ["PB-01", "PB-02", None]
    assert [slot["voltage"] for slot in slots] == [5.0, 5.0, 0.0]


def test_unhealthy_sensor_is_not_treated_as_empty():
    slot = build_slot(1, None, sensor_ok=False)
    assert slot == {
        "slot": 1,
        "bank_id": None,
        "status": "unknown",
        "current": None,
        "voltage": None,
        "charge": None,
        "sensor_ok": False,
    }

    payload = {
        "ch1": slot,
        "ch2": build_slot(2, 0.1),
        "ch3": build_slot(3, 0.0),
    }
    with pytest.raises(ValueError, match="ch1 感測資料無效"):
        payload_to_slots(payload, require_healthy=True)


def test_unhealthy_sensor_discards_stale_measurement():
    slot = build_slot(1, 0.1, voltage_v=5.0, sensor_ok=False)
    assert slot["status"] == "unknown"
    assert slot["current"] is None
    assert slot["voltage"] is None
    assert slot["charge"] is None
    assert slot["sensor_ok"] is False
