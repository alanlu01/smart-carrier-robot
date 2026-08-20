import json

import pytest
from power_monitor.power_status import (
    build_slot,
    classify_current_status,
    payload_to_slots,
)


def test_current_classification():
    assert classify_current_status(0.5) == "low"
    assert classify_current_status(0.1) == "ready"
    assert classify_current_status(0.01) == "full"
    assert classify_current_status(0.0) == "empty"


def test_payload_converts_three_channels():
    payload = json.dumps(
        {
            "ch1": {"current": 0.5, "status": "charging"},
            "ch2": {"current": 0.1, "status": "almost_full"},
            "ch3": {"current": 0.0, "status": "not_inserted"},
        }
    )
    slots = payload_to_slots(payload)
    assert [slot["status"] for slot in slots] == ["low", "ready", "empty"]
    assert [slot["bank_id"] for slot in slots] == ["PB-01", "PB-02", None]


def test_unhealthy_sensor_is_not_treated_as_empty():
    slot = build_slot(1, None, sensor_ok=False)
    assert slot == {
        "slot": 1,
        "bank_id": None,
        "status": "unknown",
        "current": None,
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
