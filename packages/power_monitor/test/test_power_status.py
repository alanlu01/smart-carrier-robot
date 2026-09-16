import json

import pytest
from power_monitor.power_status import (
    SlotStateTracker,
    build_slot,
    classify_current_status,
    classify_power_status,
    payload_to_slots,
)


def test_current_classification():
    assert classify_current_status(1.0) == "low"
    assert classify_current_status(0.15) == "low"
    assert classify_current_status(0.039) == "full"
    assert classify_current_status(0.01) == "unknown"
    assert classify_current_status(0.002) == "empty"
    assert classify_current_status(0.0) == "empty"


def test_vehicle_bus_voltage_does_not_indicate_slot_presence():
    assert classify_power_status(0.004, 10.5) == "empty"
    assert classify_power_status(0.4, 10.0) == "full"
    assert classify_power_status(0.7, 10.0) == "ready"
    assert classify_power_status(1.0, 10.1) == "low"
    assert classify_power_status(0.4, None) == "unknown"


def test_type_c_power_threshold_boundaries():
    assert classify_power_status(0.5, 10.0) == "full"
    assert classify_power_status(0.501, 10.0) == "ready"
    assert classify_power_status(1.0, 10.0) == "ready"
    assert classify_power_status(1.001, 10.0) == "low"


def test_tracker_debounces_measured_empty_full_and_charging_states():
    tracker = SlotStateTracker(confirm_samples=3)

    assert tracker.update(0.004, 10.0) == "unknown"
    assert tracker.update(0.004, 10.0) == "unknown"
    assert tracker.update(0.004, 10.0) == "empty"
    assert tracker.update(0.4, 10.0) == "empty"
    assert tracker.update(0.4, 10.0) == "empty"
    assert tracker.update(0.4, 10.0) == "full"
    assert tracker.update(0.7, 10.0) == "full"
    assert tracker.update(0.7, 10.0) == "full"
    assert tracker.update(0.7, 10.0) == "ready"
    assert tracker.update(1.1, 10.0) == "ready"
    assert tracker.update(1.1, 10.0) == "ready"
    assert tracker.update(1.1, 10.0) == "low"


def test_tracker_holds_previous_state_inside_hysteresis_gap():
    tracker = SlotStateTracker(confirm_samples=1)
    assert tracker.update(0.004, 10.0) == "empty"
    assert tracker.update(0.012, 10.0) == "empty"
    assert tracker.update(0.4, 10.0) == "full"
    assert tracker.update(0.012, 10.0) == "full"


def test_tracker_applies_power_hysteresis_around_ready_boundaries():
    tracker = SlotStateTracker(confirm_samples=1, power_hysteresis_w=0.5)
    assert tracker.update(0.4, 10.0) == "full"
    assert tracker.update(0.53, 10.0) == "full"
    assert tracker.update(0.56, 10.0) == "ready"
    assert tracker.update(1.03, 10.0) == "ready"
    assert tracker.update(1.06, 10.0) == "low"
    assert tracker.update(0.97, 10.0) == "low"
    assert tracker.update(0.94, 10.0) == "ready"
    assert tracker.update(0.47, 10.0) == "ready"
    assert tracker.update(0.44, 10.0) == "full"


def test_tracker_fails_safe_on_sensor_error_and_reconfirms_recovery():
    tracker = SlotStateTracker(confirm_samples=2)
    tracker.update(0.4, 10.0)
    assert tracker.update(0.4, 10.0) == "full"
    assert tracker.update(None, None, sensor_ok=False) == "unknown"
    assert tracker.update(0.4, 10.0) == "unknown"
    assert tracker.update(0.4, 10.0) == "full"


def test_payload_converts_three_channels():
    payload = json.dumps(
        {
            "ch1": {"current": 1.0, "voltage": 10.1, "status": "charging", "charge": None},
            "ch2": {"current": 0.039, "voltage": 10.2, "status": "full", "charge": 100},
            "ch3": {"current": 0.0, "voltage": 0.0, "status": "not_inserted"},
        }
    )
    slots = payload_to_slots(payload)
    assert [slot["status"] for slot in slots] == ["low", "full", "empty"]
    assert [slot["bank_id"] for slot in slots] == ["PB-01", "PB-02", None]
    assert [slot["charge"] for slot in slots] == [None, 100, 0]
    assert [slot["voltage"] for slot in slots] == [10.1, 10.2, 0.0]


def test_charge_is_not_invented_from_charging_current():
    assert build_slot(1, 1.0, voltage_v=10.1)["charge"] is None
    assert build_slot(1, 0.7, voltage_v=10.0)["charge"] is None
    assert build_slot(1, 0.4, voltage_v=10.0)["charge"] == 100
    assert build_slot(1, 0.004, voltage_v=10.5)["charge"] == 0


def test_disabled_slot_is_not_reported_as_returnable_empty_slot():
    slot = build_slot(3, 0.002, voltage_v=10.6, enabled=False)
    assert slot["status"] == "disabled"
    assert slot["enabled"] is False
    assert slot["bank_id"] is None
    assert slot["charge"] is None


def test_disabled_slot_does_not_make_payload_unhealthy():
    payload = {
        "ch1": build_slot(1, 0.4, voltage_v=10.0),
        "ch2": build_slot(2, 1.1, voltage_v=10.0),
        "ch3": build_slot(3, 0.002, voltage_v=10.0, enabled=False),
    }
    slots = payload_to_slots(payload, require_healthy=True)
    assert [slot["status"] for slot in slots] == ["full", "low", "disabled"]


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
        "enabled": True,
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
