import pytest
from power_monitor.power_status import build_slot, payload_to_slots
from smart_delivery_core.smart_delivery import (
    SlotConfirmationTracker,
    infeasible_order_note,
    order_payload_to_order,
    schedule_orders,
)


def test_cloud_borrow_task_uses_backend_coordinates_and_ready_slot():
    slots = payload_to_slots(
        {
            "ch1": build_slot(1, 1.0),
            "ch2": build_slot(2, 0.039),
            "ch3": build_slot(3, 0.004),
        },
        require_healthy=True,
    )
    order = order_payload_to_order(
        {
            "id": "task-1",
            "task_type": "borrow",
            "power_bank_id": "PB-02",
            "required_charge": 80,
            "quantity": 1,
            "location_code": "A1",
            "location": {"name": "test", "x": 1.0, "y": 2.0, "yaw": 0.0},
        }
    )

    route, deferred, _ = schedule_orders([order], {"x": 0.0, "y": 0.0}, slots)

    assert not deferred
    assert route[0]["task_id"] == "task-1"
    assert route[0]["x"] == 1.0
    assert route[0]["power_bank_id"] == "PB-02"
    assert route[0]["selected_power_bank"]["bank_id"] == "PB-02"
    assert route[0]["slot_number"] == 2


def test_charging_slot_with_unknown_charge_cannot_meet_charge_requirement():
    slots = [
        {"slot": 1, "status": "ready", "charge": None, "sensor_ok": True},
        {"slot": 2, "status": "full", "charge": None, "sensor_ok": True},
        {"slot": 3, "status": "empty", "charge": 0, "sensor_ok": True},
    ]
    order = order_payload_to_order(
        {
            "id": "task-null-charge",
            "task_type": "borrow",
            "required_charge": 80,
            "location": {"name": "test", "x": 1.0, "y": 2.0},
        }
    )

    route, deferred, _ = schedule_orders([order], {"x": 0.0, "y": 0.0}, slots)

    assert not deferred
    assert route[0]["slot_number"] == 2


def test_physical_borrow_and_return_require_consecutive_slot_samples():
    borrow = SlotConfirmationTracker("borrow", 1, confirm_samples=2)
    occupied = [{"slot": 1, "status": "full", "sensor_ok": True}]
    empty = [{"slot": 1, "status": "empty", "sensor_ok": True}]
    assert not borrow.update(occupied)
    assert not borrow.update(empty)
    assert borrow.update(empty)

    returned = SlotConfirmationTracker("return", 1, confirm_samples=2)
    assert not returned.update(empty)
    assert not returned.update(occupied)
    assert returned.update(occupied)


def test_physical_confirmation_resets_on_sensor_error_or_bounce():
    tracker = SlotConfirmationTracker("borrow", 1, confirm_samples=2)
    empty = [{"slot": 1, "status": "empty", "sensor_ok": True}]
    unhealthy = [{"slot": 1, "status": "unknown", "sensor_ok": False}]
    occupied = [{"slot": 1, "status": "full", "sensor_ok": True}]
    assert not tracker.update(empty)
    assert not tracker.update(unhealthy)
    assert not tracker.update(empty)
    assert not tracker.update(occupied)
    assert not tracker.update(empty)
    assert tracker.update(empty)


def test_navigation_task_does_not_require_a_power_bank():
    order = order_payload_to_order(
        {
            "id": "task-2",
            "task_type": "navigation",
            "location": {"name": "test", "x": 3.0, "y": 4.0, "yaw": 1.0},
        }
    )
    empty_slots = [build_slot(number, 0.0) for number in range(1, 4)]

    route, deferred, _ = schedule_orders(
        [order], {"x": 0.0, "y": 0.0}, empty_slots
    )

    assert not deferred
    assert route[0]["type"] == "navigation"


def test_borrow_becomes_releasable_if_inventory_changes_after_claim():
    order = order_payload_to_order(
        {
            "id": "task-race",
            "task_type": "borrow",
            "required_charge": 80,
            "quantity": 1,
            "location": {"name": "test", "x": 1.0, "y": 2.0, "yaw": 0.0},
        }
    )
    slots = [
        {"slot": 1, "status": "low", "charge": 45, "sensor_ok": True},
        {"slot": 2, "status": "low", "charge": 30, "sensor_ok": True},
        {"slot": 3, "status": "empty", "charge": 0, "sensor_ok": True},
    ]

    route, deferred, _ = schedule_orders(
        [order], {"x": 0.0, "y": 0.0}, slots
    )

    assert not route
    assert deferred[0]["task_id"] == "task-race"
    assert "required charge" in infeasible_order_note(deferred[0])


def test_borrow_does_not_substitute_a_different_power_bank():
    order = order_payload_to_order(
        {
            "id": "task-selected-bank",
            "task_type": "borrow",
            "power_bank_id": "PB-02",
            "required_charge": 80,
            "quantity": 1,
            "location": {"name": "test", "x": 1.0, "y": 2.0, "yaw": 0.0},
        }
    )
    slots = [
        {
            "slot": 1,
            "bank_id": "PB-01",
            "status": "full",
            "charge": 100,
            "sensor_ok": True,
        },
        {"slot": 2, "bank_id": None, "status": "empty", "charge": 0, "sensor_ok": True},
        {"slot": 3, "bank_id": None, "status": "empty", "charge": 0, "sensor_ok": True},
    ]

    route, deferred, _ = schedule_orders(
        [order], {"x": 0.0, "y": 0.0}, slots
    )

    assert not route
    assert deferred[0]["power_bank_id"] == "PB-02"
    assert "PB-02" in infeasible_order_note(deferred[0])


def test_invalid_cloud_order_is_rejected_before_navigation():
    with pytest.raises(ValueError, match="只支援 1 顆"):
        order_payload_to_order(
            {
                "id": "task-3",
                "task_type": "borrow",
                "quantity": 2,
                "location": {"x": 1.0, "y": 2.0},
            }
        )

    with pytest.raises(ValueError, match="缺少地圖座標"):
        order_payload_to_order(
            {"id": "task-4", "task_type": "navigation", "location": {"name": "bad"}}
        )
