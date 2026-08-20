import pytest

from power_monitor.power_status import build_slot, payload_to_slots
from smart_delivery_core.smart_delivery import order_payload_to_order, schedule_orders


def test_cloud_borrow_task_uses_backend_coordinates_and_ready_slot():
    slots = payload_to_slots(
        {
            "ch1": build_slot(1, 0.01),
            "ch2": build_slot(2, 0.1),
            "ch3": build_slot(3, 0.0),
        },
        require_healthy=True,
    )
    order = order_payload_to_order(
        {
            "id": "task-1",
            "task_type": "borrow",
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
    assert route[0]["selected_power_bank"]["bank_id"] == "PB-01"


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
