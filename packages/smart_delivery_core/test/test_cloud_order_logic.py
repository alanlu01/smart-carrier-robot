import pytest
from nav2_simple_commander.robot_navigator import TaskResult
from power_monitor.power_status import build_slot, payload_to_slots
from smart_delivery_core.smart_delivery import (
    SlotConfirmationTracker,
    go_to_standby,
    infeasible_order_note,
    order_payload_to_order,
    recovered_slot_baseline,
    schedule_orders,
)


class _FakeLogger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass


class _FakeClock:
    def now(self):
        return self

    def to_msg(self):
        return None


class _CompletedNavigator:
    def __init__(self, result):
        self.result = result
        self.goal = None

    def get_clock(self):
        return _FakeClock()

    def get_logger(self):
        return _FakeLogger()

    def goToPose(self, goal):
        self.goal = goal

    def isTaskComplete(self):
        return True

    def getResult(self):
        return self.result


def test_standby_only_reports_arrival_after_nav2_success():
    lease = []
    failed = go_to_standby(
        _CompletedNavigator(TaskResult.FAILED),
        {"x": 0.0, "y": 0.0},
        set_navigation_active=lease.append,
    )
    assert failed is None
    assert lease == [True, False]

    lease = []
    arrived = go_to_standby(
        _CompletedNavigator(TaskResult.SUCCEEDED),
        {"x": 0.0, "y": 0.0},
        set_navigation_active=lease.append,
    )
    assert arrived is not None
    assert lease == [True, False]


def test_cloud_borrow_task_uses_backend_coordinates_and_full_slot():
    slots = payload_to_slots(
        {
            "ch1": build_slot(1, 1.1, voltage_v=10.0),
            "ch2": build_slot(2, 0.4, voltage_v=10.0),
            "ch3": build_slot(3, 0.004, voltage_v=10.0),
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

    route, deferred, _ = schedule_orders([order], {"x": 0.0, "y": 0.0}, empty_slots)

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

    route, deferred, _ = schedule_orders([order], {"x": 0.0, "y": 0.0}, slots)

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

    route, deferred, _ = schedule_orders([order], {"x": 0.0, "y": 0.0}, slots)

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


def test_multiple_orders_preserve_borrow_first_stop_distance_preference():
    orders = [
        order_payload_to_order(
            {
                "id": "near-navigation",
                "task_type": "navigation",
                "location": {"name": "near", "x": 7.5, "y": 0.0, "yaw": 0.0},
            }
        ),
        order_payload_to_order(
            {
                "id": "farther-borrow",
                "task_type": "borrow",
                "required_charge": 80,
                "location": {"name": "borrow", "x": 10.0, "y": 0.0, "yaw": 0.0},
            }
        ),
    ]
    available_slots = [
        {"slot": 1, "status": "full", "charge": 100, "sensor_ok": True},
        {"slot": 2, "status": "empty", "charge": 0, "sensor_ok": True},
        {"slot": 3, "status": "disabled", "charge": None, "sensor_ok": False},
    ]

    route, deferred, _ = schedule_orders(orders, {"x": 0.0, "y": 0.0}, available_slots)

    assert not deferred
    assert [order["task_id"] for order in route] == [
        "farther-borrow",
        "near-navigation",
    ]

    route, _, _ = schedule_orders(
        orders, {"x": 0.0, "y": 0.0}, available_slots,
        borrow_distance_weight=1.0,
    )
    assert route[0]["task_id"] == "near-navigation"


def test_september_24_batch_does_not_reward_postponing_borrow():
    orders = [
        {"task_id": "return", "type": "return", "x": 7.5, "y": 5.6},
        {
            "task_id": "borrow", "type": "borrow", "x": -2.6, "y": 4.7,
            "power_bank_id": "PB-01",
        },
    ]
    slots = [
        {"bank_id": "PB-01", "status": "full", "charge": 100},
        None,
        None,
    ]
    route, deferred, projected = schedule_orders(
        orders, {"x": 2.4289, "y": 4.9260}, slots
    )
    assert not deferred
    assert [order["task_id"] for order in route] == ["borrow", "return"]
    assert projected[route[1]["slot_number"] - 1] is not None


def test_equal_first_priority_uses_whole_route_not_claim_order():
    # Both first stops are 1m away. Choosing the original queue head produces
    # a 3.2m route; the whole-route comparison chooses the 3.1m alternative.
    orders = [
        {"task_id": "west", "type": "navigation", "x": -1.0, "y": 0.0},
        {"task_id": "east", "type": "navigation", "x": 1.0, "y": 0.0},
        {"task_id": "far-west", "type": "navigation", "x": -1.1, "y": 0.0},
    ]
    route, deferred, _ = schedule_orders(orders, {"x": 0.0, "y": 0.0}, [])
    assert not deferred
    assert [order["task_id"] for order in route] == ["east", "west", "far-west"]
    assert sum(order["distance"] for order in route) == pytest.approx(3.1)


@pytest.mark.parametrize("weight", [0.0, -0.7, float("inf"), float("nan")])
def test_scheduler_rejects_invalid_borrow_weight(weight):
    with pytest.raises(ValueError, match="borrow_distance_weight"):
        schedule_orders([], {"x": 0.0, "y": 0.0}, [], borrow_distance_weight=weight)


def test_batch_route_simulates_slot_changes_before_choosing_order():
    orders = [
        order_payload_to_order(
            {
                "id": "return-after-capacity",
                "task_type": "return",
                "location": {"name": "return", "x": 2.0, "y": 0.0, "yaw": 0.0},
            }
        ),
        order_payload_to_order(
            {
                "id": "borrow-first",
                "task_type": "borrow",
                "power_bank_id": "PB-01",
                "location": {"name": "borrow", "x": 1.0, "y": 0.0, "yaw": 0.0},
            }
        ),
    ]
    full_slots = [
        {
            "slot": number,
            "bank_id": f"PB-0{number}",
            "status": "full",
            "charge": 100,
            "sensor_ok": True,
        }
        for number in range(1, 4)
    ]

    route, deferred, _ = schedule_orders(
        orders, {"x": 0.0, "y": 0.0}, full_slots
    )

    assert not deferred
    assert [order["task_id"] for order in route] == [
        "borrow-first",
        "return-after-capacity",
    ]


def test_recovered_slot_baseline_uses_saved_snapshot():
    saved = [
        {"slot": 1, "status": "full", "sensor_ok": True, "enabled": True},
        {"slot": 2, "status": "empty", "sensor_ok": True, "enabled": True},
    ]
    task = {
        "type": "borrow",
        "slot_number": 1,
        "_slot_baseline": saved,
    }
    current = [
        {"slot": 1, "status": "empty", "sensor_ok": True, "enabled": True},
        {"slot": 2, "status": "empty", "sensor_ok": True, "enabled": True},
    ]

    baseline = recovered_slot_baseline(task, current)

    assert baseline == saved
    assert baseline is not saved


def test_recovered_slot_baseline_reconstructs_expected_pre_action_state():
    current = [
        {"slot": 1, "status": "empty", "sensor_ok": True, "enabled": True},
        {"slot": 2, "status": "full", "sensor_ok": True, "enabled": True},
    ]

    borrow = recovered_slot_baseline(
        {"type": "borrow", "slot_number": 1}, current
    )
    returned = recovered_slot_baseline(
        {"type": "return", "slot_number": 2}, current
    )

    assert borrow[0]["status"] == "full"
    assert returned[1]["status"] == "empty"
