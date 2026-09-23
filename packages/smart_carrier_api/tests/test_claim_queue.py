import pytest
from smart_carrier_api.claim_queue import (
    ClaimProjectionError,
    dispatchable_claimed_tasks,
    project_claimed_slots,
)


def slots():
    return [
        {
            "slot": 1,
            "bank_id": "PB-01",
            "status": "full",
            "charge": 100,
            "sensor_ok": True,
            "enabled": True,
        },
        {
            "slot": 2,
            "bank_id": "PB-02",
            "status": "ready",
            "charge": 85,
            "sensor_ok": True,
            "enabled": True,
        },
        {
            "slot": 3,
            "bank_id": None,
            "status": "disabled",
            "charge": None,
            "sensor_ok": False,
            "enabled": False,
        },
    ]


def test_two_claimed_borrows_reserve_two_distinct_power_banks():
    original = slots()
    projected = project_claimed_slots(
        original,
        [
            {"id": "task-1", "task_type": "borrow", "required_charge": 80},
            {"id": "task-2", "task_type": "borrow", "required_charge": 80},
        ],
    )

    assert [slot["status"] for slot in projected] == ["empty", "empty", "disabled"]
    assert [slot["status"] for slot in original] == ["full", "ready", "disabled"]


def test_third_borrow_cannot_reuse_an_already_reserved_power_bank():
    with pytest.raises(ClaimProjectionError, match="no reservable power bank"):
        project_claimed_slots(
            slots(),
            [
                {"id": "task-1", "task_type": "borrow"},
                {"id": "task-2", "task_type": "borrow"},
                {"id": "task-3", "task_type": "borrow"},
            ],
        )


def test_completed_slot_backed_borrow_is_not_reserved_twice():
    current = slots()
    current[0].update(status="empty", bank_id=None, charge=0)

    projected = project_claimed_slots(
        current,
        [{"id": "task-1", "task_type": "borrow", "power_bank_id": "PB-01"}],
    )

    assert projected == current


def test_missing_non_slot_bank_remains_a_projection_error():
    with pytest.raises(ClaimProjectionError, match="no reservable power bank"):
        project_claimed_slots(
            slots(),
            [{"id": "task-1", "task_type": "borrow", "power_bank_id": "RFID-9"}],
        )


def test_borrow_can_create_capacity_for_a_later_return():
    occupied = slots()
    occupied[2].update(bank_id="PB-03", status="low", charge=None, sensor_ok=True, enabled=True)
    projected = project_claimed_slots(
        occupied,
        [
            {"id": "borrow", "task_type": "borrow", "power_bank_id": "PB-01"},
            {"id": "return", "task_type": "return"},
        ],
    )

    assert all(slot["status"] != "empty" for slot in projected)
    assert projected[0]["bank_id"].startswith("reserved-return")


def test_navigation_claim_does_not_change_projected_inventory():
    original = slots()
    projected = project_claimed_slots(original, [{"id": "nav", "task_type": "navigation"}])

    assert projected == original
    assert projected is not original


def test_only_fifo_head_is_dispatchable_even_when_later_task_has_never_published():
    tasks = [{"id": "older"}, {"id": "newer"}, {"id": "newest"}]

    assert dispatchable_claimed_tasks(
        tasks,
        {"older": 98.0},
        now=100.0,
        interval=5.0,
    ) == []
    assert dispatchable_claimed_tasks(
        tasks,
        {"older": 90.0},
        now=100.0,
        interval=5.0,
    ) == [{"id": "older"}]


def test_next_fifo_task_is_immediately_dispatchable_after_head_is_removed():
    remaining = [{"id": "second"}, {"id": "third"}]

    assert dispatchable_claimed_tasks(
        remaining,
        {"first": 100.0},
        now=100.0,
        interval=5.0,
    ) == [{"id": "second"}]
