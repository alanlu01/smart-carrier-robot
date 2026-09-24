import pytest
from smart_carrier_api.claim_queue import (
    ClaimProjectionError,
    project_claimed_slots,
    should_close_claim_batch,
    tasks_for_batch,
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


def test_batch_closes_at_capacity_without_waiting_for_timeout():
    assert should_close_claim_batch(
        3,
        now=100.1,
        started_at=100.0,
        empty_since=None,
        max_tasks=3,
        quiet_period=1.0,
        max_wait=5.0,
    )


def test_single_task_batch_closes_after_quiet_period_not_after_three_tasks():
    assert not should_close_claim_batch(
        1,
        now=100.9,
        started_at=100.0,
        empty_since=100.0,
        max_tasks=3,
        quiet_period=1.0,
        max_wait=5.0,
    )
    assert should_close_claim_batch(
        1,
        now=101.0,
        started_at=100.0,
        empty_since=100.0,
        max_tasks=3,
        quiet_period=1.0,
        max_wait=5.0,
    )


def test_nonempty_batch_closes_at_bounded_maximum_wait_after_api_timeout():
    assert should_close_claim_batch(
        2,
        now=105.0,
        started_at=100.0,
        empty_since=None,
        max_tasks=3,
        quiet_period=1.0,
        max_wait=5.0,
    )


def test_persisted_batch_order_resolves_only_still_claimed_tasks():
    tasks = [{"id": "a"}, {"id": "b"}, {"id": "c"}]
    batch = {"id": "batch", "task_ids": ["c", "missing", "a"]}

    assert tasks_for_batch(tasks, batch) == [{"id": "c"}, {"id": "a"}]
