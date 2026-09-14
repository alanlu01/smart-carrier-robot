from __future__ import annotations

from copy import deepcopy
from typing import Any

BORROWABLE_STATUSES = {"ready", "full"}
NAVIGATION_ONLY_TASK_TYPES = {"delivery", "navigation", "callbot"}


class ClaimProjectionError(ValueError):
    """A claimed task cannot be reserved against the projected inventory."""


def _task_type(task: dict[str, Any]) -> str:
    return str(task.get("task_type") or task.get("type") or "").lower()


def _borrow_candidate_key(slot: dict[str, Any]) -> tuple[int, float, int]:
    status = str(slot.get("status") or "unknown").lower()
    priority = 0 if status == "full" else 1
    charge = slot.get("charge")
    effective_charge = 100.0 if charge is None and status == "full" else float(charge or 101)
    return priority, effective_charge, int(slot["slot"])


def _canonical_slot_for_bank(bank_id: Any) -> int | None:
    """Map the robot's slot-backed PB-01..PB-03 identifiers to a slot number."""
    if bank_id is None:
        return None
    normalized = str(bank_id).strip().upper()
    if not normalized.startswith("PB-"):
        return None
    try:
        slot_number = int(normalized.removeprefix("PB-"))
    except ValueError:
        return None
    return slot_number if slot_number in {1, 2, 3} else None


def apply_claim_to_slots(slots: list[dict[str, Any]], task: dict[str, Any]) -> list[dict[str, Any]]:
    """Reserve the inventory change a claimed task will eventually make."""

    projected = deepcopy(slots)
    task_type = _task_type(task)
    if task_type in NAVIGATION_ONLY_TASK_TYPES:
        return projected

    healthy = [
        slot
        for slot in projected
        if slot.get("enabled", True)
        and slot.get("sensor_ok", False)
        and str(slot.get("status") or "unknown").lower() != "unknown"
    ]

    if task_type == "borrow":
        required_charge = int(task.get("required_charge") or 0)
        requested_bank = task.get("power_bank_id")
        candidates = []
        for slot in healthy:
            status = str(slot.get("status") or "unknown").lower()
            bank_id = slot.get("bank_id") or slot.get("id")
            charge = slot.get("charge")
            if status not in BORROWABLE_STATUSES:
                continue
            if requested_bank is not None and bank_id != requested_bank:
                continue
            if charge is None:
                if required_charge > 0 and status != "full":
                    continue
            elif float(charge) < required_charge:
                continue
            candidates.append(slot)
        if not candidates:
            # PB-01..PB-03 are slot identities on this robot, not RFID-backed
            # physical-bank identities.  If that exact healthy slot is already
            # empty, the claimed borrow has reached the physical state being
            # projected; applying the same reservation again would be a false
            # conflict while its result is still being acknowledged.
            requested_slot = _canonical_slot_for_bank(requested_bank)
            if requested_slot is not None and any(
                int(slot["slot"]) == requested_slot
                and str(slot.get("status") or "").lower() == "empty"
                for slot in healthy
            ):
                return projected
            raise ClaimProjectionError("claimed borrow task has no reservable power bank")
        selected = min(candidates, key=_borrow_candidate_key)
        selected.update(status="empty", bank_id=None, charge=0)
        return projected

    if task_type == "return":
        empty_slots = [slot for slot in healthy if str(slot.get("status") or "").lower() == "empty"]
        if not empty_slots:
            raise ClaimProjectionError("claimed return task has no reservable empty slot")
        selected = min(empty_slots, key=lambda slot: int(slot["slot"]))
        selected.update(
            status="low",
            bank_id=task.get("power_bank_id") or f"reserved-{task.get('id', 'return')}",
            charge=None,
        )
        return projected

    raise ClaimProjectionError(f"unsupported claimed task type: {task_type}")


def project_claimed_slots(
    slots: list[dict[str, Any]], tasks: list[dict[str, Any]]
) -> list[dict[str, Any]]:
    projected = deepcopy(slots)
    for task in tasks:
        projected = apply_claim_to_slots(projected, task)
    return projected
