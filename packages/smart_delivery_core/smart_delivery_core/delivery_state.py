from __future__ import annotations

import json
import os
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from power_monitor.power_status import normalize_power_bank_status

OCCUPIED_STATUSES = {"low", "ready", "full"}

ALLOWED_TRANSITIONS = {
    "idle": {"task_accepted", "result_pending", "recovery_required"},
    "task_accepted": {"precheck", "result_pending"},
    "precheck": {"navigating", "result_pending"},
    "navigating": {
        "arrived",
        "waiting_localization",
        "result_pending",
        "recovery_required",
    },
    "waiting_localization": {"precheck", "navigating", "result_pending"},
    "arrived": {"waiting_action", "result_pending"},
    "waiting_action": {
        "wrong_slot",
        "verifying_action",
        "result_pending",
        "recovery_required",
    },
    "wrong_slot": {
        "waiting_action",
        "verifying_action",
        "result_pending",
        "recovery_required",
    },
    "verifying_action": {
        "waiting_action",
        "wrong_slot",
        "result_pending",
        "recovery_required",
    },
    "result_pending": {"result_acked"},
    "result_acked": {"idle"},
    "recovery_required": {"result_pending"},
}


def utc_now_iso() -> str:
    return datetime.now(UTC).isoformat()


class DeliveryStateMachine:
    def __init__(self, state: str = "idle"):
        if state not in ALLOWED_TRANSITIONS:
            raise ValueError(f"未知配送狀態：{state}")
        self.state = state

    def transition(self, next_state: str) -> str:
        if next_state == self.state:
            return self.state
        if next_state not in ALLOWED_TRANSITIONS.get(self.state, set()):
            raise ValueError(f"不允許的配送狀態轉換：{self.state} -> {next_state}")
        self.state = next_state
        return self.state


def localization_ready_for_resume(ready, ready_since, now, delay_seconds):
    """Gate new motion until localization has stayed ready for a short grace period."""
    if not ready or ready_since is None:
        return False
    return float(now) - float(ready_since) >= max(0.0, float(delay_seconds))


def interrupted_localization_order(orders):
    """Return the exact order interrupted by localization before route reordering."""
    return next(
        (
            order
            for order in orders
            if order.get("_resume_state") == "waiting_localization"
        ),
        None,
    )


def slot_presence(slot: dict[str, Any]) -> bool | None:
    if not slot.get("enabled", True):
        return None
    if not slot.get("sensor_ok", False):
        return None
    status = normalize_power_bank_status(slot.get("status"))
    if status == "empty":
        return False
    if status in OCCUPIED_STATUSES:
        return True
    return None


@dataclass(frozen=True)
class SlotVerification:
    state: str
    confirmed: bool = False
    changed_slot: int | None = None
    changed_slots: tuple[int, ...] = ()
    message: str = ""


class SlotActionVerifier:
    """Verify the expected physical transition while all other slots stay put."""

    def __init__(
        self,
        task_type: str,
        expected_slot: int,
        baseline_slots: list[dict[str, Any]],
        *,
        confirm_samples: int = 6,
    ):
        if task_type not in {"borrow", "return"}:
            raise ValueError("槽位確認只支援 borrow 或 return")
        if expected_slot not in (1, 2, 3):
            raise ValueError("expected_slot 必須介於 1 到 3")
        if confirm_samples < 1:
            raise ValueError("confirm_samples 必須大於 0")
        self.task_type = task_type
        self.expected_slot = expected_slot
        self.confirm_samples = confirm_samples
        self.matching_samples = 0
        self.baseline = {int(slot["slot"]): slot_presence(slot) for slot in baseline_slots}
        expected_baseline = self.baseline.get(expected_slot)
        required_baseline = task_type == "borrow"
        if expected_baseline is None:
            raise ValueError(f"{expected_slot} 號槽不可用或感測狀態未知")
        if expected_baseline != required_baseline:
            expected_text = "有行動電源" if required_baseline else "空槽"
            raise ValueError(f"{expected_slot} 號槽抵達時不是{expected_text}")

    def update(self, slots: list[dict[str, Any]]) -> SlotVerification:
        current = {int(slot["slot"]): slot_presence(slot) for slot in slots}
        if current.get(self.expected_slot) is None:
            self.matching_samples = 0
            return SlotVerification(
                "sensor_unavailable",
                message=f"{self.expected_slot} 號槽感測資料不可用",
            )

        changed_slots = []
        for slot_number, baseline_presence in self.baseline.items():
            if baseline_presence is None or slot_number == self.expected_slot:
                continue
            current_presence = current.get(slot_number)
            if current_presence is None:
                self.matching_samples = 0
                return SlotVerification(
                    "sensor_unavailable",
                    message=f"{slot_number} 號槽感測資料不可用",
                )
            if current_presence != baseline_presence:
                changed_slots.append(slot_number)

        if changed_slots:
            self.matching_samples = 0
            changed_text = "、".join(str(slot) for slot in changed_slots)
            return SlotVerification(
                "wrong_slot",
                changed_slot=changed_slots[0],
                changed_slots=tuple(changed_slots),
                message=(
                    f"操作槽位錯誤：請先恢復{changed_text}號槽，再操作{self.expected_slot}號槽"
                ),
            )

        desired_presence = self.task_type == "return"
        if current[self.expected_slot] != desired_presence:
            self.matching_samples = 0
            operation = "取走" if self.task_type == "borrow" else "放入"
            return SlotVerification(
                "waiting_action",
                message=f"請在{self.expected_slot}號槽{operation}行動電源",
            )

        self.matching_samples += 1
        if self.matching_samples >= self.confirm_samples:
            return SlotVerification(
                "confirmed",
                confirmed=True,
                message=f"{self.expected_slot}號槽實體操作已確認",
            )
        return SlotVerification(
            "verifying_action",
            message=(
                f"正在確認{self.expected_slot}號槽操作 "
                f"({self.matching_samples}/{self.confirm_samples})"
            ),
        )


class DeliveryJournal:
    """Small atomic JSON journal for active work and terminal result recovery."""

    def __init__(self, path: str | Path | None = None):
        if path is None:
            state_root = Path(
                os.getenv(
                    "SMART_CARRIER_STATE_DIR",
                    Path.home() / ".local" / "state" / "smart-carrier",
                )
            )
            path = state_root / "delivery.json"
        self.path = Path(path)

    def load(self) -> dict[str, Any]:
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return {}
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError(f"配送狀態檔無法讀取：{exc}") from exc
        return data if isinstance(data, dict) else {}

    def save(self, data: dict[str, Any]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(".tmp")
        temporary.write_text(
            json.dumps(data, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        os.replace(temporary, self.path)

    def clear(self) -> None:
        try:
            self.path.unlink()
        except FileNotFoundError:
            pass
