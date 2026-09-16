import json
import math
from dataclasses import dataclass
from typing import Any

EMPTY_CURRENT_MAX_A = 0.008
EMPTY_VOLTAGE_MAX_V = 1.0
PRESENT_CURRENT_MIN_A = 0.020
FULL_CURRENT_MAX_A = 0.080
FULL_POWER_MAX_W = 5.0
READY_POWER_MAX_W = 10.0
POWER_HYSTERESIS_W = 0.5
STATE_CONFIRM_SAMPLES = 6

CANONICAL_STATUSES = {"empty", "low", "ready", "full", "unknown", "disabled"}
STATUS_ALIASES = {
    "0": "empty",
    "1": "low",
    "2": "ready",
    "3": "full",
    "EMPTY": "empty",
    "LOW": "low",
    "READY": "ready",
    "FULL": "full",
    "UNKNOWN": "unknown",
    "DISABLED": "disabled",
    "MAINTENANCE": "disabled",
    "AVAILABLE": "ready",
    "CHARGING": "low",
    "ALMOST_FULL": "ready",
    "NOT_INSERTED": "empty",
    "空": "empty",
    "低電量": "low",
    "就緒": "ready",
    "全滿": "full",
    "停用": "disabled",
    "維修中": "disabled",
}


def classify_current_status(
    current_a: float,
    *,
    empty_current_max_a: float = EMPTY_CURRENT_MAX_A,
    present_current_min_a: float = PRESENT_CURRENT_MIN_A,
    full_current_max_a: float = FULL_CURRENT_MAX_A,
) -> str:
    """Classify one sample without hysteresis.

    Values in the empty/present gap are intentionally unknown. Runtime code
    uses :class:`SlotStateTracker` to retain the last trusted state there.
    """
    current = abs(float(current_a))
    if current <= empty_current_max_a:
        return "empty"
    if current < present_current_min_a:
        return "unknown"
    if current <= full_current_max_a:
        return "full"
    return "low"


def classify_power_status(
    current_a: float,
    voltage_v: float | None,
    *,
    empty_current_max_a: float = EMPTY_CURRENT_MAX_A,
    empty_voltage_max_v: float = EMPTY_VOLTAGE_MAX_V,
    present_current_min_a: float = PRESENT_CURRENT_MIN_A,
    full_current_max_a: float = FULL_CURRENT_MAX_A,
    full_power_max_w: float = FULL_POWER_MAX_W,
    ready_power_max_w: float = READY_POWER_MAX_W,
) -> str:
    """Classify a slot from presence current and quick-charge input power.

    The quick-charge modules remain connected to the 3S vehicle battery when a
    slot is empty, so bus voltage cannot indicate power-bank presence.
    Current therefore remains responsible for empty/present detection.  Once a
    bank is present, input power determines whether it is charging, ready, or
    full.  ``empty_voltage_max_v`` and ``full_current_max_a`` remain accepted
    for payload/API compatibility with older callers.
    """
    del empty_voltage_max_v, full_current_max_a
    current = abs(float(current_a))
    if current <= empty_current_max_a:
        return "empty"
    if current < present_current_min_a:
        return "unknown"
    power_w = input_power_w(current, voltage_v)
    if power_w is None:
        return "unknown"
    if power_w <= full_power_max_w:
        return "full"
    if power_w <= ready_power_max_w:
        return "ready"
    return "low"


def input_power_w(
    current_a: float | None, voltage_v: float | None
) -> float | None:
    """Return non-negative INA3221 input power, or ``None`` if unavailable."""
    if current_a is None or voltage_v is None:
        return None
    current = float(current_a)
    voltage = float(voltage_v)
    if not math.isfinite(current) or not math.isfinite(voltage):
        return None
    return abs(current * voltage)


@dataclass
class SlotStateTracker:
    """Debounced slot-state tracker with current and power hysteresis."""

    empty_current_max_a: float = EMPTY_CURRENT_MAX_A
    present_current_min_a: float = PRESENT_CURRENT_MIN_A
    full_power_max_w: float = FULL_POWER_MAX_W
    ready_power_max_w: float = READY_POWER_MAX_W
    power_hysteresis_w: float = POWER_HYSTERESIS_W
    confirm_samples: int = STATE_CONFIRM_SAMPLES
    state: str = "unknown"
    _candidate: str | None = None
    _candidate_samples: int = 0

    def __post_init__(self):
        if self.empty_current_max_a < 0:
            raise ValueError("empty current threshold must not be negative")
        if not self.empty_current_max_a < self.present_current_min_a:
            raise ValueError("empty threshold must be below present threshold")
        if self.full_power_max_w <= 0:
            raise ValueError("full power threshold must be positive")
        if not self.full_power_max_w < self.ready_power_max_w:
            raise ValueError("full power threshold must be below ready threshold")
        if self.power_hysteresis_w < 0:
            raise ValueError("power hysteresis must not be negative")
        if self.power_hysteresis_w >= self.full_power_max_w:
            raise ValueError("power hysteresis must be below full threshold")
        if 2.0 * self.power_hysteresis_w >= (
            self.ready_power_max_w - self.full_power_max_w
        ):
            raise ValueError("power hysteresis is too large for configured thresholds")
        if self.confirm_samples < 1:
            raise ValueError("confirm_samples must be at least one")

    def update(
        self,
        current_a: float | None,
        voltage_v: float | None = None,
        *,
        sensor_ok: bool = True,
    ) -> str:
        if not sensor_ok or current_a is None or voltage_v is None:
            self.state = "unknown"
            self._candidate = None
            self._candidate_samples = 0
            return self.state

        measured = classify_power_status(
            current_a,
            voltage_v,
            empty_current_max_a=self.empty_current_max_a,
            present_current_min_a=self.present_current_min_a,
            full_power_max_w=self.full_power_max_w,
            ready_power_max_w=self.ready_power_max_w,
        )
        if measured == "unknown":
            self._candidate = None
            self._candidate_samples = 0
            return self.state

        power_w = input_power_w(current_a, voltage_v)
        if power_w is not None and measured not in {"empty", "unknown"}:
            if self.state == "full" and power_w <= (
                self.full_power_max_w + self.power_hysteresis_w
            ):
                measured = "full"
            elif self.state == "ready":
                if power_w <= self.full_power_max_w - self.power_hysteresis_w:
                    measured = "full"
                elif power_w <= self.ready_power_max_w + self.power_hysteresis_w:
                    measured = "ready"
                else:
                    measured = "low"
            elif self.state == "low" and power_w > (
                self.ready_power_max_w - self.power_hysteresis_w
            ):
                measured = "low"
        if measured == self.state:
            self._candidate = None
            self._candidate_samples = 0
            return self.state

        if measured != self._candidate:
            self._candidate = measured
            self._candidate_samples = 1
        else:
            self._candidate_samples += 1
        if self._candidate_samples >= self.confirm_samples:
            self.state = measured
            self._candidate = None
            self._candidate_samples = 0
        return self.state


def normalize_power_bank_status(status: Any, status_aliases: dict[Any, str] | None = None) -> str:
    if status_aliases and status in status_aliases:
        status = status_aliases[status]
    key = str(status).strip()
    aliases = dict(STATUS_ALIASES)
    if status_aliases:
        aliases.update({str(alias): value for alias, value in status_aliases.items()})
    normalized = aliases.get(key, aliases.get(key.upper(), key.lower()))
    if normalized not in CANONICAL_STATUSES:
        raise ValueError(f"未知的行動電源狀態：{status}")
    return normalized


def estimate_charge(current_a: float | None, status: str) -> int | None:
    status = normalize_power_bank_status(status)
    if status == "empty":
        return 0
    if status == "full":
        return 100
    return None


def build_slot(
    channel_number: int,
    current_a: float | None,
    *,
    voltage_v: float | None = None,
    sensor_ok: bool = True,
    status: str | None = None,
    charge: int | None = None,
    charge_supplied: bool = False,
    enabled: bool = True,
    empty_current_max_a: float = EMPTY_CURRENT_MAX_A,
    empty_voltage_max_v: float = EMPTY_VOLTAGE_MAX_V,
    present_current_min_a: float = PRESENT_CURRENT_MIN_A,
    full_current_max_a: float = FULL_CURRENT_MAX_A,
    full_power_max_w: float = FULL_POWER_MAX_W,
    ready_power_max_w: float = READY_POWER_MAX_W,
) -> dict[str, Any]:
    if channel_number not in (1, 2, 3):
        raise ValueError("channel_number 必須介於 1 到 3")

    valid_current = current_a if sensor_ok else None
    valid_voltage = voltage_v if sensor_ok else None
    if not enabled:
        canonical_status = "disabled"
    elif not sensor_ok or current_a is None:
        canonical_status = "unknown"
    elif status is None:
        canonical_status = classify_power_status(
            current_a,
            voltage_v,
            empty_current_max_a=empty_current_max_a,
            empty_voltage_max_v=empty_voltage_max_v,
            present_current_min_a=present_current_min_a,
            full_current_max_a=full_current_max_a,
            full_power_max_w=full_power_max_w,
            ready_power_max_w=ready_power_max_w,
        )
    else:
        canonical_status = normalize_power_bank_status(status)

    return {
        "slot": channel_number,
        "bank_id": (
            None
            if canonical_status in {"empty", "unknown", "disabled"}
            else f"PB-{channel_number:02d}"
        ),
        "status": canonical_status,
        "current": None if valid_current is None else round(float(valid_current), 3),
        "voltage": None if valid_voltage is None else round(float(valid_voltage), 3),
        "charge": (
            charge
            if charge_supplied
            else estimate_charge(valid_current, canonical_status)
        ),
        "sensor_ok": bool(sensor_ok),
        "enabled": bool(enabled),
    }


def payload_to_slots(
    payload: str | dict[str, Any], *, require_healthy: bool = False
) -> list[dict[str, Any]]:
    try:
        data = json.loads(payload) if isinstance(payload, str) else payload
        if not isinstance(data, dict):
            raise TypeError("payload 必須是 JSON object")

        slots = []
        for number in range(1, 4):
            channel = data[f"ch{number}"]
            sensor_ok = bool(channel.get("sensor_ok", True))
            enabled = bool(channel.get("enabled", True))
            current_value = channel.get("current")
            current = None if current_value is None else float(current_value)
            voltage_value = channel.get("voltage")
            voltage = None if voltage_value is None else float(voltage_value)
            slot = build_slot(
                number,
                current,
                voltage_v=voltage,
                sensor_ok=sensor_ok,
                status=channel.get("status"),
                charge=channel.get("charge"),
                charge_supplied="charge" in channel,
                enabled=enabled,
            )
            if require_healthy and enabled and (
                not sensor_ok or slot["status"] == "unknown"
            ):
                raise ValueError(f"ch{number} 感測資料無效")
            slots.append(slot)
        return slots
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise ValueError(f"無效的 power_status 數據：{exc}") from exc
