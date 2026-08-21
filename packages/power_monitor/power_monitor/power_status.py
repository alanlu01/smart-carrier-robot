import json
from typing import Any

EMPTY_CURRENT_MAX_A = 0.005
EMPTY_VOLTAGE_MAX_V = 1.0
READY_CURRENT_MIN_A = 0.1
LOW_CURRENT_MIN_A = 0.4

CANONICAL_STATUSES = {"empty", "low", "ready", "full", "unknown"}
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
    "AVAILABLE": "ready",
    "CHARGING": "low",
    "ALMOST_FULL": "ready",
    "NOT_INSERTED": "empty",
    "空": "empty",
    "低電量": "low",
    "就緒": "ready",
    "全滿": "full",
}


def classify_current_status(
    current_a: float,
    *,
    empty_current_max_a: float = EMPTY_CURRENT_MAX_A,
    ready_current_min_a: float = READY_CURRENT_MIN_A,
    low_current_min_a: float = LOW_CURRENT_MIN_A,
) -> str:
    """Convert INA3221 current into the canonical slot state."""
    current = abs(float(current_a))
    if current <= empty_current_max_a:
        return "empty"
    if current >= low_current_min_a:
        return "low"
    if current >= ready_current_min_a:
        return "ready"
    return "full"


def classify_power_status(
    current_a: float,
    voltage_v: float | None,
    *,
    empty_current_max_a: float = EMPTY_CURRENT_MAX_A,
    empty_voltage_max_v: float = EMPTY_VOLTAGE_MAX_V,
    ready_current_min_a: float = READY_CURRENT_MIN_A,
    low_current_min_a: float = LOW_CURRENT_MIN_A,
) -> str:
    """Classify a slot using bus voltage for presence and current for charge state.

    A powered slot with almost no current is a fully charged power bank, while an
    unpowered slot is empty. Older payloads without voltage retain the
    current-only fallback.
    """
    if voltage_v is None:
        return classify_current_status(
            current_a,
            empty_current_max_a=empty_current_max_a,
            ready_current_min_a=ready_current_min_a,
            low_current_min_a=low_current_min_a,
        )

    if abs(float(voltage_v)) <= empty_voltage_max_v:
        return "empty"

    current = abs(float(current_a))
    if current <= empty_current_max_a:
        return "empty"
    if current >= low_current_min_a:
        return "low"
    if current >= ready_current_min_a:
        return "ready"
    return "full"


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
    if status == "unknown" or current_a is None:
        return None
    current = abs(float(current_a))
    if status == "empty":
        return 0
    if status == "full":
        return 100
    if status == "low":
        return min(79, max(1, round(80 * (1.0 - min(current, 1.0)))))

    bounded_current = min(max(current, READY_CURRENT_MIN_A), LOW_CURRENT_MIN_A)
    ready_ratio = (LOW_CURRENT_MIN_A - bounded_current) / (
        LOW_CURRENT_MIN_A - READY_CURRENT_MIN_A
    )
    return round(80 + ready_ratio * 19)


def build_slot(
    channel_number: int,
    current_a: float | None,
    *,
    voltage_v: float | None = None,
    sensor_ok: bool = True,
    status: str | None = None,
    empty_current_max_a: float = EMPTY_CURRENT_MAX_A,
    empty_voltage_max_v: float = EMPTY_VOLTAGE_MAX_V,
    ready_current_min_a: float = READY_CURRENT_MIN_A,
    low_current_min_a: float = LOW_CURRENT_MIN_A,
) -> dict[str, Any]:
    if channel_number not in (1, 2, 3):
        raise ValueError("channel_number 必須介於 1 到 3")

    valid_current = current_a if sensor_ok else None
    valid_voltage = voltage_v if sensor_ok else None
    if not sensor_ok or current_a is None:
        canonical_status = "unknown"
    elif status is None:
        canonical_status = classify_power_status(
            current_a,
            voltage_v,
            empty_current_max_a=empty_current_max_a,
            empty_voltage_max_v=empty_voltage_max_v,
            ready_current_min_a=ready_current_min_a,
            low_current_min_a=low_current_min_a,
        )
    else:
        canonical_status = normalize_power_bank_status(status)

    return {
        "slot": channel_number,
        "bank_id": (
            None
            if canonical_status in {"empty", "unknown"}
            else f"PB-{channel_number:02d}"
        ),
        "status": canonical_status,
        "current": None if valid_current is None else round(float(valid_current), 3),
        "voltage": None if valid_voltage is None else round(float(valid_voltage), 3),
        "charge": estimate_charge(valid_current, canonical_status),
        "sensor_ok": bool(sensor_ok),
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
            )
            if require_healthy and (not sensor_ok or slot["status"] == "unknown"):
                raise ValueError(f"ch{number} 感測資料無效")
            slots.append(slot)
        return slots
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise ValueError(f"無效的 power_status 數據：{exc}") from exc
