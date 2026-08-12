import json
from typing import Any


def classify_current(current_a: float) -> str:
    current = abs(float(current_a))
    if current >= 0.4:
        return "charging"
    if current >= 0.05:
        return "almost_full"
    return "not_inserted"


def estimate_charge(current_a: float, status: str) -> int:
    current = abs(float(current_a))
    if status == "not_inserted":
        return 0
    if status == "charging":
        return min(79, max(1, round(80 * (1.0 - min(current, 1.0)))))
    ratio = (0.4 - min(max(current, 0.05), 0.4)) / 0.35
    return round(80 + ratio * 19)


def payload_to_slots(payload: str | dict[str, Any]) -> list[dict[str, Any]]:
    data = json.loads(payload) if isinstance(payload, str) else payload
    slots = []
    for number in range(1, 4):
        channel = data[f"ch{number}"]
        current = float(channel["current"])
        hardware_status = str(channel.get("status") or classify_current(current))
        status = "empty" if hardware_status == "not_inserted" else "ready"
        if hardware_status == "charging":
            status = "charging"
        slots.append(
            {
                "bank_id": None if status == "empty" else f"PB-{number:02d}",
                "status": status,
                "current": current,
                "charge": estimate_charge(current, hardware_status),
            }
        )
    return slots
