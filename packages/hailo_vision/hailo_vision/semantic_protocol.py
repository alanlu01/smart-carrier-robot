import json
from dataclasses import dataclass


@dataclass(frozen=True)
class SemanticPacket:
    detections: list
    stamp_ns: int | None
    frame_id: str
    sequence: int | None
    legacy: bool = False


def build_semantic_payload(detections, stamp_sec, stamp_nanosec, frame_id, sequence):
    """Build the timestamped JSON-compatible semantic detection envelope."""
    return {
        "stamp": {
            "sec": int(stamp_sec),
            "nanosec": int(stamp_nanosec),
        },
        "frame_id": str(frame_id),
        "sequence": int(sequence),
        "detections": list(detections),
    }


def parse_semantic_payload(payload):
    """Parse timestamped semantic data while accepting the legacy detection list."""
    data = json.loads(payload) if isinstance(payload, str) else payload
    if isinstance(data, list):
        return SemanticPacket(data, None, "", None, legacy=True)
    if not isinstance(data, dict):
        raise ValueError("semantic payload must be an object or legacy list")

    detections = data.get("detections")
    if not isinstance(detections, list):
        raise ValueError("semantic detections must be a list")

    stamp = data.get("stamp")
    stamp_ns = None
    if stamp is not None:
        if not isinstance(stamp, dict):
            raise ValueError("semantic stamp must be an object")
        sec = int(stamp.get("sec", 0))
        nanosec = int(stamp.get("nanosec", 0))
        if sec < 0 or not 0 <= nanosec < 1_000_000_000:
            raise ValueError("semantic stamp is outside the valid ROS time range")
        if sec or nanosec:
            stamp_ns = sec * 1_000_000_000 + nanosec

    sequence_value = data.get("sequence")
    sequence = None if sequence_value is None else int(sequence_value)
    return SemanticPacket(
        detections=detections,
        stamp_ns=stamp_ns,
        frame_id=str(data.get("frame_id") or ""),
        sequence=sequence,
    )


def semantic_safety_multiplier(
    age_sec,
    timeout_sec,
    stale_multiplier,
    hard_stop_timeout_sec,
):
    """Return the safety speed multiplier for the current semantic-data age."""
    if age_sec <= timeout_sec:
        return 1.0
    if age_sec <= hard_stop_timeout_sec:
        return float(stale_multiplier)
    return 0.0


def semantic_health_state(
    age_sec,
    previous_state,
    timeout_sec,
    recovery_timeout_sec,
    hard_stop_timeout_sec,
):
    """Classify semantic freshness with hysteresis around the soft timeout."""
    age_sec = max(0.0, float(age_sec))
    if not 0.0 <= recovery_timeout_sec < timeout_sec < hard_stop_timeout_sec:
        raise ValueError("semantic freshness thresholds must be ordered")
    if age_sec > hard_stop_timeout_sec:
        return "stopped"
    if previous_state in {"stale", "stopped"} and age_sec > recovery_timeout_sec:
        return "stale"
    if age_sec > timeout_sec:
        return "stale"
    return "healthy"


def scan_sync_safety_multiplier(
    current_multiplier,
    unmatched_age_sec,
    grace_sec,
    fallback_multiplier,
):
    """Debounce isolated camera/LiDAR sync misses before limiting speed."""
    current_multiplier = max(0.0, min(1.0, float(current_multiplier)))
    fallback_multiplier = max(0.0, min(1.0, float(fallback_multiplier)))
    if max(0.0, float(unmatched_age_sec)) < max(0.0, float(grace_sec)):
        return current_multiplier
    return min(current_multiplier, fallback_multiplier)


def semantic_data_age(receipt_age_sec, source_stamp_ns=None, now_ns=None):
    """Return the older age from transport receipt and the source image stamp."""
    receipt_age_sec = max(0.0, float(receipt_age_sec))
    if source_stamp_ns is None or now_ns is None:
        return receipt_age_sec
    source_age_sec = max(0.0, (int(now_ns) - int(source_stamp_ns)) / 1e9)
    return max(receipt_age_sec, source_age_sec)


def stamps_are_synchronized(first_stamp_ns, second_stamp_ns, max_skew_sec):
    if first_stamp_ns is None or second_stamp_ns is None:
        return True
    return abs(first_stamp_ns - second_stamp_ns) <= max_skew_sec * 1_000_000_000


def closest_timestamped_item(items, target_stamp_ns, max_skew_sec):
    """Return the closest ``(stamp_ns, value)`` pair inside the allowed skew."""
    if target_stamp_ns is None:
        return items[-1] if items else None
    if max_skew_sec < 0:
        raise ValueError("maximum timestamp skew must not be negative")
    closest = min(
        items,
        key=lambda item: abs(int(item[0]) - int(target_stamp_ns)),
        default=None,
    )
    if closest is None:
        return None
    if abs(int(closest[0]) - int(target_stamp_ns)) > max_skew_sec * 1_000_000_000:
        return None
    return closest
