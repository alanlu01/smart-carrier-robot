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
