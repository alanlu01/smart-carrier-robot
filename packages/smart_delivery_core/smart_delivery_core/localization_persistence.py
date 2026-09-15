"""Persist a trusted AMCL pose only within the current Linux boot."""

import json
import math
import os
import time
from pathlib import Path


CACHE_VERSION = 1
DEFAULT_BOOT_ID_PATH = "/proc/sys/kernel/random/boot_id"


def read_boot_id(path=DEFAULT_BOOT_ID_PATH):
    """Return the Linux boot ID, or ``None`` when it cannot be read."""
    try:
        value = Path(path).read_text(encoding="utf-8").strip()
    except OSError:
        return None
    return value or None


def load_cache(path):
    """Load a cache dictionary without raising on a missing/corrupt file."""
    try:
        payload = json.loads(Path(path).expanduser().read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError):
        return None
    return payload if isinstance(payload, dict) else None


def select_same_boot_pose(
    cache, boot_id, now=None, max_age_sec=3600.0
):
    """Classify startup and return a trusted pose only from this Linux boot.

    The status is one of ``new_boot``, ``same_boot_untrusted``, or
    ``same_boot_cached``.  A same-boot cache that is stale or malformed must not
    silently fall back to the fixed power-on pose because the robot may already
    be elsewhere on the map.
    """
    if (
        not boot_id
        or not isinstance(cache, dict)
        or cache.get("boot_id") != boot_id
    ):
        return "new_boot", None

    pose = cache.get("pose")
    saved_at = cache.get("saved_at")
    if not isinstance(pose, dict):
        return "same_boot_untrusted", None
    try:
        values = tuple(float(pose[key]) for key in ("x", "y", "yaw"))
        current_time = float(now if now is not None else time.time())
        age = current_time - float(saved_at)
    except (KeyError, TypeError, ValueError):
        return "same_boot_untrusted", None
    if not all(math.isfinite(value) for value in values):
        return "same_boot_untrusted", None
    if age < 0.0 or age > max(0.0, float(max_age_sec)):
        return "same_boot_untrusted", None
    return "same_boot_cached", values


def write_cache(path, boot_id, pose=None, now=None):
    """Atomically write the boot marker and optional trusted pose."""
    target = Path(path).expanduser()
    target.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "version": CACHE_VERSION,
        "boot_id": str(boot_id),
        "saved_at": float(now if now is not None else time.time()),
        "pose": None,
    }
    if pose is not None:
        x, y, yaw = (float(value) for value in pose)
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError("pose must contain finite values")
        payload["pose"] = {"x": x, "y": y, "yaw": yaw}

    temporary = target.with_name(f".{target.name}.{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, sort_keys=True),
        encoding="utf-8",
    )
    os.replace(temporary, target)
