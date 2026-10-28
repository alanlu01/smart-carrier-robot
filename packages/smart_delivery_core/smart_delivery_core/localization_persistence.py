"""Persist a trusted AMCL pose only within the current Linux boot."""

import json
import hashlib
import math
import os
import time
import threading
from contextlib import nullcontext
from pathlib import Path
import yaml

from smart_delivery_core.pipeline_health import DurationMetrics


CACHE_VERSION = 1
DEFAULT_BOOT_ID_PATH = "/proc/sys/kernel/random/boot_id"


def map_file_identity(path):
    """Bind cached poses to the YAML plus actual raster, not just a file name."""
    source = Path(path)
    content = source.read_bytes()
    metadata = yaml.safe_load(content)
    raster = Path(metadata['image'])
    if not raster.is_absolute():
        raster = source.parent/raster
    digest = hashlib.sha256(content)
    digest.update(raster.read_bytes())
    return digest.hexdigest()


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
    cache, boot_id, now=None, max_age_sec=3600.0, map_identity=None
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
    if map_identity is not None and cache.get('map_identity') != map_identity:
        return 'same_boot_untrusted', None
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


def write_cache(path, boot_id, pose=None, now=None, map_identity=None, metrics=None):
    """Atomically write the boot marker and optional trusted pose."""
    target = Path(path).expanduser()
    payload = {
        "version": CACHE_VERSION,
        "boot_id": str(boot_id),
        "saved_at": float(now if now is not None else time.time()),
        "pose": None,
        "map_identity": map_identity,
    }
    if pose is not None:
        x, y, yaw = (float(value) for value in pose)
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError("pose must contain finite values")
        payload["pose"] = {"x": x, "y": y, "yaw": yaw}

    measure = metrics.measure if metrics is not None else lambda _name: nullcontext()
    temporary = target.with_name(f".{target.name}.{os.getpid()}.tmp")
    with measure('serialize'):
        content = json.dumps(payload, ensure_ascii=False, sort_keys=True)
    with measure('file_write'):
        target.parent.mkdir(parents=True, exist_ok=True)
        temporary.write_text(content, encoding="utf-8")
    with measure('atomic_replace'):
        os.replace(temporary, target)


class LatestPoseWriter:
    """One I/O worker and one replaceable pending pose; no ROS or control calls."""

    def __init__(self, path, boot_id, map_identity, writer=write_cache):
        self.path, self.boot_id, self.map_identity = path, boot_id, map_identity
        self.writer = writer
        self.condition = threading.Condition()
        self.pending = None
        self.closed = self.active = False
        self.submitted = self.replaced = self.completed = self.failures = 0
        self.last_error = ''
        self.metrics = DurationMetrics()
        self.last_durations = {}
        self.active_since = self.metrics_at = None
        self.thread = threading.Thread(target=self._run, name='trusted-pose-cache', daemon=True)
        self.thread.start()

    def submit(self, pose, observed_at):
        """Freeze the original observation, never the eventual disk-write time."""
        snapshot = tuple(float(v) for v in pose)
        if (len(snapshot) != 3 or not all(math.isfinite(v) for v in snapshot)
                or not math.isfinite(observed_at) or observed_at <= 0):
            return False
        with self.condition:
            if self.closed:
                return False
            self.replaced += self.pending is not None
            self.pending = (snapshot, observed_at)
            self.submitted += 1
            self.condition.notify()
        return True

    def _run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.closed or self.pending is not None)
                if self.closed:
                    return
                pose, observed_at = self.pending
                self.pending, self.active = None, True
                self.active_since = time.monotonic()
            error = ''
            try:
                with self.metrics.measure('total_write'):
                    self.writer(self.path, self.boot_id, pose, now=observed_at,
                                map_identity=self.map_identity, metrics=self.metrics)
            except Exception as exc:
                error = f'{type(exc).__name__}: {exc}'
            with self.condition:
                self.last_error = error
                self.failures += bool(error)
                self.completed += not bool(error)
                self.metrics_at = time.monotonic()
                self.last_durations = self.metrics.snapshot(self.metrics_at)
                self.active = False
                self.condition.notify_all()

    def snapshot(self):
        with self.condition:
            now = time.monotonic()
            durations = {name: dict(value, peak_age_sec=round(
                value['peak_age_sec'] + max(0.,now-self.metrics_at), 3))
                for name,value in self.last_durations.items()}
            return dict(active=self.active, pending=self.pending is not None,
                        active_elapsed_sec=(round(now-self.active_since,6) if self.active else 0.),
                        submitted=self.submitted, replaced=self.replaced,
                        completed=self.completed, failures=self.failures,
                        last_error=self.last_error, wall_durations=durations,
                        closed=self.closed)

    def close(self, timeout=.5):
        """Do not hang shutdown on storage; daemon exits with this process."""
        with self.condition:
            self.closed, self.pending = True, None
            self.condition.notify_all()
        self.thread.join(timeout=timeout)
        return not self.thread.is_alive()
