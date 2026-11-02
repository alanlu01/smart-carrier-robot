"""Bounded, odom-aligned fallback for a single-frame recovery visibility gap."""
from collections import deque
import math
import numpy as np
from smart_delivery_core.spin_evidence import ScanEvidence, aligned_spin_report


class RecoveryScanEvidence:
    """Keep raw and filtered evidence independent; never invent missing ranges."""
    def __init__(self):
        self.frames = {name: deque(maxlen=5) for name in ('scan', 'scan_filtered')}

    def observe(self, name, scan, receipt, sensor_pose, base_pose, mount, mask, sensor_z):
        frames = self.frames[name]
        source = scan.header.stamp.sec + scan.header.stamp.nanosec / 1e9
        if frames and source <= frames[-1].source_sec:
            return
        frame = ScanEvidence.from_scan(scan, receipt, sensor_pose, base_pose)
        if mask is not None:
            angles = frame.angles + mount[2]
            with np.errstate(invalid='ignore'):
                x = mount[0] + frame.ranges * np.cos(angles)
                y = mount[1] + frame.ranges * np.sin(angles)
            inside = ((x > mask[0]) & (x < mask[1]) & (y > mask[2]) & (y < mask[3]))
            if mask[4] < sensor_z < mask[5]:
                frame.ranges[inside] = np.nan
        frames.append(frame)

    def report(self, name, base_pose, now, ros_now):
        frames = self.frames[name]
        # A 73% current frame is allowed only with stronger independent coverage.
        fraction = 0. if not frames else float(np.mean(
            np.isfinite(frames[-1].ranges) & (frames[-1].ranges >= frames[-1].range_min)
            & (frames[-1].ranges <= frames[-1].range_max)))
        return aligned_spin_report(frames, base_pose, now, ros_now,
                                   math.hypot(.22, .16) + .10, 20.,
                                   current_min=.73, union_min=.80 if fraction < .75 else .75)
