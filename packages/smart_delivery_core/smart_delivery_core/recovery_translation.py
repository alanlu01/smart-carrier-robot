"""Map-independent, fail-closed geometry for one supervised recovery sidestep."""

import math


def lateral_progress(anchor, pose, direction):
    """Project displacement in the ORIGINAL body frame, never an AMCL frame."""
    dx, dy = pose[0] - anchor[0], pose[1] - anchor[1]
    c, s = math.cos(anchor[2]), math.sin(anchor[2])
    yaw_error = math.atan2(math.sin(pose[2] - anchor[2]), math.cos(pose[2] - anchor[2]))
    return direction * (-s * dx + c * dy), c * dx + s * dy, yaw_error


def swept_scan_clear(scan, sensor_pose, direction, distance, half_x, half_y,
                     margin=0.08, max_gap=math.radians(4)):
    """Require measured free rays across the entire added swept rectangle.

    Transform rays into base_footprint (including a backwards-mounted laser).
    NaN/Inf/occluded/missing rays are NOT evidence of free space. The rectangle
    includes a margin around the full footprint, not merely its centre line.
    This does not claim to detect transparent glass invisible to the lidar.
    """
    if direction not in (-1, 1) or distance <= 0 or not scan.ranges:
        return False
    sx, sy, yaw = sensor_pose
    bounds = (-half_x - margin, half_x + margin,
              half_y if direction > 0 else -half_y - distance - margin,
              half_y + distance + margin if direction > 0 else -half_y)
    observed = []
    for index, value in enumerate(scan.ranges):
        angle = scan.angle_min + index * scan.angle_increment + yaw
        vx, vy = math.cos(angle), math.sin(angle)
        enter, leave = 0.0, float('inf')
        for origin, vector, low, high in (
            (sx, vx, bounds[0], bounds[1]), (sy, vy, bounds[2], bounds[3])
        ):
            if abs(vector) < 1e-9:
                if not low <= origin <= high:
                    leave = -1.0
                continue
            first, last = sorted(((low - origin) / vector, (high - origin) / vector))
            enter, leave = max(enter, first), min(leave, last)
        if leave < max(enter, 0.0):
            continue
        valid = math.isfinite(value) and scan.range_min <= value <= scan.range_max
        if valid and value < leave + 0.02:
            return False
        observed.append((angle, valid))
    if not observed or len(observed) < 3:
        return False
    # Check all intersecting rays, allowing only very small missing-beam holes.
    missing_arc = 0.0
    for _, valid in observed:
        missing_arc = 0.0 if valid else missing_arc + abs(scan.angle_increment)
        if missing_arc > max_gap:
            return False
    return sum(valid for _, valid in observed) / len(observed) >= 0.95


def lateral_speed(progress, distance=0.30, cap=0.25):
    """Brake before the target; stopping is verified separately by real feedback."""
    remaining = distance - progress
    if remaining <= 0.015:
        return 0.0
    return min(cap, math.sqrt(2.0 * 0.5 * max(0.0, remaining - 0.015)))
