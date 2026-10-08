"""Map-independent, fail-closed geometry for one supervised recovery sidestep."""

import math

import yaml


def load_self_mask(path, half_x, half_y):
    """Read the same box as laser_filters; never accept an enlarged free zone."""
    with open(path, encoding='utf-8') as stream:
        try:
            config = yaml.safe_load(stream)['scan_to_scan_filter_chain']['ros__parameters']
        except yaml.YAMLError as exc:
            raise ValueError('Invalid laser filter YAML') from exc
    if config['filters'] != ['filter1']:
        raise ValueError('Unsupported laser filter chain')
    entry = config['filter1']
    params = entry['params']
    if (entry['type'] != 'laser_filters/LaserScanBoxFilter'
            or params['box_frame'] != 'base_footprint' or params['invert'] is not False):
        raise ValueError('Self mask must be a non-inverted base_footprint box')
    bounds = tuple(float(params[key]) for key in
                   ('min_x', 'max_x', 'min_y', 'max_y', 'min_z', 'max_z'))
    if (not all(math.isfinite(value) for value in bounds)
            or any(bounds[i] >= bounds[i + 1] for i in (0, 2, 4))
            or not -half_x <= bounds[0] < bounds[1] <= half_x
            or not -half_y <= bounds[2] < bounds[3] <= half_y):
        raise ValueError('Self mask must remain inside the recovery footprint')
    return bounds


def lateral_progress(anchor, pose, direction):
    """Project displacement in the ORIGINAL body frame, never an AMCL frame."""
    dx, dy = pose[0] - anchor[0], pose[1] - anchor[1]
    c, s = math.cos(anchor[2]), math.sin(anchor[2])
    yaw_error = math.atan2(math.sin(pose[2] - anchor[2]), math.cos(pose[2] - anchor[2]))
    return direction * (-s * dx + c * dy), c * dx + s * dy, yaw_error


def swept_scan_clear(scan, sensor_pose, direction, distance, half_x, half_y,
                     margin=0.08, max_gap=math.radians(4), self_mask=None, sensor_z=0.0):
    """Compatibility boolean interface to the detailed fail-closed check."""
    return swept_scan_report(scan, sensor_pose, direction, distance, half_x, half_y,
                             margin, max_gap, self_mask, sensor_z)['clear']


def swept_scan_report(scan, sensor_pose, direction, distance, half_x, half_y,
                      margin=0.08, max_gap=math.radians(4), self_mask=None, sensor_z=0.0):
    """Require measured free rays across the entire added swept rectangle.

    Transform rays into base_footprint (including a backwards-mounted laser).
    NaN/Inf/occluded/missing rays are NOT evidence of free space. The rectangle
    includes a margin around the full footprint, not merely its centre line.
    This does not claim to detect transparent glass invisible to the lidar.
    """
    report = {'clear': False, 'reason': 'invalid_geometry', 'masked_beams': 0,
              'ray_count': 0, 'valid_fraction': 0.0, 'missing_arc_deg': 0.0}
    if (direction not in (-1, 1) or distance <= 0 or not scan.ranges
            or not all(math.isfinite(v) for v in (*sensor_pose, sensor_z, distance,
                                                  half_x, half_y, scan.angle_increment))
            or scan.angle_increment == 0 or half_x <= 0 or half_y <= 0
            or not math.isfinite(scan.angle_min)):
        return report
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
        # A known self-body return is occluded/UNKNOWN, never measured free.
        # Only the existing <=4 degree / >=95% small-hole policy may accept it.
        if valid and self_mask is not None:
            x, y = sx + value * vx, sy + value * vy
            if (self_mask[0] < x < self_mask[1] and self_mask[2] < y < self_mask[3]
                    and self_mask[4] < sensor_z < self_mask[5]):
                valid = False
                report['masked_beams'] += 1
        if valid and value < leave + 0.02:
            report.update(reason='obstacle_or_occlusion', obstacle_x=round(sx + value * vx, 4),
                          obstacle_y=round(sy + value * vy, 4), range_m=round(value, 4))
            return report
        observed.append((index, valid))
    report['ray_count'] = len(observed)
    if not observed or len(observed) < 3:
        report['reason'] = 'insufficient_rays'
        return report
    # Check all intersecting rays, allowing only very small missing-beam holes.
    missing_arc = peak_arc = leading_arc = 0.0
    leading = True
    for _, valid in observed:
        missing_arc = 0.0 if valid else missing_arc + abs(scan.angle_increment)
        peak_arc = max(peak_arc, missing_arc)
        if valid:
            leading = False
        elif leading:
            leading_arc += abs(scan.angle_increment)
    # A full-circle scan may split one unknown hole across its array boundary.
    if (observed[0][0] == 0 and observed[-1][0] == len(scan.ranges) - 1
            and abs(abs(scan.angle_increment) * len(scan.ranges) - 2 * math.pi) < .02):
        peak_arc = max(peak_arc, min(leading_arc + missing_arc,
                                   len(observed) * abs(scan.angle_increment)))
    fraction = sum(valid for _, valid in observed) / len(observed)
    clear = peak_arc <= max_gap and fraction >= .95
    report.update(clear=clear, reason='clear' if clear else 'unknown_coverage',
                  valid_fraction=round(fraction, 4), missing_arc_deg=round(math.degrees(peak_arc), 3))
    return report


def lateral_speed(progress, distance=0.30, cap=0.25):
    """Brake before the target; stopping is verified separately by real feedback."""
    remaining = distance - progress
    if remaining <= 0.015:
        return 0.0
    return min(cap, math.sqrt(2.0 * 0.5 * max(0.0, remaining - 0.015)))
