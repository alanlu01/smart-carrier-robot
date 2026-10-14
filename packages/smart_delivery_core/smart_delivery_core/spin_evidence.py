"""Short-lived odom-aligned, measured free-space evidence for crowd rotation.

Unknown is never converted into a range. Historical free rays must reach past
the sweep plus an age-dependent 2 m/s moving-obstacle allowance. Near returns
take priority, including recent occupied evidence. Not a safety certification.
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass
class ScanEvidence:
    source_sec: float
    received_at: float
    sensor_pose: tuple
    base_pose: tuple
    angles: np.ndarray
    ranges: np.ndarray
    range_min: float
    range_max: float

    @classmethod
    def from_scan(cls, scan, received_at, sensor_pose, base_pose):
        values = (*sensor_pose, *base_pose, scan.angle_min, scan.angle_increment,
                  scan.range_min, scan.range_max)
        if (not all(math.isfinite(v) for v in values) or scan.range_min < 0
                or scan.range_max <= scan.range_min
                or not 6.0 <= abs(scan.angle_increment)*len(scan.ranges) <= 2*math.pi+.1):
            raise ValueError('invalid full-circle laser geometry')
        return cls(scan.header.stamp.sec+scan.header.stamp.nanosec/1e9, received_at,
                   tuple(sensor_pose), tuple(base_pose),
                   scan.angle_min+np.arange(len(scan.ranges))*scan.angle_increment,
                   np.asarray(scan.ranges, dtype=float).copy(), scan.range_min, scan.range_max)


def aligned_spin_report(frames, base_pose, now, ros_now, radius, max_unknown_deg=15.):
    """Rebin measured rays about the current laser; preserve the 15° scan budget."""
    report = {'clear': False, 'reason': 'insufficient_recent_scan_evidence',
              'evidence_frames': 0, 'evidence_mode': 'odom_aligned_short_history'}
    if not frames or not all(math.isfinite(v) for v in (*base_pose, now, ros_now, radius)):
        return report
    # Must have a freshly transformed latest frame, never just old free space.
    newest = frames[-1]
    if not (-.1 <= ros_now-newest.source_sec <= .30 and 0 <= now-newest.received_at <= .30):
        return dict(report, reason='scan_stale_or_absent')
    latest_valid = np.isfinite(newest.ranges) & (newest.ranges >= newest.range_min) & (newest.ranges <= newest.range_max)
    current_fraction = float(np.mean(latest_valid))
    report['current_valid_fraction'] = round(current_fraction,4)
    sx0, sy0, yaw0 = base_pose
    cosine, sine = math.cos(yaw0), math.sin(yaw0)
    # The old 15° budget is defined about the LASER origin, not chassis center.
    # A laser-centered circle enclosing the full body sweep is conservative and
    # avoids silently changing that angular budget for an offset lidar.
    nx,ny,nyaw = newest.base_pose
    if (math.hypot(nx-sx0,ny-sy0) > .03
            or abs(math.atan2(math.sin(nyaw-yaw0),math.cos(nyaw-yaw0))) > .35):
        return dict(report, reason='laser_pose_jump')
    ndx,ndy = newest.sensor_pose[0]-nx,newest.sensor_pose[1]-ny
    cn,sn = math.cos(nyaw),math.sin(nyaw)
    current_laser_x,current_laser_y = cn*ndx+sn*ndy,-sn*ndx+cn*ndy
    ring_radius = radius+math.hypot(current_laser_x,current_laser_y)
    known = np.zeros(360, dtype=bool)
    used = 0
    source_ids = set()
    used_sources = []
    for frame in reversed(frames):
        age = max(0., ros_now-frame.source_sec)
        if not (-.1 <= ros_now-frame.source_sec <= .50 and 0 <= now-frame.received_at <= .45):
            continue
        if frame.source_sec in source_ids:
            continue
        source_ids.add(frame.source_sec)
        bx, by, byaw = frame.base_pose
        turn = abs(math.atan2(math.sin(byaw-yaw0), math.cos(byaw-yaw0)))
        if math.hypot(bx-sx0, by-sy0) > .03 or turn > .35:
            # Old unaligned/large-motion history never authorizes this pose.
            continue
        # Fixed mounting is part of this evidence model; changing TF calibration
        # must not mix rays from two different lidar extrinsics.
        mount_dx,mount_dy = frame.sensor_pose[0]-bx,frame.sensor_pose[1]-by
        cb,sb = math.cos(byaw),math.sin(byaw)
        mount_x,mount_y = cb*mount_dx+sb*mount_dy,-sb*mount_dx+cb*mount_dy
        mount_turn = frame.sensor_pose[2]-byaw-(newest.sensor_pose[2]-nyaw)
        if (math.hypot(mount_x-current_laser_x,mount_y-current_laser_y) > .005
                or abs(math.atan2(math.sin(mount_turn),math.cos(mount_turn))) > .01):
            continue
        dx, dy = frame.sensor_pose[0]-sx0, frame.sensor_pose[1]-sy0
        sx, sy = cosine*dx+sine*dy, -sine*dx+cosine*dy
        if sx*sx+sy*sy >= radius*radius:
            return dict(report, reason='laser_origin_outside_sweep')
        angles = frame.angles+frame.sensor_pose[2]-yaw0
        vx, vy = np.cos(angles), np.sin(angles)
        valid = np.isfinite(frame.ranges) & (frame.ranges >= frame.range_min) & (frame.ranges <= frame.range_max)
        ranges = frame.ranges[valid]
        vx, vy = vx[valid], vy[valid]
        x, y = sx+ranges*vx, sy+ranges*vy
        near = np.hypot(x, y) <= radius+.03
        if np.any(near):
            return dict(report, reason='near_obstacle' if frame is newest else 'recent_near_obstacle',
                        obstacle_distance_m=round(float(np.min(np.hypot(x[near],y[near]))),3))
        ring_sx,ring_sy = sx-current_laser_x,sy-current_laser_y
        dot = ring_sx*vx+ring_sy*vy
        # .03 m spatial budget + .04 s cache lifetime + 2 m/s temporal budget.
        required_radius = ring_radius+.03+2.0*(age+.04)
        required_exit = -dot+np.sqrt(dot*dot+required_radius**2-ring_sx**2-ring_sy**2)
        free = ranges >= required_exit
        exit_distance = -dot+np.sqrt(dot*dot+ring_radius**2-ring_sx**2-ring_sy**2)
        angles_exit = np.arctan2(ring_sy+exit_distance[free]*vy[free],
                                 ring_sx+exit_distance[free]*vx[free])
        bins = np.floor((angles_exit+math.pi)*180/math.pi).astype(int) % 360
        known[bins] = True
        used += 1
        used_sources.append(frame.source_sec)
    report['evidence_frames'] = used
    run = peak = 0
    for available in np.concatenate((known,known)):
        run = 0 if available else run+1
        peak = max(peak,run)
    # Quantization must not make a real unknown hole look smaller.
    gap = min(360., min(peak,360)+2.0) if peak else 0.
    fraction = float(np.mean(known))
    clear = used >= 3 and current_fraction >= .75 and fraction >= .75 and gap <= max_unknown_deg
    report.update(clear=clear, reason='clear' if clear else 'unknown_scan_coverage',
                  valid_fraction=round(fraction,4), unknown_arc_deg=gap)
    report['evidence_deadline_sec'] = min(used_sources)+.50 if used_sources else ros_now
    return report
