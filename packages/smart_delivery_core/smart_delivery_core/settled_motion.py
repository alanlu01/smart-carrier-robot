"""Stopped-entry and bounded convergence helpers; no ROS or hardware access."""
import math


def can_arm_stationary_anchor(now, last_command_at, commanded, moving, stop_proven):
    return (not commanded and not moving and stop_proven
            and now-last_command_at >= 1.)


def near_convergence(samples, xy_limit, yaw_limit, distance=.20, angle=.174533):
    if len(samples) < 4 or samples[-1][0]-samples[0][0] < 3.:
        return False
    first, last = samples[0], samples[-1]
    if not (0 < last[2] <= xy_limit*1.2 and last[3] <= yaw_limit
            and last[2] <= first[2]*.97):
        return False
    anchor = last[1]
    for _, pose, xy, _yaw in samples:
        if not all(math.isfinite(v) for v in (*pose, xy)):
            return False
        delta = pose[2]-anchor[2]
        if (math.hypot(pose[0]-anchor[0], pose[1]-anchor[1]) > distance
                or abs(math.atan2(math.sin(delta), math.cos(delta))) > angle):
            return False
    return all(b[2] <= a[2]+.01 for a,b in zip(samples,samples[1:]))
