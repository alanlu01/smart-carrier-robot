import math
from dataclasses import dataclass


def normalize_angle(angle):
    """Return an angle in the closed-open interval [-pi, pi)."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_to_yaw(quaternion):
    """Extract planar yaw from a geometry_msgs-compatible quaternion."""
    siny_cosp = 2.0 * (
        float(quaternion.w) * float(quaternion.z)
        + float(quaternion.x) * float(quaternion.y)
    )
    cosy_cosp = 1.0 - 2.0 * (
        float(quaternion.y) * float(quaternion.y)
        + float(quaternion.z) * float(quaternion.z)
    )
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass(frozen=True)
class PoseQuality:
    xy_std: float
    yaw_std: float
    healthy: bool
    critical: bool


def pose_quality(
    covariance,
    healthy_xy_std,
    healthy_yaw_std,
    critical_xy_std,
    critical_yaw_std,
):
    """Summarize the planar uncertainty from a 6x6 pose covariance."""
    if len(covariance) < 36:
        raise ValueError("pose covariance must contain 36 values")
    xy_variance = max(0.0, float(covariance[0]), float(covariance[7]))
    yaw_variance = max(0.0, float(covariance[35]))
    xy_std = math.sqrt(xy_variance)
    yaw_std = math.sqrt(yaw_variance)
    return PoseQuality(
        xy_std=xy_std,
        yaw_std=yaw_std,
        healthy=xy_std <= healthy_xy_std and yaw_std <= healthy_yaw_std,
        critical=xy_std >= critical_xy_std or yaw_std >= critical_yaw_std,
    )


def pose_is_near(x, y, yaw, reference_x, reference_y, reference_yaw, xy_limit, yaw_limit):
    """Check whether a pose is a plausible correction around a seeded pose."""
    return (
        math.hypot(float(x) - float(reference_x), float(y) - float(reference_y))
        <= float(xy_limit)
        and abs(normalize_angle(float(yaw) - float(reference_yaw))) <= float(yaw_limit)
    )


def pose_jump(previous, current):
    """Return planar translation and yaw differences between two pose tuples."""
    if previous is None or current is None:
        return 0.0, 0.0
    distance = math.hypot(float(current[0]) - float(previous[0]), float(current[1]) - float(previous[1]))
    angle = abs(normalize_angle(float(current[2]) - float(previous[2])))
    return distance, angle


def occupancy_match_score(
    endpoints,
    occupancy_data,
    width,
    height,
    resolution,
    origin_x,
    origin_y,
    origin_yaw=0.0,
    occupied_threshold=65,
    neighborhood_cells=2,
    minimum_endpoints=1,
):
    """Return the fraction of laser endpoints near occupied map cells."""
    if width <= 0 or height <= 0 or resolution <= 0.0:
        return None
    cosine = math.cos(float(origin_yaw))
    sine = math.sin(float(origin_yaw))
    matches = 0
    considered = 0
    for world_x, world_y in endpoints:
        delta_x = float(world_x) - float(origin_x)
        delta_y = float(world_y) - float(origin_y)
        grid_x = int(math.floor((cosine * delta_x + sine * delta_y) / resolution))
        grid_y = int(math.floor((-sine * delta_x + cosine * delta_y) / resolution))
        if not 0 <= grid_x < width or not 0 <= grid_y < height:
            continue
        considered += 1
        matched = False
        for offset_y in range(-neighborhood_cells, neighborhood_cells + 1):
            y = grid_y + offset_y
            if not 0 <= y < height:
                continue
            for offset_x in range(-neighborhood_cells, neighborhood_cells + 1):
                x = grid_x + offset_x
                if not 0 <= x < width:
                    continue
                if int(occupancy_data[y * width + x]) >= occupied_threshold:
                    matched = True
                    break
            if matched:
                break
        matches += int(matched)
    return matches / considered if considered >= minimum_endpoints else None
