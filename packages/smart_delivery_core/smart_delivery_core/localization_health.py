import math
import statistics
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


@dataclass(frozen=True)
class ScanSampleSelection:
    """Laser samples retained for localization-only map matching."""

    samples: tuple[tuple[int, float], ...]
    near_fraction: float
    sector_count: int
    valid_count: int


@dataclass
class EvidenceWaitClock:
    """Count usable observation time, never charging a sensor outage as failure."""

    elapsed: float = 0.0
    last_checked_at: float | None = None
    previously_ready: bool = False
    blocked_since: float | None = None

    def update(self, now, ready):
        """Conservatively exclude both edges of an unavailable-data interval."""
        now = float(now)
        if self.last_checked_at is not None and self.previously_ready and ready:
            self.elapsed += max(0.0, now - self.last_checked_at)
        self.last_checked_at = now
        self.previously_ready = bool(ready)
        if ready:
            self.blocked_since = None
        elif self.blocked_since is None:
            self.blocked_since = now
        return self.elapsed

    def blocked_for(self, now):
        """Return the continuous outage duration for a bounded safe-stop wait."""
        return 0.0 if self.blocked_since is None else max(
            0.0, float(now) - self.blocked_since
        )


def minimum_verification_wait(samples, amcl_period, map_period, map_samples):
    """Budget map-window warmup, independent AMCL samples and scheduling slack."""
    interval = max(float(amcl_period), float(map_period))
    return max(1, int(map_samples)) * float(map_period) + (
        max(1, int(samples)) + 2
    ) * interval


def covariance_is_converging(samples, minimum_improvement=0.02):
    """Require recent independent evidence of improving position uncertainty."""
    if len(samples) < 3:
        return False
    elapsed, current = samples[-1]
    recent = [(t, std) for t, std in samples if elapsed - t <= 8.0]
    if len(recent) < 3 or elapsed - recent[0][0] < 3.0:
        return False
    previous = recent[0][1]
    return previous - current >= max(minimum_improvement, previous * 0.05)


def convergence_grace_allowed(elapsed, normal_wait, maximum_wait, promising):
    """Extend observation only within a finite budget, never accept a pose."""
    return bool(promising and normal_wait <= elapsed < maximum_wait)


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


def heading_correction(reference_yaw, current_yaw, tolerance=0.0):
    """Return the shortest safe-to-request yaw correction back to a reference."""
    correction = normalize_angle(float(reference_yaw) - float(current_yaw))
    return 0.0 if abs(correction) <= float(tolerance) else correction


def damped_heading_command(error, gain=0.65, maximum=math.radians(20.0)):
    """Scale and bound a heading correction to reduce small-angle overshoot."""
    if not 0.0 < gain <= 1.0:
        raise ValueError("heading correction gain must be in (0, 1]")
    if maximum <= 0.0:
        raise ValueError("maximum heading correction must be positive")
    error = normalize_angle(error)
    command = error * gain
    return math.copysign(min(abs(command), maximum), command)


def heading_error_is_improving(previous_error, current_error, minimum_improvement=0.0):
    """Reject a correction loop that reverses direction or stops converging."""
    if previous_error is None:
        return True
    if minimum_improvement < 0.0:
        raise ValueError("minimum heading improvement must not be negative")
    if previous_error * current_error < 0.0:
        return False
    return abs(current_error) <= abs(previous_error) - minimum_improvement


def map_match_status(score, age, minimum_score, critical_score, maximum_age):
    """Classify a scan-to-map score using freshness and hysteresis thresholds."""
    if score is None or age < 0.0 or age > maximum_age:
        return "unknown"
    if score <= critical_score:
        return "critical"
    if score < minimum_score:
        return "degraded"
    return "healthy"


def sensor_timeout_requires_recovery(age, timeout, grace=0.0):
    """Return whether a receive gap exceeded both its limit and debounce grace."""
    if age is None:
        return True
    return float(age) > float(timeout) + max(0.0, float(grace))


def scan_pipeline_status(raw_age, filtered_age, timeout, grace=0.0):
    """Classify a filtered-scan gap without confusing it with lidar loss."""
    filtered_stale = sensor_timeout_requires_recovery(
        filtered_age, timeout, grace
    )
    if not filtered_stale:
        return "healthy"
    raw_stale = sensor_timeout_requires_recovery(raw_age, timeout, grace)
    return "raw_timeout" if raw_stale else "filtered_timeout"


def localization_suspicion_required(
    match_status,
    quality_healthy,
    quality_critical,
    commanded_motion,
    odom_motion,
):
    """Require corroboration before a map-score drop pauses localization."""
    if bool(quality_critical):
        return True
    if match_status not in {"critical", "degraded", "unknown"}:
        return False
    return (
        not bool(quality_healthy)
        or bool(commanded_motion)
        or bool(odom_motion)
    )


def transient_sensor_recovery_qualified(
    quality_healthy,
    map_match_recovered,
    scan_age,
    scan_timeout,
    odom_age,
    odom_timeout,
    amcl_age,
    amcl_timeout,
    commanded_motion,
    odom_motion,
):
    """Require fresh independent evidence before clearing a transient timeout."""
    ages = (scan_age, odom_age, amcl_age)
    if any(age is None or float(age) < 0.0 for age in ages):
        return False
    return (
        bool(quality_healthy)
        and bool(map_match_recovered)
        and float(scan_age) <= float(scan_timeout)
        and float(odom_age) <= float(odom_timeout)
        and float(amcl_age) <= float(amcl_timeout)
        and not bool(commanded_motion)
        and not bool(odom_motion)
    )


def amcl_timeout_requires_recovery(age, timeout, odom_moving, stationary_grace=0.0):
    """Allow one no-motion refresh window when odometry says the robot is still."""
    if age is None:
        return True
    age = float(age)
    timeout = float(timeout)
    if age <= timeout:
        return False
    if odom_moving:
        return True
    return age > timeout + max(0.0, float(stationary_grace))


def update_stability_samples(qualified, sample_at, last_sample_at, sample_count):
    """Count each fresh qualified sample once and reset on an unhealthy sample."""
    sample_at = float(sample_at)
    last_sample_at = float(last_sample_at)
    if not qualified:
        return 0, max(last_sample_at, sample_at)
    if sample_at <= last_sample_at:
        return int(sample_count), last_sample_at
    return int(sample_count) + 1, sample_at


def smoothed_map_score(scores, minimum_samples=1):
    """Return a median score only after enough finite observations exist."""
    finite_scores = [float(score) for score in scores if math.isfinite(float(score))]
    if len(finite_scores) < int(minimum_samples):
        return None
    return float(statistics.median(finite_scores))


def suspect_requires_recovery(
    qualified,
    match_status,
    quality_critical,
    elapsed,
    hold_seconds,
    maximum_seconds,
    promising=False,
    extended_maximum_seconds=None,
):
    """Ensure SUSPECT cannot remain forever inside the hysteresis band."""
    if qualified or float(elapsed) < float(hold_seconds):
        return False
    if promising:
        extended_maximum = max(
            float(maximum_seconds),
            float(
                maximum_seconds
                if extended_maximum_seconds is None
                else extended_maximum_seconds
            ),
        )
        return float(elapsed) >= extended_maximum
    if float(elapsed) >= float(maximum_seconds):
        return True
    return match_status in {"critical", "degraded", "unknown"} or bool(
        quality_critical
    )


def should_extend_global_recovery(
    elapsed,
    normal_wait_seconds,
    maximum_wait_seconds,
    map_match_recovered,
):
    """Allow extra AMCL convergence time only for a map-consistent hypothesis."""
    elapsed = float(elapsed)
    normal_wait_seconds = float(normal_wait_seconds)
    maximum_wait_seconds = max(normal_wait_seconds, float(maximum_wait_seconds))
    return (
        bool(map_match_recovered)
        and elapsed >= normal_wait_seconds
        and elapsed < maximum_wait_seconds
    )


def select_scan_samples(
    ranges,
    range_min,
    range_max,
    ignore_below_range,
    maximum_samples,
    sector_total=12,
):
    """Select broad, non-near-field scan samples for localization scoring.

    Close returns are still available to Nav2 and collision monitoring. They are
    excluded only from the static-map score because nearby people commonly
    occlude the mapped wall behind them.
    """
    range_min = float(range_min)
    range_max = float(range_max)
    ignore_below_range = max(range_min, float(ignore_below_range))
    maximum_samples = max(1, int(maximum_samples))
    sector_total = max(1, int(sector_total))
    total_ranges = len(ranges)
    valid = []
    nearby_count = 0
    for index, raw_distance in enumerate(ranges):
        distance = float(raw_distance)
        if not math.isfinite(distance):
            continue
        if distance < range_min or distance >= range_max * 0.995:
            continue
        if distance < ignore_below_range:
            nearby_count += 1
            continue
        valid.append((index, distance))

    valid_count = len(valid) + nearby_count
    near_fraction = nearby_count / valid_count if valid_count else 0.0
    if not valid:
        return ScanSampleSelection((), near_fraction, 0, valid_count)

    stride = max(1, math.ceil(len(valid) / maximum_samples))
    sampled = tuple(valid[::stride][:maximum_samples])
    sectors = {
        min(sector_total - 1, index * sector_total // max(1, total_ranges))
        for index, _distance in sampled
    }
    return ScanSampleSelection(sampled, near_fraction, len(sectors), valid_count)


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
