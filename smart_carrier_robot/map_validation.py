import math
from collections.abc import Sequence
from dataclasses import dataclass


@dataclass(frozen=True)
class OccupancyMap:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: Sequence[int]
    origin_yaw: float = 0.0

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("map width and height must be positive")
        if self.resolution <= 0:
            raise ValueError("map resolution must be positive")
        if len(self.data) != self.width * self.height:
            raise ValueError("occupancy data size does not match map dimensions")

    @property
    def local_bounds(self) -> tuple[float, float, float, float]:
        return (
            self.origin_x,
            self.origin_x + self.width * self.resolution,
            self.origin_y,
            self.origin_y + self.height * self.resolution,
        )

    def world_to_grid(self, x: float, y: float) -> tuple[int, int] | None:
        dx = float(x) - self.origin_x
        dy = float(y) - self.origin_y
        cos_yaw = math.cos(self.origin_yaw)
        sin_yaw = math.sin(self.origin_yaw)
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        grid_x = math.floor(local_x / self.resolution)
        grid_y = math.floor(local_y / self.resolution)
        if not self.contains_grid(grid_x, grid_y):
            return None
        return grid_x, grid_y

    def contains_grid(self, grid_x: int, grid_y: int) -> bool:
        return 0 <= grid_x < self.width and 0 <= grid_y < self.height

    def value_at(self, grid_x: int, grid_y: int) -> int:
        return int(self.data[grid_y * self.width + grid_x])


@dataclass(frozen=True)
class LocationValidation:
    valid: bool
    reason: str
    grid_x: int | None = None
    grid_y: int | None = None
    nearest_blocked_m: float | None = None


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def validate_location(
    occupancy_map: OccupancyMap,
    x: float,
    y: float,
    *,
    clearance_m: float = 0.35,
    max_cost: int = 0,
) -> LocationValidation:
    if clearance_m < 0:
        raise ValueError("clearance_m must not be negative")

    grid = occupancy_map.world_to_grid(x, y)
    if grid is None:
        min_x, max_x, min_y, max_y = occupancy_map.local_bounds
        return LocationValidation(
            False,
            (
                f"outside map bounds x=[{min_x:.3f}, {max_x:.3f}), "
                f"y=[{min_y:.3f}, {max_y:.3f})"
            ),
        )

    grid_x, grid_y = grid
    center_value = occupancy_map.value_at(grid_x, grid_y)
    if center_value < 0:
        return LocationValidation(False, "target cell is unknown", grid_x, grid_y)
    if center_value > max_cost:
        return LocationValidation(
            False,
            f"target cell cost {center_value} exceeds allowed cost {max_cost}",
            grid_x,
            grid_y,
            0.0,
        )

    radius_cells = math.ceil(clearance_m / occupancy_map.resolution)
    nearest_blocked_m = None
    for candidate_y in range(grid_y - radius_cells, grid_y + radius_cells + 1):
        for candidate_x in range(grid_x - radius_cells, grid_x + radius_cells + 1):
            distance_m = math.hypot(candidate_x - grid_x, candidate_y - grid_y)
            distance_m *= occupancy_map.resolution
            if distance_m > clearance_m:
                continue
            if not occupancy_map.contains_grid(candidate_x, candidate_y):
                return LocationValidation(
                    False,
                    f"required {clearance_m:.2f} m clearance crosses map boundary",
                    grid_x,
                    grid_y,
                )
            value = occupancy_map.value_at(candidate_x, candidate_y)
            if value < 0 or value > max_cost:
                nearest_blocked_m = (
                    distance_m
                    if nearest_blocked_m is None
                    else min(nearest_blocked_m, distance_m)
                )

    if nearest_blocked_m is not None:
        return LocationValidation(
            False,
            (
                f"nearest unknown/blocked cell is {nearest_blocked_m:.2f} m away; "
                f"requires {clearance_m:.2f} m"
            ),
            grid_x,
            grid_y,
            nearest_blocked_m,
        )

    return LocationValidation(
        True,
        f"free with at least {clearance_m:.2f} m clearance",
        grid_x,
        grid_y,
    )
