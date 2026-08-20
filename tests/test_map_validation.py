import pytest

from smart_carrier_robot.map_validation import OccupancyMap, quaternion_to_yaw, validate_location


def make_map(*, width=145, height=144, fill=0):
    return OccupancyMap(
        width=width,
        height=height,
        resolution=0.05,
        origin_x=-4.879,
        origin_y=-0.969,
        data=[fill] * (width * height),
    )


def test_room_map_bounds_match_metadata():
    assert make_map().local_bounds == pytest.approx((-4.879, 2.371, -0.969, 6.231))


@pytest.mark.parametrize("point", [(14, 7), (8, 3), (5, 12)])
def test_existing_cloud_locations_outside_room_map(point):
    result = validate_location(make_map(), *point)
    assert not result.valid
    assert result.reason.startswith("outside map bounds")


def test_a1_is_inside_room_map():
    result = validate_location(make_map(), 0, 0, clearance_m=0.35)
    assert result.valid
    assert (result.grid_x, result.grid_y) == (97, 19)


def test_rejects_target_without_required_clearance():
    room_map = make_map(width=20, height=20)
    data = list(room_map.data)
    data[10 * room_map.width + 12] = 100
    room_map = OccupancyMap(
        width=room_map.width,
        height=room_map.height,
        resolution=room_map.resolution,
        origin_x=room_map.origin_x,
        origin_y=room_map.origin_y,
        data=data,
    )
    x = room_map.origin_x + 10 * room_map.resolution
    y = room_map.origin_y + 10 * room_map.resolution

    result = validate_location(room_map, x, y, clearance_m=0.15)

    assert not result.valid
    assert result.nearest_blocked_m == pytest.approx(0.1)


def test_quaternion_to_yaw():
    assert quaternion_to_yaw(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0)
    assert quaternion_to_yaw(0.0, 0.0, 2**-0.5, 2**-0.5) == pytest.approx(1.57079632679)
