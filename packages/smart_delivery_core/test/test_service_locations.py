from smart_delivery_core.service_locations import LOCATION_DB, STANDBY_POINTS


def test_service_and_standby_locations_match_the_3f_map():
    assert LOCATION_DB == {
        "右樓梯口": {"x": 7.7, "y": -7.2, "yaw": 0.0},
        "左樓梯口": {"x": 7.5, "y": 5.6, "yaw": 0.0},
        "電機工程學系": {"x": -2.6, "y": 4.7, "yaw": 0.0},
    }
    assert STANDBY_POINTS == [
        {"name": "CYCU EE", "x": 2.4, "y": 5.0, "yaw": 0.0},
        {"name": "座位燈", "x": 1.2, "y": -5.0, "yaw": 0.0},
    ]
