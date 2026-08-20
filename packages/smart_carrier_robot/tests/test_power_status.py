import json

from smart_carrier_robot.power_status import classify_current, payload_to_slots


def test_current_classification():
    assert classify_current(0.5) == "charging"
    assert classify_current(0.1) == "almost_full"
    assert classify_current(0.0) == "not_inserted"


def test_payload_converts_three_channels():
    payload = json.dumps(
        {
            "ch1": {"current": 0.5, "status": "charging"},
            "ch2": {"current": 0.1, "status": "almost_full"},
            "ch3": {"current": 0.0, "status": "not_inserted"},
        }
    )
    slots = payload_to_slots(payload)
    assert [slot["status"] for slot in slots] == ["charging", "ready", "empty"]
