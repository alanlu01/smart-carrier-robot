import json

import pytest

from hailo_vision.semantic_protocol import (
    build_semantic_payload,
    closest_timestamped_item,
    parse_semantic_payload,
    semantic_data_age,
    semantic_safety_multiplier,
    stamps_are_synchronized,
)


def test_timestamped_semantic_payload_round_trip():
    payload = build_semantic_payload(
        [{"id": 0, "angle": 1.5}],
        stamp_sec=12,
        stamp_nanosec=345,
        frame_id="camera",
        sequence=7,
    )

    packet = parse_semantic_payload(json.dumps(payload))
    assert packet.detections == [{"id": 0, "angle": 1.5}]
    assert packet.stamp_ns == 12_000_000_345
    assert packet.frame_id == "camera"
    assert packet.sequence == 7
    assert not packet.legacy


def test_legacy_detection_list_remains_accepted_during_rollout():
    packet = parse_semantic_payload('[{"id": 10}]')
    assert packet.detections == [{"id": 10}]
    assert packet.stamp_ns is None
    assert packet.legacy


@pytest.mark.parametrize(
    ("age", "expected"),
    [(0.5, 1.0), (0.51, 0.5), (2.0, 0.5), (2.01, 0.0)],
)
def test_semantic_freshness_policy(age, expected):
    assert semantic_safety_multiplier(age, 0.5, 0.5, 2.0) == expected


def test_semantic_source_and_scan_stamp_skew():
    assert stamps_are_synchronized(1_000_000_000, 1_190_000_000, 0.2)
    assert not stamps_are_synchronized(1_000_000_000, 1_210_000_000, 0.2)
    assert stamps_are_synchronized(None, 1_210_000_000, 0.2)


def test_semantic_age_uses_delayed_source_stamp_even_when_recently_received():
    assert semantic_data_age(
        receipt_age_sec=0.1,
        source_stamp_ns=1_000_000_000,
        now_ns=3_500_000_000,
    ) == pytest.approx(2.5)


def test_invalid_stamp_is_rejected():
    with pytest.raises(ValueError, match="stamp"):
        parse_semantic_payload(
            {
                "stamp": {"sec": 1, "nanosec": 1_000_000_000},
                "detections": [],
            }
        )


def test_closest_timestamped_item_uses_historical_sensor_sample():
    samples = [
        (1_000_000_000, "old"),
        (1_100_000_000, "closest"),
        (1_200_000_000, "new"),
    ]

    assert closest_timestamped_item(samples, 1_130_000_000, 0.05) == (
        1_100_000_000,
        "closest",
    )


def test_closest_timestamped_item_rejects_excessive_skew():
    samples = [(1_000_000_000, "scan")]
    assert closest_timestamped_item(samples, 1_210_000_000, 0.20) is None


def test_closest_timestamped_item_accepts_latest_for_legacy_payload():
    samples = [(1, "old"), (2, "latest")]
    assert closest_timestamped_item(samples, None, 0.20) == (2, "latest")
