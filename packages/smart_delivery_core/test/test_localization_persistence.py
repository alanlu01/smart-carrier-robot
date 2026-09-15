import json

from smart_delivery_core.localization_persistence import (
    load_cache,
    select_same_boot_pose,
    write_cache,
)


def test_same_boot_cache_reuses_only_recent_trusted_pose(tmp_path):
    path = tmp_path / "pose.json"
    write_cache(path, "boot-a", (1.25, -2.5, 0.4), now=100.0)
    status, pose = select_same_boot_pose(
        load_cache(path), "boot-a", now=120.0, max_age_sec=60.0
    )
    assert status == "same_boot_cached"
    assert pose == (1.25, -2.5, 0.4)


def test_new_boot_never_reuses_previous_pose(tmp_path):
    path = tmp_path / "pose.json"
    write_cache(path, "boot-a", (1.0, 2.0, 3.0), now=100.0)
    assert select_same_boot_pose(load_cache(path), "boot-b", now=101.0) == (
        "new_boot",
        None,
    )


def test_same_boot_without_recent_pose_requires_manual_seed(tmp_path):
    path = tmp_path / "pose.json"
    write_cache(path, "boot-a", now=100.0)
    assert select_same_boot_pose(load_cache(path), "boot-a", now=101.0) == (
        "same_boot_untrusted",
        None,
    )

    write_cache(path, "boot-a", (1.0, 2.0, 3.0), now=100.0)
    assert select_same_boot_pose(
        load_cache(path), "boot-a", now=200.0, max_age_sec=60.0
    ) == ("same_boot_untrusted", None)


def test_corrupt_same_boot_pose_is_not_treated_as_power_on(tmp_path):
    path = tmp_path / "pose.json"
    path.write_text(
        json.dumps({"boot_id": "boot-a", "saved_at": 100.0, "pose": {"x": 1}}),
        encoding="utf-8",
    )
    assert select_same_boot_pose(load_cache(path), "boot-a", now=101.0) == (
        "same_boot_untrusted",
        None,
    )
