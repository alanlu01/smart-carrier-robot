# Raspberry Pi baseline

This document records the source and runtime baseline used to place the existing robot under Git version control. No files on the Raspberry Pi were changed while preparing this baseline.

## Source verification

- Raspberry Pi source: `/home/kj0921/dev_ws/src`
- Verification date: 2026-08-20 (Asia/Taipei)
- Compared with the previously downloaded local latest copy.
- Excluded only nested `.git` metadata and generated `__pycache__` entries.
- Files in each copy: 214
- Differing files: 0

The project-owned packages copied without functional modification are:

- `hailo_vision`
- `power_monitor`
- `smart_delivery_core`
- `vm_robot_model`

The exact customized `rplidar_ros-dev-ros2` directory is stored under `vendor/`. It differs from the nearest upstream ROS 2 revision in launch defaults, dependency declarations, and retained launch/RViz files, so pinning only the upstream repository would not reproduce the running workspace.

## Pinned external source

`dependencies.repos` restores these clean upstream packages at the revisions found on the Raspberry Pi:

- `camera_ros`: `ac71910ca47bd2110a26892a5bc1b2be664abe29`
- `rf2o_laser_odometry`: `313bb4c4123bcc0cc2e042f278312b19a3c46f31`

## Platform

- User: `kj0921`
- Architecture: `aarch64`
- ROS distribution: Jazzy
- ROS domain ID used by the existing shell environment: `30`

## External runtime dependencies

These machine-level components are required but are not stored in this repository:

- `/etc/udev/rules.d/99-robot-serial.rules` for stable serial device permissions/names.
- Raspberry Pi hardware configuration in `/boot/firmware/config.txt`.
- Custom libcamera libraries under `/usr/local/lib/aarch64-linux-gnu/` (observed version `0.7.1`).
- HailoRT runtime and driver (observed version `4.24.0`).
- Python `hailo_platform` installed in the user's local Python environment.
- The STM32 controller firmware; its source was not found on the Raspberry Pi.
- `~/.config/smart-carrier/robot.env`, which contains cloud runtime settings and must remain secret.
- The repository deploy private key, which must remain outside Git.

## Backup and rollback

Before integration work, the complete workspace was archived at:

- Raspberry Pi: `/home/kj0921/backups/dev_ws_20260818_130438.tar.gz`
- Local recovery copy: `raspberry_pi_backups/dev_ws_20260818_130438.tar.gz`
- SHA-256: `3F152A63329F90E26B67EA8C6694F76D0A053B7C8A5DED6114F1896F67F2BB58`

Git commits provide source-level rollback after this baseline. The tar archive remains the recovery point for generated workspace files and other content that Git intentionally excludes.

## Known baseline limitations

- Several package manifests do not yet declare all runtime dependencies.
- The primary bringup does not start `robot_state_publisher`.
- A mapping launch file references a `mapping.rviz` file that is not present.
- Backend location coordinates still need to be reconciled with the robot's room map.
- INA3221 read errors can currently appear as a zero-current sample.
- The cloud prototype duplicates INA3221 and navigation responsibilities already present in the existing packages.
- No systemd service currently owns the complete robot bringup.

These observations are recorded rather than fixed in the baseline commit so that later functional changes remain reviewable and independently reversible.
