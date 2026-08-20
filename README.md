# Smart Carrier Robot

ROS 2 monorepo for the Raspberry Pi side of the Smart Carrier project. This branch starts from a byte-for-byte verified snapshot of `~/dev_ws/src` taken on 2026-08-20 and preserves the current robot behavior before the cloud bridge is consolidated into the existing packages.

## Repository layout

- `packages/smart_delivery_core` — current delivery state machine, navigation, voice, buttons, and screen integration.
- `packages/power_monitor` — current INA3221 power monitoring and power-bank state topics.
- `packages/hailo_vision` — Hailo-based vision package and model.
- `packages/vm_robot_model` — URDF, meshes, and robot description launch files.
- `packages/smart_carrier_robot` — initial cloud API bridge prototype retained for the migration.
- `vendor/rplidar_ros-dev-ros2` — exact customized RPLIDAR package from the Raspberry Pi.
- `dependencies.repos` — exact upstream revisions for `camera_ros` and `rf2o_laser_odometry`.
- `docs/raspberry-pi-baseline.md` — provenance, external dependencies, backup, and known limitations.

The prototype `smart_carrier_robot` package currently contains its own INA3221 and navigator nodes. Do not launch those nodes together with the corresponding nodes in `power_monitor` and `smart_delivery_core`; they are retained only as migration input. The planned end state is one API bridge package using the existing power and navigation implementations.

## Restore the source workspace

Install `vcstool`, clone this repository into `src`, and fetch the two pinned external packages:

```bash
mkdir -p ~/dev_ws/src
git clone https://github.com/alanlu01/smart-carrier-robot.git ~/dev_ws/src/smart-carrier-robot
cd ~/dev_ws/src
vcs import . < smart-carrier-robot/dependencies.repos
```

Build on the Raspberry Pi with ROS 2 Jazzy:

```bash
cd ~/dev_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The current package manifests do not yet declare every system and Python dependency. Reproduce the existing Raspberry Pi runtime described in the baseline document before expecting a clean machine build.

## Secrets

Runtime credentials are deliberately excluded. Keep `SMART_CARRIER_API_URL`, `SMART_CARRIER_ROBOT_ID`, and `SMART_CARRIER_ROBOT_TOKEN` in `~/.config/smart-carrier/robot.env` with mode `0600`; never commit that file or the GitHub deploy key.

## Recovery points

- Original cloud prototype: commit `53ca8af` on `main` before this import.
- Verified Raspberry Pi backup: `dev_ws_20260818_130438.tar.gz`.
- Backup SHA-256: `3F152A63329F90E26B67EA8C6694F76D0A053B7C8A5DED6114F1896F67F2BB58`.
