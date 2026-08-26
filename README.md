# Smart Carrier Robot

ROS 2 monorepo for the Raspberry Pi side of the Smart Carrier project. It starts from a byte-for-byte verified snapshot of `~/dev_ws/src` taken on 2026-08-20 and integrates the cloud bridge without duplicating the existing power or navigation nodes.

## Repository layout

- `packages/smart_delivery_core` — current delivery state machine, navigation, voice, buttons, and screen integration.
- `packages/power_monitor` — INA3221 readings and the canonical power-bank slot states.
- `packages/hailo_vision` — Hailo-based vision package and model.
- `packages/vm_robot_model` — URDF, meshes, and robot description launch files.
- `packages/smart_carrier_api` — cloud HTTP to ROS 2 bridge only.
- `vendor/rplidar_ros-dev-ros2` — exact customized RPLIDAR package from the Raspberry Pi.
- `dependencies.repos` — exact upstream revisions for `camera_ros` and `rf2o_laser_odometry`.
- `docs/raspberry-pi-baseline.md` — provenance, external dependencies, backup, and known limitations.

The runtime responsibility is deliberately split as follows:

```text
power_monitor ── power_status ──► smart_delivery_core
       └──────── power_status ──► smart_carrier_api ── heartbeat ──► cloud
cloud ── claimed task ──► smart_carrier_api ── order ──► smart_delivery_core
cloud ◄── task result ─── smart_carrier_api ◄── /smart_carrier/task_result
```

`smart_carrier_api` does not access I²C and does not start a Nav2 navigator. The old standalone `smart_carrier_ws` is retained on the Raspberry Pi only as a temporary rollback copy and must not be launched together with this workspace.

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

## Cloud bridge

Load the protected environment file, then launch the API-only bridge:

```bash
source ~/.config/smart-carrier/robot.env
ros2 launch smart_carrier_api api_bridge.launch.py
```

The power monitor publishes three `ch1`–`ch3` objects on `power_status`. Each object contains `slot`, `bank_id`, `status`, `voltage`, `current`, `charge`, `sensor_ok`, and `enabled`. The measured bus voltage is the shared 3S vehicle supply and is diagnostic only; slot presence is determined from current using hysteresis and six-sample confirmation. Canonical states are `empty`, `low` (charging), `full`, `unknown`, and `disabled`. Charging percentage cannot be inferred from input current, so `charge` is `null` while charging, `0` when empty, and `100` when full. Borrow and return tasks are completed only after the assigned slot confirms the physical removal or insertion; the user is warned after 30 seconds and the task fails after 60 seconds.

## Secrets

Runtime credentials are deliberately excluded. Keep `SMART_CARRIER_API_URL`, `SMART_CARRIER_ROBOT_ID`, and `SMART_CARRIER_ROBOT_TOKEN` in `~/.config/smart-carrier/robot.env` with mode `0600`; never commit that file or the GitHub deploy key.

## Recovery points

- Original cloud prototype: commit `53ca8af` on `main` before this import.
- Verified Raspberry Pi backup: `dev_ws_20260818_130438.tar.gz`.
- Backup SHA-256: `3F152A63329F90E26B67EA8C6694F76D0A053B7C8A5DED6114F1896F67F2BB58`.
