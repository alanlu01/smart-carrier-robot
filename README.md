# Smart Carrier Robot

ROS2 package for INA3221 three-slot monitoring, outbound cloud task polling, and Nav2 execution.

The previous hard-coded `pending_orders` loop is replaced by this flow:

```text
FastAPI claim endpoint → dispatch_bridge_node → /smart_carrier/task
→ navigator_node / Nav2 → /smart_carrier/task_result → FastAPI
```

## Raspberry Pi setup

Place this Repository in a ROS2 workspace `src/` directory, then:

```bash
sudo apt install python3-smbus2
colcon build --packages-select smart_carrier_robot
source install/setup.bash
export SMART_CARRIER_API_URL=https://api.your-domain.example
export SMART_CARRIER_ROBOT_ID=R1
export SMART_CARRIER_ROBOT_TOKEN=replace_me
ros2 launch smart_carrier_robot smart_carrier.launch.py
```

Map coordinates are loaded from the backend `locations` table. A task with missing `x` or `y` is rejected safely instead of navigating to an assumed origin.

## Topics

- `power_status` — INA3221 JSON for three channels
- `/smart_carrier/task` — claimed cloud task
- `/smart_carrier/task_result` — Nav2 result sent back to the cloud

## Validate cloud locations against the active map

The packaged `maps/room.yaml` map is 145×144 pixels at 0.05 m/pixel with origin
`(-4.879, -0.969)`. Its world bounds are X `[-4.879, 2.371)` and Y
`[-0.969, 6.231)`.

After the map server is active, validate every cloud location against `/map`:

```bash
ros2 run smart_carrier_robot location_validator_node
```

The validator rejects coordinates outside the map, unknown/occupied cells, and targets
without 0.35 m clearance. To include Nav2 inflation costs, validate the published global
costmap instead:

```bash
ros2 run smart_carrier_robot location_validator_node --ros-args \
  -p map_topic:=/global_costmap/costmap -p clearance_m:=0.0 -p max_cost:=0
```

The original cloud seed coordinates place E1, F1, and S1 outside this map. Do not claim
real navigation tasks until all service points are measured in RViz and this validator
reports `PASS` for every location.

To record a safe service point, start the calibrator and then use RViz `2D Goal Pose` to
click the target position and drag its desired arrival heading:

```bash
ros2 run smart_carrier_robot location_calibrator_node --ros-args \
  -p location_code:=A1
```

The node rejects unsafe clicks and prints one machine-readable `CALIBRATION` JSON line for
an accepted pose. Repeat for E1, F1, and S1. These values can then be applied from the API
VM with the dry-run location calibration command documented in the API Repository.

## Remaining hardware validation

- Calibrate current thresholds against the actual power banks.
- Read the robot pose from TF and include it in heartbeats.
- Add backend cancellation during navigation and a local offline result queue.
- Confirm insertion/removal before marking borrow/return service complete.
