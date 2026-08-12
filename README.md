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

## Remaining hardware validation

- Calibrate current thresholds against the actual power banks.
- Read the robot pose from TF and include it in heartbeats.
- Add backend cancellation during navigation and a local offline result queue.
- Confirm insertion/removal before marking borrow/return service complete.
