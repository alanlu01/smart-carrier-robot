# Cloud API integration

## Ownership

The integration keeps one owner for each responsibility:

- `power_monitor` owns INA3221 access, filtering, slot classification, charge estimation, and sensor health.
- `smart_delivery_core` owns order feasibility, Nav2 execution, standby behavior, and task-result publication.
- `smart_carrier_api` owns authenticated HTTP requests and translation between the backend and ROS topics.

The former `smart_carrier_robot` prototype package was reduced to `smart_carrier_api`. Its duplicate INA3221 and Nav2 nodes were removed.

## ROS interfaces

### `power_status` (`std_msgs/String`)

The payload is a JSON object with `ch1`, `ch2`, and `ch3`. Each channel contains:

```json
{
  "slot": 1,
  "bank_id": "PB-01",
  "status": "ready",
  "current": 0.1,
  "charge": 96,
  "sensor_ok": true
}
```

Canonical states are:

- `empty`: no measurable bank current (`<= 0.0005 A`).
- `full`: inserted bank at trickle current (`> 0.0005 A` and `< 0.05 A`).
- `ready`: borrowable bank approaching full charge (`>= 0.05 A` and `< 0.4 A`).
- `low`: bank charging at high current (`>= 0.4 A`).
- `unknown`: no valid INA3221 sample is available.

An I²C failure publishes `sensor_ok: false`. `smart_delivery_core` retains the last healthy inventory and pauses new dispatches after ten seconds without a healthy update; it does not interpret an I²C failure as an empty slot.

### `order` (`std_msgs/String`)

`smart_carrier_api` publishes the complete claimed-task JSON returned by the backend. `smart_delivery_core` accepts backend `location.x/y/yaw` coordinates and retains its original name-based lookup only for legacy local messages.

The bridge will not claim a task until it has healthy `power_status` data and an active subscriber on `order`. This prevents a task from being claimed before the delivery process is ready to receive it.

### `/smart_carrier/task_result` (`std_msgs/String`)

`smart_delivery_core` publishes:

```json
{"task_id": "...", "status": "done", "note": "Nav2 goal reached"}
```

The API bridge reports that result to the backend and only then releases its active task.

## Supported tasks

- `borrow`: requires a `ready` or `full` slot meeting `required_charge`.
- `return`: requires an empty slot.
- `delivery`, `navigation`, and `callbot`: navigate without consuming a power-bank slot.
- `quantity` must currently be `1`; larger quantities are rejected and reported as failed instead of being partially fulfilled.

Physical insertion/removal confirmation remains future work. A successful result currently means that Nav2 reached the destination.

## Runtime

```bash
source /opt/ros/jazzy/setup.bash
source ~/dev_ws/install/setup.bash
source ~/.config/smart-carrier/robot.env

ros2 run power_monitor ina3221_node --ros-args \
  --params-file ~/dev_ws/install/power_monitor/share/power_monitor/config/power_monitor.yaml
ros2 run smart_delivery_core smart_delivery
ros2 launch smart_carrier_api api_bridge.launch.py
```

Only one INA3221 node and one delivery node may run. The retired `smart_carrier_ws` must not be sourced or launched with `dev_ws`.
