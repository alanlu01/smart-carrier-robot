# Split mapping workflow

The Raspberry Pi owns all robot hardware, odometry, robot TF, and
`slam_toolbox`. The VMware workstation only renders RViz. Both machines use
`ROS_DOMAIN_ID=30`.

Do not run Nav2/AMCL (`start_nav`) while mapping. SLAM Toolbox must be the only
publisher of `map -> odom`.

## Start mapping

On the Raspberry Pi, use separate terminals:

```bash
start_all
gen_map
map_teleop
```

`map_teleop` publishes keyboard commands directly to `/chassis_cmd_vel`. Start
at low speed and avoid fast rotations so the mecanum odometry and scan matcher
remain consistent.

On the VMware workstation:

```bash
gen_map
```

The VMware alias starts `display.launch.py` with the dedicated `mapping.rviz`;
it must not start a second SLAM, lidar, odometry, or robot-state publisher.

No initial pose is required while mapping. The robot's startup pose becomes the
map origin.

## Save a candidate map

Use a new name first; do not overwrite the current navigation map:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/dev_ws/src/smart-carrier-robot/packages/smart_delivery_core/maps/room_candidate

ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/kj0921/dev_ws/src/smart-carrier-robot/packages/smart_delivery_core/maps/room_candidate'}"
```

The first command creates the Nav2 `.pgm` and `.yaml`. The second preserves the
SLAM pose graph for later continuation. Replace `room.pgm` and `room.yaml` only
after the candidate map has been reviewed and backed up.
