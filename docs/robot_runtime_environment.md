# Robot ROS runtime environment

The Raspberry Pi uses Fast DDS shared memory for local node-to-node traffic and
UDPv4 for RViz and other ROS 2 participants on the LAN. Both transports must
remain enabled; do not set `ROS_LOCALHOST_ONLY=1`.

After building the workspace, interactive robot shells load:

```bash
export ROS_DOMAIN_ID=30
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/dev_ws/install/smart_delivery_core/share/smart_delivery_core/config/fastdds_robot.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
```

The same `ROS_DOMAIN_ID` must be used by the VMware development machine.
