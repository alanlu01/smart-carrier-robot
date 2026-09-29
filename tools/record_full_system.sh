#!/usr/bin/env bash
# Foreground recording: no stdin-reading background ros2 process / SIGTTIN.
set -e
source /opt/ros/jazzy/setup.bash
source /home/kj0921/dev_ws/install/setup.bash
set -u
RECORD_ROOT="${ROBOT_TEST_LOG_ROOT:-/home/kj0921/test_logs}"
mkdir -p "$RECORD_ROOT"
RECORD_DIR="$(mktemp -d "$RECORD_ROOT/$(date +%Y%m%d_%H%M%S)_full_system_XXXXXX")"
QOS_FILE="$(ros2 pkg prefix smart_delivery_core)/share/smart_delivery_core/config/recording_qos.yaml"
printf 'Recording to %s; finish with Ctrl+C in this terminal.\n' "$RECORD_DIR"
# Timing-only health topics are small. Do not record raw camera images.
TOPICS=(
  /scan /scan_filtered /stm32_data /odom /amcl_pose /initialpose /tf /tf_static
  /map /plan /cmd_vel_nav /cmd_vel_smoothed /cmd_vel /chassis_cmd_vel
  /collision_monitor_state /localization/state /localization/ready
  /localization/motion_inhibited /smart_carrier/delivery_navigation_active
  /power_status /vision/semantic_info /semantic/speed_multiplier /visual_glass
  /vision/pipeline_health /semantic/pipeline_health /chassis/feedback_health
  /delivery/standby_state /order /smart_carrier/task_state /smart_carrier/task_result
  /smart_carrier/task_result_ack /diagnostics
  /smart_carrier/task_cancel
  /navigate_to_pose/_action/feedback /navigate_to_pose/_action/status
  /spin/_action/feedback /spin/_action/status
  /local_costmap/costmap /local_costmap/costmap_updates /local_costmap/published_footprint
  /global_costmap/costmap /global_costmap/costmap_updates
)
trap ':' INT
set +e
ros2 bag record --qos-profile-overrides-path "$QOS_FILE" \
  -o "$RECORD_DIR/rosbag" "${TOPICS[@]}" 2>&1 | tee "$RECORD_DIR/rosbag_console.log"
RECORD_EXIT="${PIPESTATUS[0]}"
printf '%s\n' "$RECORD_EXIT" > "$RECORD_DIR/rosbag_exit_code.txt"
printf 'Recording closed; ros2 exit code %s; directory %s\n' "$RECORD_EXIT" "$RECORD_DIR"
exit "$RECORD_EXIT"
