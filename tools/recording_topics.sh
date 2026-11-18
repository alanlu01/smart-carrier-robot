#!/usr/bin/env bash
# Sourced by the recorder; listing a profile never starts ROS or writes files.
# Preserve every topic from the full-system checklist. No raw camera frames.
FULL_TOPICS=(
  /scan /scan_filtered /stm32_data /chassis/wheel_sample /odom /amcl_pose /initialpose
  /tf /tf_static /map /particle_cloud
  /plan /behavior_tree_log
  /cmd_vel_nav /cmd_vel_smoothed /cmd_vel /chassis_cmd_vel
  /collision_monitor_state
  /navigate_to_pose/_action/feedback /navigate_to_pose/_action/status
  /navigate_through_poses/_action/feedback /navigate_through_poses/_action/status
  /spin/_action/feedback /spin/_action/status
  /local_costmap/costmap /local_costmap/costmap_updates
  /local_costmap/published_footprint
  /global_costmap/costmap /global_costmap/costmap_updates
  /global_costmap/published_footprint
  /localization/state /localization/ready
  /localization/speed_limit
  /localization/motion_inhibited
  /localization/recovery_motion_lease /localization/recovery_motion_guard
  /smart_carrier/dispatch_sync_request /smart_carrier/dispatch_sync_state
  /smart_carrier/result_sync_state
  /smart_carrier/optional_idle_state /smart_carrier/task_admission
  /vehicle_battery/verified_status
  /smart_carrier/delivery_navigation_active /delivery/standby_state /delivery/station_state
  /delivery/station_controller_state
  /delivery/navigation_policy
  /delivery/startup_state
  /chassis/odom_local /chassis/odometry_transport_health
  /chassis/serial_rx_health /chassis/serial_tx_health /chassis/serial_feedback /navigation/pipeline_health
  /chassis/serial_io_diagnostics
  /navigation/diagnostics
  /power_status /vehicle_battery_status
  /vision/semantic_info /visual_glass /semantic/speed_multiplier
  /vision/pipeline_health /semantic/pipeline_health /chassis/feedback_health
  /lidar/pipeline_health
  /order /smart_carrier/task_state
  /bond /amcl/transition_event /map_server/transition_event
  /controller_server/transition_event /bt_navigator/transition_event /behavior_server/transition_event
  /smart_carrier/task_result /smart_carrier/task_result_ack
  /smart_carrier/task_cancel /diagnostics /rosout
)

select_recording_topics() {
  TOPICS=()
  local topic
  for topic in "${FULL_TOPICS[@]}"; do
    if [[ "$1" == daily ]]; then
      case "$topic" in
        /global_costmap/costmap|/global_costmap/costmap_updates|/particle_cloud|/bond) continue ;;
      esac
    fi
    TOPICS+=("$topic")
  done
}
