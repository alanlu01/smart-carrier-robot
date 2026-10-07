#!/usr/bin/env bash
# Observations only: does not start/stop robot nodes or change parameters.
set -e
source "${ROBOT_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
source "${ROBOT_WS_SETUP:-/home/kj0921/dev_ws/install/setup.bash}"
set -u
RECORD_ROOT="${ROBOT_TEST_LOG_ROOT:-/home/kj0921/test_logs}"
mkdir -p "$RECORD_ROOT"
RECORD_DIR="$(mktemp -d "$RECORD_ROOT/$(date +%Y%m%d_%H%M%S)_full_system_XXXXXX")"
CORE_SHARE="$(ros2 pkg prefix smart_delivery_core)/share/smart_delivery_core"
QOS_FILE="$CORE_SHARE/config/recording_qos.yaml"
test -r "$QOS_FILE" || { printf 'Missing QoS file: %s\n' "$QOS_FILE" >&2; exit 1; }
RECORD_HELP="$(ros2 bag record --help)"
for FLAG in --include-hidden-topics --disable-keyboard-controls; do
  if [[ "$RECORD_HELP" != *"$FLAG"* ]]; then
    printf 'Recorder does not support %s; inspect installed rosbag2 first.\n' "$FLAG" >&2
    exit 1
  fi
done
START_TIME="$(date --iso-8601=seconds)"
MONITOR_PIDS=()
cleanup() {
  local pid
  for pid in "${MONITOR_PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
  for pid in "${MONITOR_PIDS[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
# Preserve every topic from the full-system checklist. No raw camera frames.
TOPICS=(
  /scan /scan_filtered /stm32_data /odom /amcl_pose /initialpose
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
  /localization/motion_inhibited
  /localization/recovery_motion_lease /localization/recovery_motion_guard
  /smart_carrier/dispatch_sync_request /smart_carrier/dispatch_sync_state
  /smart_carrier/result_sync_state
  /smart_carrier/optional_idle_state /smart_carrier/task_admission
  /vehicle_battery/verified_status
  /smart_carrier/delivery_navigation_active /delivery/standby_state
  /power_status /vehicle_battery_status
  /vision/semantic_info /visual_glass /semantic/speed_multiplier
  /vision/pipeline_health /semantic/pipeline_health /chassis/feedback_health
  /order /smart_carrier/task_state
  /smart_carrier/task_result /smart_carrier/task_result_ack
  /smart_carrier/task_cancel /diagnostics /rosout
)
printf '%s\n' "${TOPICS[@]}" > "$RECORD_DIR/requested_topics.txt"
printf '%s\n' "$RECORD_DIR" > "$RECORD_ROOT/latest_recording_path.txt"
printf '%s\t%s\n' "$START_TIME" recording_start > "$RECORD_DIR/events.tsv"
printf '%s\n' "$RECORD_HELP" > "$RECORD_DIR/recorder_help.txt"
cp "$QOS_FILE" "$RECORD_DIR/recording_qos.yaml"
mkdir -p "$RECORD_DIR/config_snapshot"
# Explicit whitelist; never copy robot.env or dump API credentials.
for CONFIG in my_nav2_params.yaml slam_mapping.yaml my_laser_filter.yaml optional_idle_features.yaml; do
  if [[ -f "$CORE_SHARE/config/$CONFIG" ]]; then
    cp "$CORE_SHARE/config/$CONFIG" "$RECORD_DIR/config_snapshot/"
  fi
done
if POWER_PREFIX="$(ros2 pkg prefix power_monitor 2>/dev/null)"; then
  POWER_CONFIG="$POWER_PREFIX/share/power_monitor/config/power_monitor.yaml"
  if [[ -r "$POWER_CONFIG" ]]; then
    cp "$POWER_CONFIG" "$RECORD_DIR/config_snapshot/power_monitor.yaml"
  fi
fi
{
  printf 'start=%s\ncore_share=%s\n' "$START_TIME" "$CORE_SHARE"
  uname -a
  for KEY in ROS_DISTRO ROS_DOMAIN_ID RMW_IMPLEMENTATION ROS_LOCALHOST_ONLY \
      ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS CYCLONEDDS_URI; do
    printf '%s=%s\n' "$KEY" "${!KEY-}"
  done
  df -h "$RECORD_ROOT"
  if command -v timedatectl >/dev/null; then
    timeout 3 timedatectl show -p NTPSynchronized -p Timezone || true
  fi
  REPO="${ROBOT_SOURCE_REPO:-/home/kj0921/dev_ws/src/smart-carrier-robot}"
  if git -C "$REPO" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    git -C "$REPO" rev-parse HEAD
    git -C "$REPO" status --short
    git -C "$REPO" diff --stat
  fi
  sha256sum "$RECORD_DIR/config_snapshot/"* "$RECORD_DIR/recording_qos.yaml" 2>/dev/null || true
} > "$RECORD_DIR/environment.txt" 2>&1

graph_snapshot() {
  printf '\n--- %s ---\n' "$(date --iso-8601=ns)"
  timeout 8 ros2 node list || true
  timeout 8 ros2 topic list --include-hidden-topics -t || true
}
printf 'Preparing snapshot; recording has NOT started yet.\n'
graph_snapshot > "$RECORD_DIR/graph_start.txt" 2>&1
{
  for TOPIC in /scan /scan_filtered /stm32_data /odom; do
    printf '\n--- %s ---\n' "$TOPIC"
    timeout 3 ros2 topic info "$TOPIC" -v || true
  done
} > "$RECORD_DIR/topic_endpoints_start.txt" 2>&1

system_monitor() {
  local path iface
  while true; do
    printf '\n--- sample %s epoch=%s ---\n' "$(date --iso-8601=ns)" "$(date +%s.%N)"
    printf '[cpu_ticks]\n'; awk '/^cpu/ {print}' /proc/stat
    printf '[load]\n'; head -n 1 /proc/loadavg
    printf '[memory_kB]\n'
    awk '/^(MemTotal|MemAvailable|SwapTotal|SwapFree|Dirty|Writeback):/ {print}' /proc/meminfo
    for path in /proc/pressure/cpu /proc/pressure/memory /proc/pressure/io \
        /sys/class/thermal/thermal_zone*/temp \
        /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq; do
      [[ -r "$path" ]] || continue
      printf '[%s]\n' "$path"; head -n 3 "$path"
    done
    printf '[network_bytes_errors_drops]\n'; head -n 30 /proc/net/dev
    if [[ -r /proc/net/wireless ]]; then
      printf '[wifi_quality]\n'; head -n 20 /proc/net/wireless
    fi
    for path in /sys/class/net/*/wireless; do
      [[ -d "$path" ]] || continue
      iface="${path%/wireless}"; iface="${iface##*/}"
      printf '[wifi_link %s]\n' "$iface"
      if command -v iw >/dev/null; then timeout 2 iw dev "$iface" link || true; fi
    done
    if command -v vcgencmd >/dev/null; then
      printf '[pi_throttling]\n'; timeout 2 vcgencmd get_throttled || true
    fi
    sleep 5
  done
}
system_monitor > "$RECORD_DIR/system_health.log" 2>&1 < /dev/null &
MONITOR_PIDS+=("$!")
if command -v top >/dev/null; then
  LC_ALL=C nice -n 10 top -b -d 5 -w 180 > "$RECORD_DIR/process_top.log" 2>&1 < /dev/null &
  MONITOR_PIDS+=("$!")
fi

printf '\nRecording to %s\nWait for recorder subscriptions, then start testing.\nFinish with Ctrl+C in THIS terminal; do not use Ctrl+Z.\n' "$RECORD_DIR"
# Recorder stays foreground; disabling keyboard controls also prevents SIGTTIN.
trap ':' INT
set +e
ros2 bag record --include-hidden-topics --disable-keyboard-controls \
  --qos-profile-overrides-path "$QOS_FILE" \
  -o "$RECORD_DIR/rosbag" "${TOPICS[@]}" 2>&1 | tee "$RECORD_DIR/rosbag_console.log"
RECORD_PIPESTATUS=("${PIPESTATUS[@]}")
RECORD_EXIT="${RECORD_PIPESTATUS[0]}"
CONSOLE_EXIT="${RECORD_PIPESTATUS[1]}"
printf '%s\n' "$RECORD_EXIT" > "$RECORD_DIR/rosbag_exit_code.txt"
printf '%s\n' "$CONSOLE_EXIT" > "$RECORD_DIR/rosbag_console_exit_code.txt"
END_TIME="$(date --iso-8601=seconds)"
printf '%s\t%s\n' "$END_TIME" recording_stop >> "$RECORD_DIR/events.tsv"
cleanup
MONITOR_PIDS=()
graph_snapshot > "$RECORD_DIR/graph_end.txt" 2>&1
timeout 20 ros2 bag info "$RECORD_DIR/rosbag" > "$RECORD_DIR/rosbag_info.txt" 2>&1
INFO_EXIT="$?"
printf '%s\n' "$INFO_EXIT" > "$RECORD_DIR/rosbag_info_exit_code.txt"
if command -v journalctl >/dev/null; then
  timeout 8 journalctl -k --since "$START_TIME" --until "$END_TIME" --no-pager \
    > "$RECORD_DIR/kernel.log" 2>&1
  timeout 8 journalctl -u NetworkManager --since "$START_TIME" --until "$END_TIME" --no-pager \
    > "$RECORD_DIR/network_manager.log" 2>&1
fi
printf 'Recording closed; ros2 exit code %s; directory %s\n' "$RECORD_EXIT" "$RECORD_DIR"
if [[ "$INFO_EXIT" -ne 0 ]]; then
  printf 'WARNING: bag info failed; inspect rosbag_info.txt. Do not delete/reindex the original bag yet.\n' >&2
fi
if [[ "$CONSOLE_EXIT" -ne 0 ]]; then
  printf 'WARNING: console log writer failed (exit=%s).\n' "$CONSOLE_EXIT" >&2
  [[ "$RECORD_EXIT" -ne 0 ]] || exit "$CONSOLE_EXIT"
fi
exit "$RECORD_EXIT"
