import json
import math
import os
import time
import uuid
from collections import deque

import rclpy
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import Spin
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration as RclpyDuration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from smart_delivery_core.localization_health import (
    EvidenceWaitClock,
    amcl_timeout_requires_recovery,
    convergence_grace_allowed,
    covariance_is_converging,
    damped_heading_command,
    heading_correction,
    heading_error_is_improving,
    localization_suspicion_required,
    map_match_status,
    minimum_verification_wait,
    occupancy_match_score,
    pose_is_near,
    pose_jump,
    pose_quality,
    quaternion_to_yaw,
    scan_pipeline_status,
    select_scan_samples,
    smoothed_map_score,
    should_extend_global_recovery,
    suspect_requires_recovery,
    transient_sensor_recovery_qualified,
    update_stability_samples,
)
from smart_delivery_core.localization_persistence import (
    load_cache,
    read_boot_id,
    select_same_boot_pose,
    write_cache,
)
from smart_delivery_core.pipeline_health import DurationMetrics, ReceiptMetrics, ScanPairMetrics
from smart_delivery_core.recovery_translation import (
    lateral_progress, lateral_speed, load_self_mask, swept_scan_report,
)
from smart_delivery_core.wheel_feedback import WheelSampleAssembler


STATE_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
SENSOR_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class LocalizationManager(Node):
    """Seed AMCL, gate navigation, and perform conservative staged recovery."""

    def __init__(self):
        super().__init__("localization_manager")
        self._declare_parameters()

        self.initial_x = float(self.get_parameter("initial_pose.x").value)
        self.initial_y = float(self.get_parameter("initial_pose.y").value)
        self.initial_yaw = float(self.get_parameter("initial_pose.yaw").value)
        self.auto_initialize = bool(self.get_parameter("auto_initialize").value)
        self.reuse_same_boot_pose = bool(
            self.get_parameter("reuse_same_boot_pose").value
        )
        self.pose_cache_path = str(self.get_parameter("pose_cache_path").value)
        self.pose_cache_max_age = float(
            self.get_parameter("pose_cache_max_age_sec").value
        )
        self.pose_cache_write_period = max(
            1.0, float(self.get_parameter("pose_cache_write_period_sec").value)
        )
        self.boot_id = read_boot_id()
        self.initial_pose_source = "fixed_power_on"
        self.same_boot_pose_untrusted = False
        self._configure_startup_pose()
        self.motion_cmd_topic = str(self.get_parameter("motion_cmd_topic").value)
        self.raw_scan_topic = str(self.get_parameter("raw_scan_topic").value)
        self.filtered_scan_topic = str(
            self.get_parameter("filtered_scan_topic").value
        )
        self.scan_timeout = float(self.get_parameter("scan_timeout_sec").value)
        self.scan_timeout_grace = float(
            self.get_parameter("scan_timeout_grace_sec").value
        )
        self.amcl_pose_timeout = float(
            self.get_parameter("amcl_pose_timeout_sec").value
        )
        self.amcl_stationary_grace = float(
            self.get_parameter("amcl_stationary_grace_sec").value
        )
        self.amcl_nomotion_refresh = float(
            self.get_parameter("amcl_nomotion_refresh_sec").value
        )
        self.verification_nomotion_refresh = max(
            0.2, float(self.get_parameter("verification_nomotion_refresh_sec").value)
        )
        self.pipeline_wait_timeout = max(
            1.0, float(self.get_parameter("recovery_pipeline_wait_timeout_sec").value)
        )
        self.odom_timeout = float(self.get_parameter("odom_timeout_sec").value)
        self.verify_timeout = float(self.get_parameter("verify_timeout_sec").value)
        self.verify_max_wait = max(
            self.verify_timeout,
            float(self.get_parameter("verify_max_wait_sec").value),
        )
        self.stable_samples_required = int(self.get_parameter("stable_samples").value)
        self.transient_recovery_samples_required = max(
            1, int(self.get_parameter("transient_recovery_samples").value)
        )
        self.transient_recovery_max = max(
            1.0, float(self.get_parameter("transient_recovery_max_sec").value)
        )
        self.healthy_xy_std = float(self.get_parameter("healthy_xy_std").value)
        self.healthy_yaw_std = float(self.get_parameter("healthy_yaw_std").value)
        self.critical_xy_std = float(self.get_parameter("critical_xy_std").value)
        self.critical_yaw_std = float(self.get_parameter("critical_yaw_std").value)
        self.seed_xy_tolerance = float(self.get_parameter("seed_xy_tolerance").value)
        self.seed_yaw_tolerance = float(self.get_parameter("seed_yaw_tolerance").value)
        self.suspect_hold = float(self.get_parameter("suspect_hold_sec").value)
        self.suspect_max = max(
            self.suspect_hold,
            float(self.get_parameter("suspect_max_sec").value),
        )
        self.suspect_promising_max = max(
            self.suspect_max,
            float(self.get_parameter("suspect_promising_max_sec").value),
        )
        self.cancel_grace = float(self.get_parameter("cancel_grace_sec").value)
        self.local_wait = float(self.get_parameter("local_recovery_wait_sec").value)
        self.enable_small_sweep = bool(self.get_parameter("enable_small_sweep").value)
        self.pre_global_wait = float(self.get_parameter("pre_global_wait_sec").value)
        self.lateral_enabled = bool(self.get_parameter("lateral_recovery_enabled").value)
        self.lateral_distance = float(self.get_parameter("lateral_recovery_distance").value)
        self.lateral_cap = float(self.get_parameter("lateral_recovery_speed").value)
        if not 0 < self.lateral_distance <= 0.30 or not 0 < self.lateral_cap <= 0.25:
            raise ValueError("Recovery translation exceeds 0.30 m / 0.25 m/s limits")
        self.lateral_half_x = float(self.get_parameter("lateral_footprint_half_x").value)
        self.lateral_half_y = float(self.get_parameter("lateral_footprint_half_y").value)
        if min(self.lateral_half_x, self.lateral_half_y) <= 0:
            raise ValueError("Recovery footprint dimensions must be positive")
        self.self_mask_config_file = str(self.get_parameter('self_mask_config_file').value)
        if not self.self_mask_config_file:
            self.self_mask_config_file = os.path.join(
                get_package_share_directory('smart_delivery_core'), 'config', 'my_laser_filter.yaml'
            )
        self.lateral_self_mask = None
        try:
            self.lateral_self_mask = load_self_mask(
                self.self_mask_config_file, self.lateral_half_x, self.lateral_half_y
            )
        except (OSError, ValueError, TypeError, KeyError) as exc:
            self.get_logger().warning(f'自體遮罩無法驗證，橫移採原始保守檢查：{exc}')
        self.lateral_path_reports = {}
        self.local_max_wait = max(
            self.local_wait,
            float(self.get_parameter("local_recovery_max_wait_sec").value),
        )
        self.global_wait = float(self.get_parameter("global_recovery_wait_sec").value)
        self.global_max_wait = max(
            self.global_wait,
            float(self.get_parameter("global_recovery_max_wait_sec").value),
        )
        self.global_convergence_max_wait = max(
            self.global_max_wait,
            float(self.get_parameter('global_convergence_max_wait_sec').value),
        )
        self.small_spin_angle = float(self.get_parameter("small_spin_angle").value)
        self.full_spin_angle = float(self.get_parameter("full_spin_angle").value)
        self.spin_time_allowance = int(self.get_parameter("spin_time_allowance_sec").value)
        self.spin_heading_tolerance = float(
            self.get_parameter("spin_heading_tolerance").value
        )
        self.spin_heading_correction_gain = float(
            self.get_parameter("spin_heading_correction_gain").value
        )
        self.spin_heading_max_correction = float(
            self.get_parameter("spin_heading_max_correction").value
        )
        self.spin_heading_min_improvement = float(
            self.get_parameter("spin_heading_min_improvement").value
        )
        self.spin_settle = max(
            0.0, float(self.get_parameter("spin_settle_sec").value)
        )
        self.spin_settle_timeout = max(
            self.spin_settle,
            float(self.get_parameter("spin_settle_timeout_sec").value),
        )
        self.spin_heading_max_corrections = max(
            1, int(self.get_parameter("spin_heading_max_corrections").value)
        )
        self.pose_jump_distance = float(self.get_parameter("pose_jump_distance").value)
        self.pose_jump_angle = float(self.get_parameter("pose_jump_angle").value)
        self.external_move_distance = float(
            self.get_parameter("external_move_distance").value
        )
        self.external_move_angle = float(self.get_parameter("external_move_angle").value)
        self.auto_motion_recovery = bool(
            self.get_parameter("automatic_motion_recovery").value
        )
        self.map_match_min_score = float(self.get_parameter("map_match_min_score").value)
        self.map_match_recovery_score = float(
            self.get_parameter("map_match_recovery_score").value
        )
        self.map_match_critical_score = float(
            self.get_parameter("map_match_critical_score").value
        )
        self.map_match_period = float(
            self.get_parameter("map_match_check_period_sec").value
        )
        self.map_match_max_beams = int(self.get_parameter("map_match_max_beams").value)
        self.map_match_min_beams = int(self.get_parameter("map_match_min_beams").value)
        self.map_match_neighborhood = int(
            self.get_parameter("map_match_neighborhood_cells").value
        )
        self.map_match_window_size = max(
            1, int(self.get_parameter("map_match_window_size").value)
        )
        self.map_match_window_min_samples = min(
            self.map_match_window_size,
            max(1, int(self.get_parameter("map_match_window_min_samples").value)),
        )
        self.map_match_ignore_below_range = float(
            self.get_parameter("map_match_ignore_below_range").value
        )
        self.map_match_min_sectors = max(
            1, int(self.get_parameter("map_match_min_sectors").value)
        )
        self.map_match_sector_total = max(
            self.map_match_min_sectors,
            int(self.get_parameter("map_match_sector_total").value),
        )
        self.map_match_stale_grace = float(
            self.get_parameter("map_match_stale_grace_sec").value
        )
        self.verification_pose_distance = float(
            self.get_parameter("verification_pose_distance").value
        )
        self.verification_pose_angle = float(
            self.get_parameter("verification_pose_angle").value
        )
        self.diagnostic_log_period = float(
            self.get_parameter("diagnostic_log_period_sec").value
        )
        # Six confirmations cannot fit in nine seconds at a 3s AMCL cadence.
        # Keep the evidence requirement and budget enough independent updates.
        minimum_wait = minimum_verification_wait(
            self.stable_samples_required,
            self.verification_nomotion_refresh,
            self.map_match_period,
            self.map_match_window_min_samples,
        )
        self.local_wait = max(self.local_wait, minimum_wait)
        self.global_wait = max(self.global_wait, minimum_wait)
        self.global_max_wait = max(self.global_max_wait, self.global_wait)
        self.global_convergence_max_wait = max(
            self.global_convergence_max_wait, self.global_max_wait
        )
        self.verify_timeout = max(self.verify_timeout, minimum_wait)

        self.state = "UNINITIALIZED"
        self.state_since = time.monotonic()
        self.state_reason = "node started"
        self.ready = False
        self.latest_scan_at = 0.0
        self.latest_raw_scan_at = 0.0
        self.latest_odom_at = 0.0
        self.latest_cmd_at = 0.0
        self.latest_pose = None
        self.latest_quality = None
        self.last_amcl_at = 0.0
        self.stable_samples = 0
        self.last_stable_map_score_at = 0.0
        self.verify_reference_required = True
        self.suspect_since = None
        self.map_match_unknown_since = None
        self.recovery_step = None
        self.recovery_step_started = 0.0
        self.recovery_reason = ""
        self.spin_sequence = []
        self.spin_in_progress = False
        self.spin_waiting_for_stop = False
        self.spin_settle_started = 0.0
        self.last_motion_command_at = 0.0
        self.odom_motion = False
        self.last_odom_motion_at = 0.0
        self.spin_restoring_heading = False
        self.spin_heading_corrections = 0
        self.spin_previous_heading_error = None
        self.spin_failure_reason = None
        self.spin_pending_failure_reason = None
        self.recovery_start_yaw = None
        self.spin_goal_target = 0.0
        self.spin_goal_started_at = 0.0
        self.commanded_motion = False
        self.stationary_odom_anchor = None
        self.stationary_amcl_anchor = None
        self.last_odom_pose = None
        self.manual_initial_pose_pending = False
        self.initial_pose_last_published = 0.0
        self.motion_inhibited = False
        self.map_message = None
        self.latest_scan = None
        self.latest_raw_scan = None
        self.latest_odom_stamp_ns = 0
        self.lateral_attempted = False
        self.lateral_phase = None
        self.lateral_token = None
        self.lateral_anchor = None
        self.lateral_direction = 0
        self.lateral_abort_reason = None
        self.lateral_started_at = 0.0
        self.lateral_stop_started_at = 0.0
        self.lateral_last_tick_at = 0.0
        self.lateral_last_pose = None
        self.lateral_stopped_since = None
        self.lateral_guard = {}
        self.lateral_guard_at = 0.0
        self.recovery_feedback = WheelSampleAssembler(0.25, time.monotonic())
        self.latest_map_score = None
        self.latest_map_score_raw = None
        self.latest_map_score_at = 0.0
        self.map_score_window = deque(maxlen=self.map_match_window_size)
        self.latest_map_near_fraction = 0.0
        self.latest_map_sector_count = 0
        self.latest_scan_stamp_age = None
        self.latest_scan_stamp_ns = 0
        self.map_match_tf_failures = 0
        self.last_map_match_error = ""
        self.last_diagnostic_log_at = 0.0
        self.last_pipeline_diagnostic_at = 0.0
        self.verification_pose_anchor = None
        self.last_map_match_check_at = 0.0
        self.last_nomotion_request_at = 0.0
        self.global_grace_announced = False
        self.recovery_diagnostics = {}
        self.transient_recovery_eligible = False
        self.transient_recovery_started_at = 0.0
        self.sensor_waiting_for_pipeline = False
        self.last_pose_cache_write_at = 0.0
        self.last_map_only_warning_at = 0.0
        self.observation_clock = EvidenceWaitClock()
        self.pipeline_clock = EvidenceWaitClock()
        self.observation_phase = None
        self.convergence_samples = deque(maxlen=10)
        self.last_convergence_map_at = 0.0
        self.convergence_grace_announced = False
        self.tick_last_at = None
        self.tick_max_gap = 0.0
        self.tick_max_duration = 0.0
        self.last_tick_metrics_at = time.monotonic()
        self.receipt_metrics = ReceiptMetrics()
        self.scan_pair_metrics = ScanPairMetrics()
        self.duration_metrics = DurationMetrics()
        self.transient_clock = EvidenceWaitClock()
        self.last_state_publish_at = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.ready_publisher = self.create_publisher(Bool, "/localization/ready", STATE_QOS)
        self.state_publisher = self.create_publisher(
            String, "/localization/state", STATE_QOS
        )
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", STATE_QOS
        )
        self.create_subscription(
            LaserScan,
            self.filtered_scan_topic,
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            self.raw_scan_topic,
            self._raw_scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(OccupancyGrid, "/map", self._map_callback, STATE_QOS)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_pose_callback, SENSOR_QOS
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self._external_initial_pose_callback,
            SENSOR_QOS,
        )
        self.create_subscription(Odometry, "/odom", self._odom_callback, 20)
        self.create_subscription(
            Twist, self.motion_cmd_topic, self._cmd_vel_callback, 10
        )
        self.create_subscription(
            Bool,
            "/localization/motion_inhibited",
            self._motion_inhibited_callback,
            STATE_QOS,
        )

        self.nomotion_client = self.create_client(Empty, "/request_nomotion_update")
        self.global_localization_client = self.create_client(
            Empty, "/reinitialize_global_localization"
        )
        self.navigate_cancel_client = self.create_client(
            CancelGoal, "/navigate_to_pose/_action/cancel_goal"
        )
        self.poses_cancel_client = self.create_client(
            CancelGoal, "/navigate_through_poses/_action/cancel_goal"
        )
        self.spin_client = ActionClient(self, Spin, "/spin")
        # Upstream of velocity smoothing, collision monitoring and semantic safety.
        self.recovery_cmd_pub = self.create_publisher(Twist, "/cmd_vel_nav", 1)
        self.recovery_lease_pub = self.create_publisher(
            String, "/localization/recovery_motion_lease", 1
        )
        self.create_subscription(String, "/localization/recovery_motion_guard",
                                 self._recovery_guard_callback, 1)
        self.create_subscription(String, "/stm32_data", self._recovery_feedback_callback, 50)
        self.lateral_timer = self.create_timer(0.05, self._tick_lateral_motion)
        self.timer = self.create_timer(0.2, self._tick)
        self._publish_state()
        self.get_logger().info(
            "定位管理器已啟動；初始位置為 "
            f"({self.initial_x:.2f}, {self.initial_y:.2f}, {self.initial_yaw:.2f})；"
            f"來源={self.initial_pose_source}；"
            f"實際底盤命令監看 {self.motion_cmd_topic}"
        )
        if self.startup_cache_error:
            self.get_logger().warning(
                f"無法建立定位開機標記，重啟保護可能無法生效：{self.startup_cache_error}"
            )
        if self.same_boot_pose_untrusted:
            self.get_logger().warning(
                "偵測到同次開機內的 Nav2 重啟，但沒有近期可信位置；"
                "不套用固定 (0,0,0)，等待人工 2D Pose Estimate"
            )

    def _declare_parameters(self):
        defaults = {
            "initial_pose.x": 0.0,
            "initial_pose.y": 0.0,
            "initial_pose.yaw": 0.0,
            "initial_pose.xy_std": 0.25,
            "initial_pose.yaw_std": 0.26,
            "auto_initialize": True,
            "reuse_same_boot_pose": True,
            "pose_cache_path": "~/.local/state/smart-carrier/localization_pose.json",
            "pose_cache_max_age_sec": 3600.0,
            "pose_cache_write_period_sec": 2.0,
            "motion_cmd_topic": "/chassis_cmd_vel",
            "raw_scan_topic": "/scan",
            "filtered_scan_topic": "/scan_filtered",
            "scan_timeout_sec": 2.0,
            "scan_timeout_grace_sec": 1.0,
            "odom_timeout_sec": 1.0,
            "amcl_pose_timeout_sec": 8.0,
            "amcl_stationary_grace_sec": 2.0,
            "amcl_nomotion_refresh_sec": 3.0,
            "verification_nomotion_refresh_sec": 1.0,
            "recovery_pipeline_wait_timeout_sec": 30.0,
            "verify_timeout_sec": 15.0,
            "verify_max_wait_sec": 40.0,
            "stable_samples": 6,
            "transient_recovery_samples": 2,
            "transient_recovery_max_sec": 8.0,
            "healthy_xy_std": 0.20,
            "healthy_yaw_std": 0.17,
            "critical_xy_std": 0.50,
            "critical_yaw_std": 0.45,
            "seed_xy_tolerance": 0.60,
            "seed_yaw_tolerance": 0.52,
            "suspect_hold_sec": 5.0,
            "suspect_max_sec": 12.0,
            "suspect_promising_max_sec": 30.0,
            "cancel_grace_sec": 2.0,
            "local_recovery_wait_sec": 12.0,
            "enable_small_sweep": False,
            "pre_global_wait_sec": 4.0,
            "lateral_recovery_enabled": True,
            "lateral_recovery_distance": 0.30,
            "lateral_recovery_speed": 0.25,
            "lateral_footprint_half_x": 0.22,
            "lateral_footprint_half_y": 0.16,
            "self_mask_config_file": "",  # Same installed YAML as my_lidar.launch.py
            "local_recovery_max_wait_sec": 30.0,
            "global_recovery_wait_sec": 12.0,
            "global_recovery_max_wait_sec": 30.0,
            "global_convergence_max_wait_sec": 40.0,
            "small_spin_angle": math.radians(20.0),
            "full_spin_angle": 2.0 * math.pi,
            "spin_time_allowance_sec": 35,
            "spin_heading_tolerance": math.radians(5.0),
            "spin_heading_correction_gain": 0.65,
            "spin_heading_max_correction": math.radians(20.0),
            "spin_heading_min_improvement": math.radians(1.0),
            "spin_settle_sec": 1.0,
            "spin_settle_timeout_sec": 5.0,
            "spin_heading_max_corrections": 2,
            "pose_jump_distance": 0.40,
            "pose_jump_angle": math.radians(25.0),
            "external_move_distance": 0.25,
            "external_move_angle": math.radians(20.0),
            "automatic_motion_recovery": True,
            "map_match_min_score": 0.30,
            "map_match_recovery_score": 0.35,
            "map_match_critical_score": 0.20,
            "map_match_check_period_sec": 1.0,
            "map_match_max_beams": 60,
            "map_match_min_beams": 10,
            "map_match_neighborhood_cells": 2,
            "map_match_window_size": 5,
            "map_match_window_min_samples": 3,
            "map_match_ignore_below_range": 0.60,
            "map_match_min_sectors": 4,
            "map_match_sector_total": 12,
            "map_match_stale_grace_sec": 5.0,
            "verification_pose_distance": 0.20,
            "verification_pose_angle": math.radians(10.0),
            "diagnostic_log_period_sec": 5.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

    def _configure_startup_pose(self):
        """Reuse a trusted pose only when this is a same-boot Nav2 restart."""
        self.startup_cache_error = None
        if not self.reuse_same_boot_pose or self.boot_id is None:
            return

        cache = load_cache(self.pose_cache_path)
        status, pose = select_same_boot_pose(
            cache,
            self.boot_id,
            max_age_sec=self.pose_cache_max_age,
        )
        if status == "same_boot_cached":
            self.initial_x, self.initial_y, self.initial_yaw = pose
            self.initial_pose_source = "same_boot_trusted_cache"
            return
        if status == "same_boot_untrusted":
            self.same_boot_pose_untrusted = True
            self.auto_initialize = False
            self.initial_pose_source = "same_boot_manual_required"
            return

        # The first localization-manager process of a Linux boot may use the
        # known physical power-on pose.  Writing the boot marker immediately
        # prevents a second process in the same boot from making that assumption
        # before any pose has become trustworthy.
        try:
            write_cache(self.pose_cache_path, self.boot_id)
        except OSError as exc:
            self.startup_cache_error = str(exc)

    def _persist_trusted_pose(self, *, force=False):
        if (
            not self.reuse_same_boot_pose
            or self.boot_id is None
            or self.latest_pose is None
        ):
            return
        now = time.monotonic()
        if not force and now - self.last_pose_cache_write_at < self.pose_cache_write_period:
            return
        try:
            write_cache(self.pose_cache_path, self.boot_id, self.latest_pose)
        except (OSError, ValueError) as exc:
            if now - self.last_pose_cache_write_at >= self.diagnostic_log_period:
                self.get_logger().warning(f"無法保存同次開機可信位置：{exc}")
            self.last_pose_cache_write_at = now
            return
        self.last_pose_cache_write_at = now

    def _set_state(self, state, reason):
        if state != self.state:
            # Even LOCALIZED is a phase boundary. It must break the clock's
            # previous ready edge before a later identical SUSPECT phase.
            self._reset_observation_phase()
        changed = state != self.state or reason != self.state_reason
        self.state = state
        self.state_since = time.monotonic()
        self.state_reason = reason
        should_be_ready = state == "LOCALIZED"
        if should_be_ready != self.ready:
            self.ready = should_be_ready
            ready_message = Bool()
            ready_message.data = self.ready
            self.ready_publisher.publish(ready_message)
        if changed:
            message = f"定位狀態：{state}（{reason}）"
            if self.ready:
                self.get_logger().info(message)
            else:
                self.get_logger().warning(message)
        self._publish_state()

    def _publish_state(self):
        now = time.monotonic()
        self.last_state_publish_at = now
        message = String()
        freshness = self._freshness_snapshot(now)
        payload = {
            'sample_ros_stamp_ns': self.get_clock().now().nanoseconds,
            "state": self.state,
            "ready": self.ready,
            "reason": self.state_reason,
            "scan_age_sec": (
                None if not self.latest_scan_at else now - self.latest_scan_at
            ),
            "raw_scan_age_sec": freshness["raw_scan_receive_age_sec"],
            "scan_stamp_age_sec": freshness["scan_stamp_age_sec"],
            "map_match_age_sec": (
                None if not self.latest_map_score_at else now - self.latest_map_score_at
            ),
            "map_match_samples": len(self.map_score_window),
            "map_match_near_fraction": round(self.latest_map_near_fraction, 4),
            "map_match_sector_count": self.latest_map_sector_count,
            "map_match_tf_failures": self.map_match_tf_failures,
            "recovery_step": self.recovery_step,
            "lateral_attempted": self.lateral_attempted,
            "lateral_phase": self.lateral_phase,
            "lateral_direction": self.lateral_direction,
            "lateral_abort_reason": self.lateral_abort_reason,
            "amcl_pose_age_sec": freshness["amcl_pose_age_sec"],
            "odom_age_sec": freshness["odom_age_sec"],
            "cmd_vel_age_sec": freshness["cmd_vel_age_sec"],
            "stable_samples": self.stable_samples,
            "stable_samples_required": self.stable_samples_required,
            "verification_elapsed_sec": round(self.observation_clock.elapsed, 3),
            "verification_blockers": self._verification_data_blockers(now),
            "pipeline_outage_sec": round(self.pipeline_clock.blocked_for(now), 3),
            "tick_max_gap_sec": round(self.tick_max_gap, 3),
            "tick_max_duration_sec": round(self.tick_max_duration, 3),
            "pipeline_receipts": self.receipt_metrics.snapshot(now),
            "scan_pair_timing": self.scan_pair_metrics.snapshot(now),
            "wall_durations": self.duration_metrics.snapshot(now),
            "lateral_path_reports": self.lateral_path_reports,
            "lateral_self_mask": self.lateral_self_mask,
        }
        if self.suspect_since is not None:
            payload["suspect_age_sec"] = max(0.0, now - self.suspect_since)
        if self.latest_quality is not None:
            payload.update(
                {
                    "xy_std": round(self.latest_quality.xy_std, 4),
                    "yaw_std": round(self.latest_quality.yaw_std, 4),
                }
            )
        if self.latest_map_score is not None:
            payload["map_match_score"] = round(self.latest_map_score, 4)
        if self.latest_map_score_raw is not None:
            payload["map_match_raw_score"] = round(self.latest_map_score_raw, 4)
        if self.last_map_match_error:
            payload["map_match_error"] = self.last_map_match_error
        if self.state != "LOCALIZED" and self.recovery_diagnostics:
            payload["recovery_diagnostics"] = self.recovery_diagnostics
        message.data = json.dumps(payload, ensure_ascii=False)
        with self.duration_metrics.measure('state_publish'):
            self.state_publisher.publish(message)

    @staticmethod
    def _age_since(timestamp, now):
        if timestamp <= 0.0:
            return None
        return max(0.0, now - timestamp)

    def _freshness_snapshot(self, now=None):
        now = time.monotonic() if now is None else now

        def rounded_age(timestamp):
            age = self._age_since(timestamp, now)
            return None if age is None else round(age, 3)

        scan_stamp_age = None
        if self.latest_scan_stamp_ns > 0:
            scan_stamp_age = max(
                0.0,
                (
                    self.get_clock().now().nanoseconds
                    - self.latest_scan_stamp_ns
                )
                / 1_000_000_000.0,
            )

        return {
            "raw_scan_receive_age_sec": rounded_age(self.latest_raw_scan_at),
            "scan_receive_age_sec": rounded_age(self.latest_scan_at),
            "scan_stamp_age_sec": (
                None if scan_stamp_age is None else round(scan_stamp_age, 3)
            ),
            "amcl_pose_age_sec": rounded_age(self.last_amcl_at),
            "odom_age_sec": rounded_age(self.latest_odom_at),
            "cmd_vel_age_sec": rounded_age(self.latest_cmd_at),
            "map_match_age_sec": rounded_age(self.latest_map_score_at),
            "map_match_score": (
                None
                if self.latest_map_score is None
                else round(self.latest_map_score, 3)
            ),
            "map_match_raw_score": (
                None
                if self.latest_map_score_raw is None
                else round(self.latest_map_score_raw, 3)
            ),
            "xy_std": (
                None
                if self.latest_quality is None
                else round(self.latest_quality.xy_std, 3)
            ),
            "yaw_std_deg": (
                None
                if self.latest_quality is None
                else round(math.degrees(self.latest_quality.yaw_std), 2)
            ),
            "map_match_tf_failures": self.map_match_tf_failures,
            "commanded_motion": self.commanded_motion,
            "odom_motion": self.odom_motion,
        }

    def _diagnostic_snapshot_text(self, snapshot):
        def age(name):
            value = snapshot.get(name)
            return "unknown" if value is None else f"{value:.3f}s"

        score = snapshot.get("map_match_score")
        score_text = "unknown" if score is None else f"{score:.3f}"
        xy_std = snapshot.get("xy_std")
        xy_text = "unknown" if xy_std is None else f"{xy_std:.3f}m"
        yaw_std = snapshot.get("yaw_std_deg")
        yaw_text = "unknown" if yaw_std is None else f"{yaw_std:.2f}deg"
        return (
            f"raw_scan_rx={age('raw_scan_receive_age_sec')}, "
            f"filtered_scan_rx={age('scan_receive_age_sec')}, "
            f"scan_stamp={age('scan_stamp_age_sec')}, "
            f"amcl={age('amcl_pose_age_sec')}, "
            f"odom={age('odom_age_sec')}, cmd={age('cmd_vel_age_sec')}, "
            f"score={score_text}, xy_std={xy_text}, yaw_std={yaw_text}, "
            f"tf_failures={snapshot['map_match_tf_failures']}, "
            f"commanded_motion={snapshot['commanded_motion']}, "
            f"odom_motion={snapshot['odom_motion']}"
        )

    def _scan_callback(self, message):
        self.latest_scan_at = time.monotonic()
        self._record_receipt("scan_filtered", message, self.latest_scan_at)
        self.latest_scan = message
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )
        self.latest_scan_stamp_ns = stamp_ns
        if stamp_ns > 0:
            self.latest_scan_stamp_age = (
                self.get_clock().now().nanoseconds - stamp_ns
            ) / 1_000_000_000.0
        else:
            self.latest_scan_stamp_age = None

    def _raw_scan_callback(self, _message):
        self.latest_raw_scan = _message
        self.latest_raw_scan_at = time.monotonic()
        self._record_receipt("scan", _message, self.latest_raw_scan_at)

    def _record_receipt(self, name, message, now):
        """Observe source latency without changing the existing freshness gate."""
        stamp = message.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        source_age = None if stamp_ns <= 0 else (
            self.get_clock().now().nanoseconds - stamp_ns
        ) / 1e9
        self.receipt_metrics.observe(name, now, source_age)
        if name in ('scan', 'scan_filtered'):
            self.scan_pair_metrics.observe(name, stamp_ns, now)

    def _map_callback(self, message):
        self.map_message = message

    def _sensor_tf_is_ready(self):
        if self.latest_scan is None:
            return False
        try:
            self.tf_buffer.lookup_transform(
                "base_footprint",
                self.latest_scan.header.frame_id,
                Time(),
                timeout=RclpyDuration(seconds=0.05),
            )
            self.tf_buffer.lookup_transform(
                "odom",
                "base_footprint",
                Time(),
                timeout=RclpyDuration(seconds=0.05),
            )
        except TransformException as exc:
            self.last_map_match_error = f"感測器 TF 尚未就緒：{exc}"
            return False
        return True

    def _reset_map_match_history(self):
        self.map_score_window.clear()
        self.latest_map_score = None
        self.latest_map_score_raw = None
        self.latest_map_score_at = 0.0
        self.latest_map_sector_count = 0
        self.verification_pose_anchor = self.latest_pose
        self.last_stable_map_score_at = 0.0

    def _verification_pose_is_consistent(self):
        if self.latest_pose is None:
            return False
        if self.verification_pose_anchor is None:
            self.verification_pose_anchor = self.latest_pose
            return False
        distance, angle = pose_jump(self.verification_pose_anchor, self.latest_pose)
        if (
            distance > self.verification_pose_distance
            or angle > self.verification_pose_angle
        ):
            self.verification_pose_anchor = self.latest_pose
            return False
        return True

    def _cmd_vel_callback(self, message):
        now = time.monotonic()
        self.latest_cmd_at = now
        self.commanded_motion = (
            math.hypot(float(message.linear.x), float(message.linear.y)) > 0.02
            or abs(float(message.angular.z)) > 0.03
        )
        if self.commanded_motion:
            self.last_motion_command_at = now
            self.stationary_odom_anchor = None
            self.stationary_amcl_anchor = None

    def _motion_inhibited_callback(self, message):
        self.motion_inhibited = bool(message.data)

    def _odom_callback(self, message):
        now = time.monotonic()
        self.latest_odom_stamp_ns = (
            message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        )
        self._record_receipt("odom", message, now)
        self.latest_odom_at = now
        pose = message.pose.pose
        current = (
            float(pose.position.x),
            float(pose.position.y),
            quaternion_to_yaw(pose.orientation),
        )
        twist = message.twist.twist
        self.odom_motion = (
            math.hypot(float(twist.linear.x), float(twist.linear.y)) > 0.01
            or abs(float(twist.angular.z)) > 0.02
        )
        if self.odom_motion:
            self.last_odom_motion_at = now
        self.last_odom_pose = current
        if self.commanded_motion or self.state != "LOCALIZED":
            self.stationary_odom_anchor = None
            return
        if self.stationary_odom_anchor is None:
            self.stationary_odom_anchor = current
            return
        distance, angle = pose_jump(self.stationary_odom_anchor, current)
        if distance >= self.external_move_distance or angle >= self.external_move_angle:
            self.stationary_odom_anchor = current
            self._begin_recovery(
                f"無速度命令但里程計移動 {distance:.2f} m / {math.degrees(angle):.1f} deg"
            )

    def _amcl_pose_callback(self, message):
        now = time.monotonic()
        self._record_receipt("amcl_pose", message, now)
        pose = message.pose.pose
        current = (
            float(pose.position.x),
            float(pose.position.y),
            quaternion_to_yaw(pose.orientation),
        )
        quality = pose_quality(
            message.pose.covariance,
            self.healthy_xy_std,
            self.healthy_yaw_std,
            self.critical_xy_std,
            self.critical_yaw_std,
        )
        self.latest_pose = current
        self.latest_quality = quality
        self.last_amcl_at = now

        if (
            self.state == "LOCALIZED"
            and quality.healthy
            and self._map_match_has_recovered(now)
        ):
            self._persist_trusted_pose()

        if self.state == "LOCALIZED" and not self.commanded_motion:
            if self.stationary_amcl_anchor is None:
                self.stationary_amcl_anchor = current
            distance, angle = pose_jump(self.stationary_amcl_anchor, current)
            if distance >= self.pose_jump_distance or angle >= self.pose_jump_angle:
                self.stationary_amcl_anchor = current
                self._begin_recovery(
                    f"無速度命令但 AMCL 累積位移 {distance:.2f} m / "
                    f"{math.degrees(angle):.1f} deg"
                )
                return

        if self.state in {
            "VERIFYING",
            "RECOVERING_LOCAL",
            "RECOVERING_GLOBAL",
        }:
            reference_ok = True
            if self.verify_reference_required:
                reference_ok = pose_is_near(
                    *current,
                    self.initial_x,
                    self.initial_y,
                    self.initial_yaw,
                    self.seed_xy_tolerance,
                    self.seed_yaw_tolerance,
                )
            map_match_ok = self._map_match_has_recovered(now)
            recovery_motion_complete = not (
                self.spin_in_progress
                or self.spin_sequence
                or self.spin_waiting_for_stop
                or self.spin_restoring_heading
                or self.recovery_step
                in {"small_spin", "full_spin", "restore_heading", "spin_settle",
                    "lateral_move", "lateral_stop"}
            )
            qualified = (
                quality.healthy
                and reference_ok
                and map_match_ok
                and self._verification_pose_is_consistent()
                and self._scan_is_fresh(now)
                and not self._verification_data_blockers(now)
                and recovery_motion_complete
            )
            self.stable_samples, self.last_stable_map_score_at = (
                update_stability_samples(
                    qualified,
                    self.latest_map_score_at,
                    self.last_stable_map_score_at,
                    self.stable_samples,
                )
            )
            if self.transient_recovery_eligible:
                if self.transient_clock.elapsed > self.transient_recovery_max:
                    self.transient_recovery_eligible = False
                elif self._transient_recovery_is_qualified(now) and (
                    self.stable_samples >= self.transient_recovery_samples_required
                ):
                    self._mark_localized(
                        "短暫感測逾時後資料與定位品質已連續恢復"
                    )
                    return
            if qualified and self.stable_samples >= self.stable_samples_required:
                self._mark_localized()

    def _external_initial_pose_callback(self, _message):
        if self.lateral_phase is not None:
            self._stop_lateral("收到人工初始位置；先停止，待停妥後重新提供位置")
            return
        if self.manual_initial_pose_pending:
            self.manual_initial_pose_pending = False
            return
        if self.state == "MANUAL_REQUIRED":
            self.stable_samples = 0
            self._reset_map_match_history()
            self.verify_reference_required = False
            self._set_state("VERIFYING", "收到人工初始位置，正在驗證")

    def _scan_is_fresh(self, now=None):
        now = time.monotonic() if now is None else now
        return self.latest_scan_at > 0.0 and now - self.latest_scan_at <= self.scan_timeout

    def _raw_scan_is_fresh(self, now=None):
        now = time.monotonic() if now is None else now
        return (
            self.latest_raw_scan_at > 0.0
            and now - self.latest_raw_scan_at <= self.scan_timeout
        )

    def _transient_recovery_is_qualified(self, now):
        return self._raw_scan_is_fresh(now) and transient_sensor_recovery_qualified(
            self.latest_quality is not None and self.latest_quality.healthy,
            self._map_match_has_recovered(now),
            self._age_since(self.latest_scan_at, now),
            self.scan_timeout,
            self._age_since(self.latest_odom_at, now),
            self.odom_timeout,
            self._age_since(self.last_amcl_at, now),
            self.amcl_pose_timeout,
            self.commanded_motion,
            self.odom_motion,
        )

    def _verification_data_blockers(self, now):
        """Separate unavailable observations from genuinely unhealthy covariance."""
        blockers = []
        for name, stamp, limit in (
            ("raw_scan", self.latest_raw_scan_at, self.scan_timeout),
            ("scan_filtered", self.latest_scan_at, self.scan_timeout),
            ("odom", self.latest_odom_at, self.odom_timeout),
            ("amcl_pose", self.last_amcl_at, self.amcl_pose_timeout),
            ("map_match", self.latest_map_score_at, 2.0 * self.map_match_period),
        ):
            if stamp <= 0.0 or now - stamp > limit:
                blockers.append(f"{name}_unavailable")
        if self.latest_scan_stamp_ns > 0:
            stamp_age = (
                self.get_clock().now().nanoseconds - self.latest_scan_stamp_ns
            ) / 1_000_000_000.0
            if stamp_age < -0.1 or stamp_age > self.scan_timeout:
                blockers.append("scan_timestamp_invalid")
        if self.last_map_match_error:
            blockers.append("map_match_update_failed")
        return blockers

    def _observe_wait(self, now):
        """Pause verification deadlines during outages, with a bounded safe wait."""
        phase = (self.state, self.recovery_step)
        if phase != self.observation_phase:
            self._reset_observation_phase()
            self.observation_phase = phase
        blockers = self._verification_data_blockers(now)
        data_ready = not blockers
        waiting_for_user = self.motion_inhibited and self.state.startswith("RECOVERING")
        self.observation_clock.update(now, data_ready and not waiting_for_user)
        # Waiting for a user is not a sensor outage; keep their clocks separate.
        self.pipeline_clock.update(now, data_ready)
        if not data_ready:
            self.convergence_samples.clear()
        elif (
            self.latest_quality is not None
            and self.latest_map_score_at > self.last_convergence_map_at
        ):
            self.last_convergence_map_at = self.latest_map_score_at
            self.convergence_samples.append(
                (self.observation_clock.elapsed, self.latest_quality.xy_std)
            )
        if self.transient_recovery_eligible:
            self.transient_clock.update(now, data_ready and not waiting_for_user)
        if not data_ready:
            self.stable_samples = 0
            self.last_stable_map_score_at = self.latest_map_score_at
            self._request_nomotion_if_due(now)
            if self.pipeline_clock.blocked_for(now) >= self.pipeline_wait_timeout:
                self._require_manual(
                    "定位資料管線持續未恢復，維持安全停車：" + ", ".join(blockers)
                )
            elif now - self.last_diagnostic_log_at >= self.diagnostic_log_period:
                self.get_logger().warning(
                    "等待新鮮定位資料，暫停驗證計時且不啟動自旋："
                    + ", ".join(blockers)
                )
                self.last_diagnostic_log_at = now
        return data_ready

    def _reset_observation_phase(self):
        """Invalidate every phase edge, including transitions outside recovery."""
        self.observation_phase = None
        self.observation_clock = EvidenceWaitClock()
        self.pipeline_clock = EvidenceWaitClock()
        self.convergence_samples = deque(maxlen=10)
        self.last_convergence_map_at = 0.0
        self.convergence_grace_announced = False

    def _convergence_grace(self, now, normal_wait, maximum_wait):
        """Wait for a stationary improving solution without releasing motion."""
        quality = self.latest_quality
        promising = bool(
            quality is not None
            and not quality.critical
            and not self.commanded_motion
            and not self.odom_motion
            and not self.motion_inhibited
            and not self._verification_data_blockers(now)
            and self._map_match_has_recovered(now)
            and self.latest_pose is not None
            and self.verification_pose_anchor is not None
            and pose_is_near(
                *self.latest_pose, *self.verification_pose_anchor,
                self.verification_pose_distance, self.verification_pose_angle,
            )
            and (
                not self.verify_reference_required
                or pose_is_near(
                    *self.latest_pose, self.initial_x, self.initial_y,
                    self.initial_yaw, self.seed_xy_tolerance, self.seed_yaw_tolerance,
                )
            )
            and (
                (quality.healthy and self.stable_samples > 0)
                or covariance_is_converging(self.convergence_samples)
            )
        )
        extend = convergence_grace_allowed(
            self.observation_clock.elapsed, normal_wait, maximum_wait, promising
        )
        if extend and not self.convergence_grace_announced:
            self.convergence_grace_announced = True
            self.get_logger().warning(
                "定位持續收斂，維持停車並延長健康樣本確認："
                f"最多 {maximum_wait:.0f} 秒，"
                f"samples={self.stable_samples}/{self.stable_samples_required}"
            )
        return extend

    def _recovery_failure_reason(self):
        quality = self.latest_quality
        details = [f"穩定樣本 {self.stable_samples}/{self.stable_samples_required}"]
        if quality is None or not quality.healthy:
            details.append("AMCL covariance 未達健康門檻")
        if not self._map_match_has_recovered(time.monotonic()):
            details.append("雷達地圖吻合度未恢復")
        return "；".join(details)

    def _tick(self):
        started = time.monotonic()
        if self.tick_last_at is not None:
            self.tick_max_gap = max(self.tick_max_gap, started - self.tick_last_at)
        self.tick_last_at = started
        try:
            self._tick_state_machine()
        finally:
            if (started - self.last_pipeline_diagnostic_at >= 5.0
                    and self.latest_scan is not None):
                stamp_age = (
                    self.get_clock().now().nanoseconds - self.latest_scan_stamp_ns
                ) / 1e9 if self.latest_scan_stamp_ns > 0 else float('inf')
                filtered_age = started - self.latest_scan_at
                if filtered_age > 0.30 or stamp_age > 0.30:
                    self.get_logger().warning(
                        "雷達管線延遲診斷（不放寬定位安全門檻）："
                        f"raw_receive={started - self.latest_raw_scan_at:.3f}s, "
                        f"filtered_receive={filtered_age:.3f}s, "
                        f"filtered_source={stamp_age:.3f}s, "
                        f"tick_gap={self.tick_max_gap:.3f}s, "
                        f"tf_failures={self.map_match_tf_failures}"
                    )
                    self.last_pipeline_diagnostic_at = started
            self.tick_max_duration = max(
                self.tick_max_duration, time.monotonic() - started
            )
            # Publish a lightweight heartbeat even when AMCL is stationary.
            if time.monotonic() - self.last_state_publish_at >= 1.0:
                self._publish_state()
            if started - self.last_tick_metrics_at >= 5.0:
                self.tick_max_gap = self.tick_max_duration = 0.0
                self.last_tick_metrics_at = started

    def _tick_state_machine(self):
        now = time.monotonic()
        if self.lateral_phase is not None:
            # Local raw-sensor safety is checked by the independent 20 Hz loop.
            # AMCL/map evidence must not abort or accept localization mid-sidestep.
            return
        observing = self.state in {
            "VERIFYING", "SUSPECT", "RECOVERING_LOCAL", "RECOVERING_GLOBAL"
        } and not (self.spin_in_progress or self.spin_waiting_for_stop)
        if not self._scan_is_fresh(now):
            if observing:
                self._observe_wait(now)
            if self.state == "LOCALIZED":
                raw_scan_age = self._age_since(self.latest_raw_scan_at, now)
                scan_age = self._age_since(self.latest_scan_at, now)
                pipeline_status = scan_pipeline_status(
                    raw_scan_age,
                    scan_age,
                    self.scan_timeout,
                    self.scan_timeout_grace,
                )
                if pipeline_status != "healthy":
                    raw_age_text = (
                        "unknown" if raw_scan_age is None else f"{raw_scan_age:.2f}s"
                    )
                    filtered_age_text = (
                        "unknown" if scan_age is None else f"{scan_age:.2f}s"
                    )
                    recovery_limit = self.scan_timeout + self.scan_timeout_grace
                    if pipeline_status == "filtered_timeout":
                        reason = (
                            "scan_filtered 管線逾時，但原始雷達正常"
                            f"（raw={raw_age_text}, filtered={filtered_age_text}, "
                            f"limit={recovery_limit:.2f}s）"
                        )
                    else:
                        reason = (
                            "原始雷達與 scan_filtered 均逾時"
                            f"（raw={raw_age_text}, filtered={filtered_age_text}, "
                            f"limit={recovery_limit:.2f}s）"
                        )
                    self._begin_recovery(
                        reason,
                        transient_sensor_timeout=True,
                        sensor_pipeline_timeout=True,
                    )
            elif self.state == "UNINITIALIZED":
                self._set_state(
                    "WAITING_FOR_SENSORS", f"等待 {self.filtered_scan_topic}"
                )
            self._publish_state()
            return

        if now - self.last_map_match_check_at >= self.map_match_period:
            self.last_map_match_check_at = now
            self._update_map_match_score(now)

        if self.spin_waiting_for_stop:
            self._tick_spin_settle(now)
            self._publish_state()
            return

        if observing and not self._observe_wait(now):
            return

        if self.state in {"UNINITIALIZED", "WAITING_FOR_SENSORS"}:
            if not self.auto_initialize:
                reason = "自動初始位置已停用"
                if self.same_boot_pose_untrusted:
                    reason = "同次開機重啟但沒有近期可信位置"
                self._require_manual(reason)
                return
            if self.map_message is None:
                self._set_state("WAITING_FOR_SENSORS", "等待靜態地圖")
                return
            if self.initial_pose_publisher.get_subscription_count() < 1:
                self._set_state("WAITING_FOR_SENSORS", "等待 AMCL /initialpose 訂閱者")
                return
            if not self._sensor_tf_is_ready():
                self._set_state("WAITING_FOR_SENSORS", "等待雷達與里程計 TF")
                return
            self._publish_initial_pose()
            return

        if self.state == "VERIFYING":
            self._request_nomotion_if_due(now)
            if (
                self.last_amcl_at < self.initial_pose_last_published
                and now - self.initial_pose_last_published >= 2.0
                and self.initial_pose_publisher.get_subscription_count() >= 1
            ):
                self._publish_initial_pose(retry=True)
            if self.observation_clock.elapsed >= self.verify_timeout:
                if self._convergence_grace(now, self.verify_timeout, self.verify_max_wait):
                    return
                self._begin_recovery("初始位置驗證逾時")
            return

        if self.state == "LOCALIZED":
            if self.latest_quality is None:
                self._begin_recovery("AMCL 位姿資料逾時")
                return
            amcl_pose_age = now - self.last_amcl_at
            odom_is_fresh = (
                self.latest_odom_at > 0.0
                and now - self.latest_odom_at <= self.odom_timeout
            )
            odom_is_moving = not odom_is_fresh or self.odom_motion
            if (
                not odom_is_moving
                and amcl_pose_age >= self.amcl_nomotion_refresh
                and now - self.last_nomotion_request_at >= self.amcl_nomotion_refresh
            ):
                self._request_nomotion_update()
            if amcl_timeout_requires_recovery(
                amcl_pose_age,
                self.amcl_pose_timeout,
                odom_is_moving,
                self.amcl_stationary_grace,
            ):
                self._begin_recovery(
                    "AMCL 位姿資料逾時"
                    f"（age={amcl_pose_age:.2f}s > "
                    f"{self.amcl_pose_timeout:.2f}s）",
                    transient_sensor_timeout=True,
                )
                return
            match_status = self._map_match_status(now)
            if match_status == "unknown":
                if self.map_match_unknown_since is None:
                    self.map_match_unknown_since = now
            else:
                self.map_match_unknown_since = None
            map_match_stale = (
                self.map_match_unknown_since is not None
                and now - self.map_match_unknown_since >= self.map_match_stale_grace
            )
            map_match_bad = match_status in {"critical", "degraded"} or map_match_stale
            suspicion_required = localization_suspicion_required(
                "unknown" if map_match_stale else match_status,
                self.latest_quality.healthy,
                self.latest_quality.critical,
                self.commanded_motion,
                self.odom_motion,
            )
            if self.latest_quality.critical or (map_match_bad and suspicion_required):
                if self.suspect_since is None:
                    self.suspect_since = now
                    self.stable_samples = 0
                    self.last_stable_map_score_at = self.latest_map_score_at
                    self.verification_pose_anchor = self.latest_pose
                    if match_status == "critical":
                        reason = f"雷達地圖吻合度過低 ({self.latest_map_score:.2f})"
                    elif match_status == "degraded":
                        reason = (
                            f"雷達地圖吻合度未達可信門檻 "
                            f"({self.latest_map_score:.2f} < {self.map_match_min_score:.2f})"
                        )
                    elif map_match_stale:
                        reason = "雷達地圖吻合度資料持續無法更新"
                    else:
                        reason = "AMCL 不確定度持續偏高"
                    self._set_state("SUSPECT", reason)
            else:
                self.suspect_since = None
                if (
                    map_match_bad
                    and now - self.last_map_only_warning_at
                    >= self.diagnostic_log_period
                ):
                    score_text = (
                        "unknown"
                        if self.latest_map_score is None
                        else f"{self.latest_map_score:.2f}"
                    )
                    self.get_logger().warning(
                        "車輛靜止且 AMCL covariance 健康；"
                        f"單一地圖吻合低分（{score_text}）僅記錄，"
                        "不觸發定位自旋"
                    )
                    self.last_map_only_warning_at = now
            return

        if self.state == "SUSPECT":
            if not self.commanded_motion and not self.odom_motion:
                self._request_nomotion_if_due(now)
            match_status = self._map_match_status(now)
            quality_critical = (
                self.latest_quality is not None and self.latest_quality.critical
            )
            quality_acceptable = (
                self.latest_quality is not None
                and (
                    self.latest_quality.healthy
                    or (
                        not quality_critical
                        and not self.commanded_motion
                        and not self.odom_motion
                    )
                )
            )
            qualified = (
                quality_acceptable
                and self._map_match_has_recovered(now)
                and self._verification_pose_is_consistent()
                and now - self.last_amcl_at <= self.amcl_pose_timeout
                and self._scan_is_fresh(now)
            )
            self.stable_samples, self.last_stable_map_score_at = (
                update_stability_samples(
                    qualified,
                    self.latest_map_score_at,
                    self.last_stable_map_score_at,
                    self.stable_samples,
                )
            )
            if qualified and self.stable_samples >= self.stable_samples_required:
                self._mark_localized()
            elif self.suspect_since is not None:
                suspect_age = self.observation_clock.elapsed
                convergence_promising = (
                    self._map_match_has_recovered(now)
                    and self.latest_quality is not None
                    and not quality_critical
                    and now - self.last_amcl_at <= self.amcl_pose_timeout
                    and self._scan_is_fresh(now)
                )
                if suspect_requires_recovery(
                    qualified,
                    match_status,
                    quality_critical,
                    suspect_age,
                    self.suspect_hold,
                    self.suspect_max,
                    convergence_promising,
                    self.suspect_promising_max,
                ):
                    reason = "定位可信度持續不足"
                    if suspect_age >= self.suspect_max:
                        reason = "定位疑慮超過最大等待時間"
                    elif quality_critical:
                        reason = "AMCL 不確定度超過保守門檻"
                    elif match_status in {"critical", "degraded", "unknown"}:
                        reason = "雷達地圖吻合度持續低於可信門檻"
                    self._begin_recovery(reason)
                elif now - self.last_diagnostic_log_at >= self.diagnostic_log_period:
                    score_text = (
                        "unknown"
                        if self.latest_map_score is None
                        else f"{self.latest_map_score:.2f}"
                    )
                    self.get_logger().warning(
                        "定位仍在 SUSPECT："
                        f"age={suspect_age:.1f}s, score={score_text}, "
                        f"samples={len(self.map_score_window)}, "
                        f"near={self.latest_map_near_fraction:.0%}, "
                        f"sectors={self.latest_map_sector_count}, "
                        f"convergence_grace={convergence_promising}"
                    )
                    self.last_diagnostic_log_at = now
            return

        if self.state == "RECOVERING_LOCAL":
            self._tick_local_recovery(now)
        elif self.state == "RECOVERING_GLOBAL":
            self._tick_global_recovery(now)

    def _update_map_match_score(self, now):
        if self.map_message is None or self.latest_scan is None:
            return
        scan = self.latest_scan
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                scan.header.frame_id,
                Time.from_msg(scan.header.stamp),
                timeout=RclpyDuration(seconds=0.05),
            )
        except TransformException as exc:
            self.map_match_tf_failures += 1
            self.last_map_match_error = f"雷達地圖 TF 失敗：{exc}"
            if now - self.last_diagnostic_log_at >= self.diagnostic_log_period:
                self.get_logger().warning(self.last_map_match_error)
                self.last_diagnostic_log_at = now
            return

        selection = select_scan_samples(
            scan.ranges,
            scan.range_min,
            scan.range_max,
            self.map_match_ignore_below_range,
            self.map_match_max_beams,
            self.map_match_sector_total,
        )
        self.latest_map_near_fraction = selection.near_fraction
        self.latest_map_sector_count = selection.sector_count
        if len(selection.samples) < self.map_match_min_beams:
            self.last_map_match_error = (
                f"可用雷達端點不足 ({len(selection.samples)} < "
                f"{self.map_match_min_beams})"
            )
            return
        if selection.sector_count < self.map_match_min_sectors:
            self.last_map_match_error = (
                f"雷達有效視野不足 ({selection.sector_count} < "
                f"{self.map_match_min_sectors} sectors)"
            )
            return

        laser_yaw = quaternion_to_yaw(transform.transform.rotation)
        cosine = math.cos(laser_yaw)
        sine = math.sin(laser_yaw)
        translation = transform.transform.translation
        endpoints = []
        for index, distance in selection.samples:
            angle = float(scan.angle_min) + index * float(scan.angle_increment)
            laser_x = distance * math.cos(angle)
            laser_y = distance * math.sin(angle)
            endpoints.append(
                (
                    float(translation.x) + cosine * laser_x - sine * laser_y,
                    float(translation.y) + sine * laser_x + cosine * laser_y,
                )
            )

        map_info = self.map_message.info
        origin = map_info.origin
        score = occupancy_match_score(
            endpoints,
            self.map_message.data,
            int(map_info.width),
            int(map_info.height),
            float(map_info.resolution),
            float(origin.position.x),
            float(origin.position.y),
            quaternion_to_yaw(origin.orientation),
            neighborhood_cells=self.map_match_neighborhood,
            minimum_endpoints=self.map_match_min_beams,
        )
        if score is not None and len(endpoints) >= self.map_match_min_beams:
            self.latest_map_score_raw = score
            self.map_score_window.append(score)
            self.latest_map_score = smoothed_map_score(
                self.map_score_window,
                self.map_match_window_min_samples,
            )
            self.latest_map_score_at = now
            self.last_map_match_error = ""

    def _map_match_status(self, now):
        return map_match_status(
            self.latest_map_score,
            now - self.latest_map_score_at,
            self.map_match_min_score,
            self.map_match_critical_score,
            2.0 * self.map_match_period,
        )

    def _map_match_has_recovered(self, now):
        return (
            self.latest_map_score is not None
            and self.latest_map_score >= self.map_match_recovery_score
            and len(self.map_score_window) >= self.map_match_window_min_samples
            and now - self.latest_map_score_at <= 2.0 * self.map_match_period
        )

    def _publish_initial_pose(self, retry=False):
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = self.initial_x
        message.pose.pose.position.y = self.initial_y
        message.pose.pose.orientation.z = math.sin(self.initial_yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(self.initial_yaw / 2.0)
        xy_std = float(self.get_parameter("initial_pose.xy_std").value)
        yaw_std = float(self.get_parameter("initial_pose.yaw_std").value)
        message.pose.covariance[0] = xy_std**2
        message.pose.covariance[7] = xy_std**2
        message.pose.covariance[35] = yaw_std**2
        self.manual_initial_pose_pending = True
        self.stable_samples = 0
        if not retry:
            self._reset_map_match_history()
        else:
            self.last_stable_map_score_at = self.latest_map_score_at
        self.verify_reference_required = True
        if not retry:
            source_text = (
                "同次開機最近可信位置"
                if self.initial_pose_source == "same_boot_trusted_cache"
                else "固定開機位置"
            )
            self._set_state(
                "SEEDING",
                f"發布{source_text} "
                f"({self.initial_x:.2f},{self.initial_y:.2f},{self.initial_yaw:.2f})",
            )
        self.initial_pose_publisher.publish(message)
        self.initial_pose_last_published = time.monotonic()
        if retry:
            self.get_logger().warning("尚未收到 AMCL 位姿，重新發布本次初始位置")
        else:
            self._set_state("VERIFYING", "等待 AMCL 收斂")

    def _mark_localized(self, reason_override=None):
        if (
            self.spin_in_progress
            or self.spin_sequence
            or self.spin_waiting_for_stop
            or self.spin_restoring_heading
        ):
            return
        self.suspect_since = None
        self.map_match_unknown_since = None
        self.recovery_step = None
        self.spin_sequence = []
        self.spin_in_progress = False
        self.spin_waiting_for_stop = False
        self.spin_restoring_heading = False
        self.spin_heading_corrections = 0
        self.spin_previous_heading_error = None
        self.spin_failure_reason = None
        self.spin_pending_failure_reason = None
        self.recovery_start_yaw = None
        self.transient_recovery_eligible = False
        self.transient_recovery_started_at = 0.0
        self.sensor_waiting_for_pipeline = False
        self.stationary_odom_anchor = self.last_odom_pose
        self.stationary_amcl_anchor = self.latest_pose
        quality = self.latest_quality
        reason = reason_override or "AMCL 已穩定"
        if quality is not None:
            reason += (
                f"，xy_std={quality.xy_std:.3f} m，"
                f"yaw_std={math.degrees(quality.yaw_std):.1f} deg"
            )
        self._set_state("LOCALIZED", reason)
        self._persist_trusted_pose(force=True)

    def _cancel_navigation(self):
        request = CancelGoal.Request()
        for client in (self.navigate_cancel_client, self.poses_cancel_client):
            if client.service_is_ready():
                client.call_async(request)

    def _begin_recovery(
        self,
        reason,
        *,
        transient_sensor_timeout=False,
        sensor_pipeline_timeout=False,
    ):
        if self.state in {"RECOVERING_LOCAL", "RECOVERING_GLOBAL", "MANUAL_REQUIRED"}:
            return
        now = time.monotonic()
        self.transient_recovery_eligible = bool(
            transient_sensor_timeout
            and self.latest_quality is not None
            and self.latest_quality.healthy
            and not self.commanded_motion
            and not self.odom_motion
        )
        self.sensor_waiting_for_pipeline = bool(sensor_pipeline_timeout)
        self.transient_recovery_started_at = (
            now if self.transient_recovery_eligible else 0.0
        )
        self.transient_clock = EvidenceWaitClock()
        self.recovery_diagnostics = self._freshness_snapshot(now)
        self.get_logger().warning(
            "定位觸發快照："
            f"reason={reason}; "
            f"{self._diagnostic_snapshot_text(self.recovery_diagnostics)}"
        )
        self._cancel_navigation()
        self.stable_samples = 0
        self._reset_map_match_history()
        self.stationary_odom_anchor = None
        self.stationary_amcl_anchor = None
        self.verify_reference_required = False
        self.recovery_reason = reason
        self.lateral_attempted = False
        self.lateral_abort_reason = None
        self.spin_sequence = []
        self.spin_in_progress = False
        self.spin_waiting_for_stop = False
        self.spin_restoring_heading = False
        self.spin_heading_corrections = 0
        self.spin_previous_heading_error = None
        self.spin_failure_reason = None
        self.spin_pending_failure_reason = None
        self.recovery_start_yaw = None
        self.recovery_step = "cancel_wait"
        self.recovery_step_started = now
        self._set_state("RECOVERING_LOCAL", reason)
        if self.transient_recovery_eligible:
            self.get_logger().warning(
                "逾時前 AMCL 與雷達地圖品質健康；等待新的 scan/odom/AMCL "
                "及地圖樣本一致，符合時將略過定位自旋"
            )

    def _request_nomotion_update(self):
        now = time.monotonic()
        self.last_nomotion_request_at = now
        if self.nomotion_client.service_is_ready():
            self.nomotion_client.call_async(Empty.Request())
            self.get_logger().info("已要求 AMCL 執行靜止更新")
            return True
        self.get_logger().warning("AMCL 靜止更新服務尚未就緒")
        return False

    def _request_nomotion_if_due(self, now):
        interval = (
            self.amcl_nomotion_refresh if self.state == "LOCALIZED"
            else self.verification_nomotion_refresh
        )
        if now - self.last_nomotion_request_at >= interval:
            self._request_nomotion_update()

    def _motion_pipeline_is_ready(self):
        if self.count_publishers(self.motion_cmd_topic) > 0:
            return True
        self._require_manual(
            f"底盤速度管線未就緒（{self.motion_cmd_topic} 無發布者；"
            "請確認 fusion_node/start_ai 已啟動）"
        )
        return False

    def _tick_local_recovery(self, now):
        if self.sensor_waiting_for_pipeline:
            raw_fresh = self._raw_scan_is_fresh(now)
            filtered_fresh = self._scan_is_fresh(now)
            if not raw_fresh or not filtered_fresh:
                self._request_nomotion_if_due(now)
                if now - self.last_diagnostic_log_at >= self.diagnostic_log_period:
                    snapshot = self._freshness_snapshot(now)
                    self.get_logger().warning(
                        "感測管線尚未恢復，維持停車且不執行定位自旋："
                        f"{self._diagnostic_snapshot_text(snapshot)}"
                    )
                    self.last_diagnostic_log_at = now
                return
            self.sensor_waiting_for_pipeline = False
            self.recovery_step = "nomotion_wait"
            self.recovery_step_started = now
            self.stable_samples = 0
            self.last_stable_map_score_at = self.latest_map_score_at
            self._request_nomotion_update()
            self.get_logger().warning(
                "原始雷達與 scan_filtered 已恢復；"
                "先驗證新鮮 scan/odom/AMCL，不立即自旋"
            )
            return

        elapsed = self.observation_clock.elapsed
        if self.recovery_step == "cancel_wait" and elapsed >= self.cancel_grace:
            self._request_nomotion_update()
            self.recovery_step = "nomotion_wait"
            self.recovery_step_started = now
        elif self.recovery_step == "nomotion_wait":
            self._request_nomotion_if_due(now)
            wait = self.local_wait if self.enable_small_sweep else self.pre_global_wait
            if elapsed >= wait:
                if self._convergence_grace(now, wait, self.local_max_wait):
                    return
                if not self.auto_motion_recovery:
                    self._require_manual("自動旋轉恢復已停用")
                else:
                    if self.enable_small_sweep:
                        self._start_small_sweep()
                    else:
                        self._start_global_recovery()
        elif self.recovery_step == "post_small":
            self._request_nomotion_if_due(now)
            if elapsed >= self.local_wait:
                if self._convergence_grace(now, self.local_wait, self.local_max_wait):
                    return
                self._start_global_recovery()
        elif self.recovery_step == "post_failed_small":
            self._request_nomotion_if_due(now)
            if elapsed >= self.local_wait:
                if self._convergence_grace(now, self.local_wait, self.local_max_wait):
                    return
                reason = self.recovery_reason or "小幅定位掃描未安全完成"
                self._require_manual(f"{reason}；回正後定位仍未恢復")
        elif self.recovery_step == "motion_inhibited" and not self.motion_inhibited:
            if self.enable_small_sweep:
                self._start_small_sweep()
            else:
                self._start_global_recovery()

    def _start_small_sweep(self):
        if self.motion_inhibited:
            self.recovery_step = "motion_inhibited"
            self.recovery_step_started = time.monotonic()
            self.get_logger().warning("實體取還操作中，暫不執行自動旋轉")
            return
        if not self._motion_pipeline_is_ready():
            return
        if not self.spin_client.server_is_ready():
            self._require_manual("Spin 行為伺服器未就緒")
            return
        self.transient_recovery_eligible = False
        self.recovery_step = "small_spin"
        self.recovery_start_yaw = (
            None if self.last_odom_pose is None else self.last_odom_pose[2]
        )
        self.spin_restoring_heading = False
        self.spin_heading_corrections = 0
        self.spin_previous_heading_error = None
        self.spin_failure_reason = None
        self.spin_pending_failure_reason = None
        self.spin_sequence = [
            self.small_spin_angle,
            -2.0 * self.small_spin_angle,
            self.small_spin_angle,
        ]
        self.get_logger().warning(
            f"開始碰撞檢查的小幅定位掃描（±{math.degrees(self.small_spin_angle):.0f} deg）"
        )
        self._send_next_spin()

    def _start_global_recovery(self):
        self.transient_recovery_eligible = False
        if not self.global_localization_client.service_is_ready():
            self._require_manual("AMCL 全域重新定位服務未就緒")
            return
        self.global_localization_client.call_async(Empty.Request())
        self.stable_samples = 0
        self._reset_map_match_history()
        self.recovery_step = "global_settle"
        self.recovery_step_started = time.monotonic()
        self.global_grace_announced = False
        self._set_state("RECOVERING_GLOBAL", "全域粒子重新定位")

    def _tick_global_recovery(self, now):
        elapsed = self.observation_clock.elapsed
        if self.recovery_step == "global_settle" and elapsed >= 1.0:
            if self.motion_inhibited:
                self.recovery_step_started = now
                return
            if not self._motion_pipeline_is_ready():
                return
            if not self.spin_client.server_is_ready():
                self._require_manual("Spin 行為伺服器未就緒")
                return
            self.recovery_step = "full_spin"
            self.recovery_start_yaw = (
                None if self.last_odom_pose is None else self.last_odom_pose[2]
            )
            self.spin_restoring_heading = False
            self.spin_heading_corrections = 0
            self.spin_previous_heading_error = None
            self.spin_failure_reason = None
            self.spin_pending_failure_reason = None
            self.spin_sequence = [self.full_spin_angle]
            self.get_logger().warning("開始碰撞檢查的 360 度低速定位自旋")
            self._send_next_spin()
        elif self.recovery_step == "post_global":
            self._request_nomotion_if_due(now)
            if elapsed >= self.global_wait:
                map_match_recovered = self._map_match_has_recovered(now)
                if should_extend_global_recovery(
                    elapsed,
                    self.global_wait,
                    self.global_max_wait,
                    map_match_recovered,
                ):
                    if not self.global_grace_announced:
                        self.global_grace_announced = True
                        self.get_logger().warning(
                            "雷達地圖已吻合但 AMCL covariance 尚未收斂；"
                            f"延長觀察至最多 {self.global_max_wait:.0f} 秒"
                        )
                    return
                if self._convergence_grace(
                    now, self.global_max_wait, self.global_convergence_max_wait
                ):
                    return
                if not self._try_lateral_recovery(now):
                    self._require_manual(
                        "全域重新定位後仍未通過可信度檢查；"
                        + self._recovery_failure_reason()
                    )

    def _recovery_feedback_callback(self, message):
        self.recovery_feedback.add_line(message.data, time.monotonic())

    def _recovery_guard_callback(self, message):
        try:
            payload = json.loads(message.data)
            if isinstance(payload, dict):
                self.lateral_guard = payload
                self.lateral_guard_at = time.monotonic()
        except (ValueError, TypeError):
            pass

    def _publish_lateral_lease(self, active=True, permit=False):
        message = String()
        message.data = json.dumps({
            "token": self.lateral_token, "active": active, "permit": permit,
        })
        self.recovery_lease_pub.publish(message)

    def _lateral_sensors_ready(self, now):
        if self.motion_inhibited or self.last_odom_pose is None:
            return False
        if self.recovery_feedback.last_complete_at is None:
            return False
        if self.recovery_feedback.is_stale(now, 0.30):
            return False
        for scan, at in ((self.latest_raw_scan, self.latest_raw_scan_at),
                         (self.latest_scan, self.latest_scan_at)):
            if scan is None or not 0 <= now - at <= 0.30:
                return False
            stamp = scan.header.stamp.sec * 1_000_000_000 + scan.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds - stamp) / 1e9
            if stamp <= 0 or not -0.1 <= age <= 0.30:
                return False
        age = (self.get_clock().now().nanoseconds - self.latest_odom_stamp_ns) / 1e9
        return (self.latest_odom_stamp_ns > 0 and -0.1 <= age <= 0.25
                and 0 <= now - self.latest_odom_at <= 0.25)

    def _lateral_path_clear(self, direction, distance):
        reports = {}
        self.lateral_path_reports[str(direction)] = reports
        try:
            # Local odometry TF only: global localization is deliberately untrusted.
            transform = self.tf_buffer.lookup_transform("odom", "base_footprint", Time())
            stamp = transform.header.stamp
            age = (self.get_clock().now().nanoseconds
                   - stamp.sec * 1_000_000_000 - stamp.nanosec) / 1e9
            if not -0.1 <= age <= 0.30:
                reports['reason'] = 'odom_tf_stale'
                return False
            clear = True
            for name, scan in (('scan', self.latest_raw_scan), ('scan_filtered', self.latest_scan)):
                transform = self.tf_buffer.lookup_transform(
                    "base_footprint", scan.header.frame_id, Time()
                )
                t = transform.transform
                # The current inverted mounting is yaw=pi, not a tilted scan plane.
                # Do not project a changed roll/pitch installation as a flat scan.
                if abs(t.rotation.x) > 1e-4 or abs(t.rotation.y) > 1e-4:
                    reports[name] = {'clear': False, 'reason': 'unsupported_laser_tilt'}
                    return False
                sensor_pose = (t.translation.x, t.translation.y, quaternion_to_yaw(t.rotation))
                reports[name] = swept_scan_report(
                    scan, sensor_pose, direction, distance, self.lateral_half_x, self.lateral_half_y,
                    self_mask=self.lateral_self_mask, sensor_z=t.translation.z,
                )
                clear = clear and reports[name]['clear']
        except (TransformException, ValueError, TypeError, AttributeError) as exc:
            reports['reason'] = f'transform_or_geometry_error: {exc}'
            return False
        return clear

    def _try_lateral_recovery(self, now):
        if (not self.lateral_enabled or self.lateral_attempted
                or not self.auto_motion_recovery or self.motion_inhibited
                or self.spin_in_progress or self.spin_waiting_for_stop
                or self.commanded_motion or self.odom_motion):
            return False
        # Count even a rejected attempt, so missing data cannot create a retry loop.
        self.lateral_attempted = True
        if not self._lateral_sensors_ready(now):
            self.get_logger().warning("橫移拒絕：即時雷達／完整四輪回授／里程計不足")
            return False
        directions = [d for d in (1, -1) if self._lateral_path_clear(d, self.lateral_distance)]
        if not directions:
            self.get_logger().warning(
                '橫移拒絕：左右掃掠區域不具充分安全視野；'
                + json.dumps(self.lateral_path_reports, ensure_ascii=False)
            )
            return False
        self._cancel_navigation()
        self.lateral_direction = directions[0]
        self.lateral_anchor = self.last_odom_pose
        self.lateral_started_at = now
        self.lateral_stopped_since = None
        self.lateral_abort_reason = None
        self.lateral_token = str(uuid.uuid4())
        self.lateral_phase = "arming"
        self.recovery_step = "lateral_move"
        self.stable_samples = 0
        self._reset_map_match_history()
        self.get_logger().warning(
            f"首次全域定位未收斂，準備{'左' if directions[0] > 0 else '右'}橫移 "
            f"{self.lateral_distance:.2f} m，速度上限 {self.lateral_cap:.2f} m/s"
        )
        return True

    def _stop_lateral(self, reason=None):
        if self.lateral_phase != "stopping":
            self.lateral_stop_started_at = time.monotonic()
        if reason:
            self.lateral_abort_reason = reason
        self.lateral_phase = "stopping"
        self.recovery_step = "lateral_stop"
        self._publish_lateral_lease(permit=False)
        self.recovery_cmd_pub.publish(Twist())

    def _tick_lateral_motion(self):
        if self.lateral_phase is None:
            return
        now = time.monotonic()
        phase = self.lateral_phase
        guard_fresh = now - self.lateral_guard_at <= 0.25
        guard_matches = (guard_fresh and self.lateral_guard.get("active") is True
                         and self.lateral_guard.get("token") == self.lateral_token)
        if phase == "disarming":
            self._publish_lateral_lease(active=False)
            self.recovery_cmd_pub.publish(Twist())
            if guard_fresh and self.lateral_guard.get("active") is False:
                self.lateral_phase = None
                self.lateral_token = None
                if self.lateral_abort_reason:
                    self._require_manual("橫移已停止：" + self.lateral_abort_reason)
                else:
                    self._start_global_recovery()
            return
        self._publish_lateral_lease(permit=phase in {"authorize", "moving"})
        if phase == "stopping":
            self.recovery_cmd_pub.publish(Twist())
            if self.last_odom_pose is not None:
                travelled, cross, angle = lateral_progress(
                    self.lateral_anchor, self.last_odom_pose, self.lateral_direction
                )
                if (not all(math.isfinite(v) for v in (travelled, cross, angle))
                        or travelled > self.lateral_distance + 0.03
                        or abs(cross) > 0.05 or abs(angle) > math.radians(5)):
                    self.lateral_abort_reason = "橫移煞停距離／角度偏差超限"
            stopped = (self._lateral_sensors_ready(now) and not self.commanded_motion
                       and not self.odom_motion and now - self.latest_cmd_at <= 0.25)
            self.lateral_stopped_since = (
                (self.lateral_stopped_since or now) if stopped else None
            )
            if self.lateral_stopped_since and now - self.lateral_stopped_since >= 1.0:
                self.lateral_phase = "disarming"
            elif now - self.lateral_stop_started_at > 3.0 and self.state != "MANUAL_REQUIRED":
                # Report the fault, but KEEP the stop lease latched until fresh
                # physical feedback proves a stop. Do not unlock on a timer.
                self._require_manual(self.lateral_abort_reason or "無新鮮回授確認橫移已停妥")
            return  # Missing feedback latches STOP, never grants new motion.
        if now - self.lateral_started_at > 8.0 or not self._lateral_sensors_ready(now):
            self._stop_lateral("動作逾時或感測資料逾時／取還中")
            return
        if phase == "arming":
            self.recovery_cmd_pub.publish(Twist())
            if guard_matches:
                self.lateral_phase = "authorize"
            return
        if phase == "authorize":
            self.recovery_cmd_pub.publish(Twist())
            if guard_matches and self.lateral_guard.get("blocked") is False:
                self.lateral_phase = "moving"
                self.lateral_last_tick_at = now
                self.lateral_last_pose = self.last_odom_pose
            return
        if not guard_matches or self.lateral_guard.get("blocked") is not False:
            self._stop_lateral("融合節點安全許可逾時")
            return
        progress, cross_track, yaw_error = lateral_progress(
            self.lateral_anchor, self.last_odom_pose, self.lateral_direction
        )
        tick_gap = now - self.lateral_last_tick_at
        step_distance = math.hypot(
            self.last_odom_pose[0] - self.lateral_last_pose[0],
            self.last_odom_pose[1] - self.lateral_last_pose[1],
        )
        if tick_gap > 0.25 or step_distance > max(0.04, self.lateral_cap * tick_gap + 0.03):
            self._stop_lateral("控制迴圈逾時或里程計跳躍")
            return
        self.lateral_last_tick_at, self.lateral_last_pose = now, self.last_odom_pose
        if (not all(math.isfinite(v) for v in (progress, cross_track, yaw_error))
                or progress < -0.03 or progress > self.lateral_distance + 0.03
                or abs(cross_track) > 0.05 or abs(yaw_error) > math.radians(5)):
            self._stop_lateral("里程計軌跡偏移或旋轉超出限制")
            return
        speed = lateral_speed(progress, self.lateral_distance, self.lateral_cap)
        if speed == 0.0:
            self._stop_lateral()
            return
        if not self._lateral_path_clear(self.lateral_direction,
                                        self.lateral_distance - progress + 0.07):
            self._stop_lateral("橫移途中障礙物／盲區／TF 失效")
            return
        command = Twist()
        command.linear.y = self.lateral_direction * speed
        self.recovery_cmd_pub.publish(command)

    def _send_next_spin(self):
        if self.spin_in_progress:
            return
        if not self.spin_sequence:
            self._complete_spin_sequence()
            return
        goal = Spin.Goal()
        goal.target_yaw = float(self.spin_sequence.pop(0))
        goal.time_allowance = Duration(sec=self.spin_time_allowance)
        self.spin_goal_target = goal.target_yaw
        self.spin_goal_started_at = time.monotonic()
        odom_yaw = (
            "unknown"
            if self.last_odom_pose is None
            else f"{math.degrees(self.last_odom_pose[2]):+.1f} deg"
        )
        amcl_yaw = (
            "unknown"
            if self.latest_pose is None
            else f"{math.degrees(self.latest_pose[2]):+.1f} deg"
        )
        self.get_logger().info(
            "送出定位 Spin："
            f"target={math.degrees(goal.target_yaw):+.1f} deg, "
            f"odom_yaw={odom_yaw}, amcl_yaw={amcl_yaw}"
        )
        self.spin_in_progress = True
        future = self.spin_client.send_goal_async(goal)
        future.add_done_callback(self._spin_goal_response)

    def _spin_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:  # ROS middleware exception must stop autonomous motion
            self.spin_in_progress = False
            self._require_manual(f"Spin 送出失敗：{exc}")
            return
        if not goal_handle.accepted:
            self.spin_in_progress = False
            self._require_manual("Spin 被行為伺服器拒絕，可能沒有安全旋轉空間")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._spin_result)

    def _spin_result(self, future):
        self.spin_in_progress = False
        try:
            result = future.result()
        except Exception as exc:  # ROS middleware exception must stop autonomous motion
            self._require_manual(f"Spin 執行失敗：{exc}")
            return
        elapsed = max(0.0, time.monotonic() - self.spin_goal_started_at)
        current_error = self._heading_restore_command(tolerance=0.0)
        error_text = (
            "unknown"
            if current_error is None
            else f"{math.degrees(current_error):+.1f} deg"
        )
        self.get_logger().info(
            "定位 Spin 結果："
            f"target={math.degrees(self.spin_goal_target):+.1f} deg, "
            f"status={result.status}, elapsed={elapsed:.1f}s, "
            f"heading_error={error_text}"
        )
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.spin_pending_failure_reason = (
                f"Spin 未安全完成（action status={result.status}）"
            )
            self.spin_waiting_for_stop = True
            self.spin_settle_started = time.monotonic()
            self.recovery_step = "spin_failure_settle"
            return
        self.spin_waiting_for_stop = True
        self.spin_settle_started = time.monotonic()
        self.recovery_step = "spin_settle"

    def _tick_spin_settle(self, now):
        """Wait out velocity-smoother deceleration before measuring final yaw."""
        elapsed = now - self.spin_settle_started
        stopped_for = now - max(
            self.spin_settle_started,
            self.last_motion_command_at,
            self.last_odom_motion_at,
        )
        if (
            not self.commanded_motion
            and not self.odom_motion
            and stopped_for >= self.spin_settle
        ):
            self.spin_waiting_for_stop = False
            if self.spin_pending_failure_reason:
                reason = self.spin_pending_failure_reason
                self.spin_pending_failure_reason = None
                self._handle_spin_failure(reason)
                return
            self._send_next_spin()
            return
        if elapsed >= self.spin_settle_timeout:
            self._require_manual("Spin 完成後底盤速度未在期限內歸零")

    def _heading_restore_command(self, *, tolerance=None):
        if self.recovery_start_yaw is None or self.last_odom_pose is None:
            return None
        return heading_correction(
            self.recovery_start_yaw,
            self.last_odom_pose[2],
            self.spin_heading_tolerance if tolerance is None else tolerance,
        )

    def _queue_heading_correction(self, correction, message_prefix):
        if not heading_error_is_improving(
            self.spin_previous_heading_error,
            correction,
            self.spin_heading_min_improvement,
        ):
            previous = self.spin_previous_heading_error
            self.get_logger().warning(
                "停止定位回正以避免反向振盪："
                f"previous={math.degrees(previous):+.1f} deg, "
                f"current={math.degrees(correction):+.1f} deg"
            )
            return False
        if self.spin_heading_corrections >= self.spin_heading_max_corrections:
            self.get_logger().warning(
                "定位掃描回正已達修正次數上限；"
                f"仍殘留 {math.degrees(correction):+.1f} deg"
            )
            return False

        command = damped_heading_command(
            correction,
            self.spin_heading_correction_gain,
            self.spin_heading_max_correction,
        )
        self.spin_previous_heading_error = correction
        self.spin_heading_corrections += 1
        self.spin_restoring_heading = True
        self.recovery_step = "restore_heading"
        self.spin_sequence = [command]
        self.get_logger().warning(
            f"{message_prefix}：residual={math.degrees(correction):+.1f} deg, "
            f"command={math.degrees(command):+.1f} deg "
            f"({self.spin_heading_corrections}/"
            f"{self.spin_heading_max_corrections})"
        )
        self._send_next_spin()
        return True

    def _complete_spin_sequence(self):
        correction = self._heading_restore_command()
        if correction is not None and correction != 0.0:
            if self._queue_heading_correction(
                correction, "定位掃描完成後衰減回正"
            ):
                return

        self.spin_restoring_heading = False
        self.recovery_start_yaw = None
        if self.spin_failure_reason:
            self._finish_failed_small_spin()
        else:
            self._finish_spin_sequence()

    def _finish_spin_sequence(self):
        self._request_nomotion_update()
        self.recovery_step = (
            "post_global" if self.state == "RECOVERING_GLOBAL" else "post_small"
        )
        self.recovery_step_started = time.monotonic()

    def _handle_spin_failure(self, reason):
        if self.spin_restoring_heading:
            self._require_manual(f"{reason}；回到定位掃描原始方向也未安全完成")
            return

        self.spin_sequence = []
        self.spin_failure_reason = reason
        correction = self._heading_restore_command()
        if correction is None:
            self._require_manual(f"{reason}；沒有里程計角度可安全回正")
            return
        if correction == 0.0:
            self.recovery_start_yaw = None
            self._finish_failed_small_spin()
            return

        if self._queue_heading_correction(
            correction, f"{reason}；先衰減回到掃描前方向"
        ):
            return
        self._finish_failed_small_spin()

    def _finish_failed_small_spin(self):
        reason = self.spin_failure_reason or "定位掃描未安全完成"
        self.spin_failure_reason = None
        if self.state == "RECOVERING_GLOBAL":
            self._request_nomotion_update()
            self.recovery_step = "post_global"
            self.recovery_step_started = time.monotonic()
            self.get_logger().warning(
                f"{reason}；底盤已停止，先用現有雷達與 AMCL 資料驗證定位"
            )
            return
        self._request_nomotion_update()
        self.recovery_step = "post_failed_small"
        self.recovery_step_started = time.monotonic()
        self.get_logger().warning(
            f"{reason}；車頭已回正，先用現有雷達資料重新驗證定位"
        )

    def _require_manual(self, reason):
        if self.lateral_phase is not None and self.lateral_phase != "disarming":
            self._stop_lateral(reason)
        self.spin_sequence = []
        self.spin_in_progress = False
        self.spin_waiting_for_stop = False
        self.spin_restoring_heading = False
        self.spin_heading_corrections = 0
        self.spin_previous_heading_error = None
        self.spin_failure_reason = None
        self.spin_pending_failure_reason = None
        self.recovery_start_yaw = None
        self.transient_recovery_eligible = False
        self.transient_recovery_started_at = 0.0
        self.recovery_step = None
        self._cancel_navigation()
        self._set_state("MANUAL_REQUIRED", reason)
        self.get_logger().error("請在 RViz 使用 2D Pose Estimate 重新設定初始位置")


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.lateral_phase is not None:
            node._stop_lateral("定位管理器關閉")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
