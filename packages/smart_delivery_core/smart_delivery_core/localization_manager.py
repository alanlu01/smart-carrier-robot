import json
import math
import time

import rclpy
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
    map_match_status,
    occupancy_match_score,
    pose_is_near,
    pose_jump,
    pose_quality,
    quaternion_to_yaw,
    update_stability_samples,
)


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
        self.motion_cmd_topic = str(self.get_parameter("motion_cmd_topic").value)
        self.scan_timeout = float(self.get_parameter("scan_timeout_sec").value)
        self.amcl_pose_timeout = float(
            self.get_parameter("amcl_pose_timeout_sec").value
        )
        self.amcl_nomotion_refresh = float(
            self.get_parameter("amcl_nomotion_refresh_sec").value
        )
        self.verify_timeout = float(self.get_parameter("verify_timeout_sec").value)
        self.stable_samples_required = int(self.get_parameter("stable_samples").value)
        self.healthy_xy_std = float(self.get_parameter("healthy_xy_std").value)
        self.healthy_yaw_std = float(self.get_parameter("healthy_yaw_std").value)
        self.critical_xy_std = float(self.get_parameter("critical_xy_std").value)
        self.critical_yaw_std = float(self.get_parameter("critical_yaw_std").value)
        self.seed_xy_tolerance = float(self.get_parameter("seed_xy_tolerance").value)
        self.seed_yaw_tolerance = float(self.get_parameter("seed_yaw_tolerance").value)
        self.suspect_hold = float(self.get_parameter("suspect_hold_sec").value)
        self.cancel_grace = float(self.get_parameter("cancel_grace_sec").value)
        self.local_wait = float(self.get_parameter("local_recovery_wait_sec").value)
        self.global_wait = float(self.get_parameter("global_recovery_wait_sec").value)
        self.small_spin_angle = float(self.get_parameter("small_spin_angle").value)
        self.full_spin_angle = float(self.get_parameter("full_spin_angle").value)
        self.spin_time_allowance = int(self.get_parameter("spin_time_allowance_sec").value)
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

        self.state = "UNINITIALIZED"
        self.state_since = time.monotonic()
        self.state_reason = "node started"
        self.ready = False
        self.latest_scan_at = 0.0
        self.latest_pose = None
        self.latest_quality = None
        self.last_amcl_at = 0.0
        self.stable_samples = 0
        self.last_stable_map_score_at = 0.0
        self.verify_reference_required = True
        self.suspect_since = None
        self.recovery_step = None
        self.recovery_step_started = 0.0
        self.recovery_reason = ""
        self.spin_sequence = []
        self.spin_in_progress = False
        self.commanded_motion = False
        self.stationary_odom_anchor = None
        self.stationary_amcl_anchor = None
        self.last_odom_pose = None
        self.manual_initial_pose_pending = False
        self.initial_pose_last_published = 0.0
        self.motion_inhibited = False
        self.map_message = None
        self.latest_scan = None
        self.latest_map_score = None
        self.latest_map_score_at = 0.0
        self.last_map_match_check_at = 0.0
        self.last_nomotion_request_at = 0.0

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
            LaserScan, "/scan_filtered", self._scan_callback, qos_profile_sensor_data
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
        self.timer = self.create_timer(0.2, self._tick)
        self._publish_state()
        self.get_logger().info(
            "定位管理器已啟動；固定初始位置為 "
            f"({self.initial_x:.2f}, {self.initial_y:.2f}, {self.initial_yaw:.2f})；"
            f"實際底盤命令監看 {self.motion_cmd_topic}"
        )

    def _declare_parameters(self):
        defaults = {
            "initial_pose.x": 0.0,
            "initial_pose.y": 0.0,
            "initial_pose.yaw": 0.0,
            "initial_pose.xy_std": 0.25,
            "initial_pose.yaw_std": 0.26,
            "auto_initialize": True,
            "motion_cmd_topic": "/chassis_cmd_vel",
            "scan_timeout_sec": 2.0,
            "amcl_pose_timeout_sec": 8.0,
            "amcl_nomotion_refresh_sec": 1.0,
            "verify_timeout_sec": 15.0,
            "stable_samples": 5,
            "healthy_xy_std": 0.20,
            "healthy_yaw_std": 0.17,
            "critical_xy_std": 0.50,
            "critical_yaw_std": 0.45,
            "seed_xy_tolerance": 0.60,
            "seed_yaw_tolerance": 0.52,
            "suspect_hold_sec": 5.0,
            "cancel_grace_sec": 2.0,
            "local_recovery_wait_sec": 6.0,
            "global_recovery_wait_sec": 12.0,
            "small_spin_angle": math.radians(20.0),
            "full_spin_angle": 2.0 * math.pi,
            "spin_time_allowance_sec": 35,
            "pose_jump_distance": 0.40,
            "pose_jump_angle": math.radians(25.0),
            "external_move_distance": 0.25,
            "external_move_angle": math.radians(20.0),
            "automatic_motion_recovery": True,
            "map_match_min_score": 0.35,
            "map_match_critical_score": 0.20,
            "map_match_check_period_sec": 1.0,
            "map_match_max_beams": 60,
            "map_match_min_beams": 10,
            "map_match_neighborhood_cells": 2,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

    def _set_state(self, state, reason):
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
        message = String()
        payload = {
            "state": self.state,
            "ready": self.ready,
            "reason": self.state_reason,
            "scan_age_sec": (
                None if not self.latest_scan_at else time.monotonic() - self.latest_scan_at
            ),
        }
        if self.latest_quality is not None:
            payload.update(
                {
                    "xy_std": round(self.latest_quality.xy_std, 4),
                    "yaw_std": round(self.latest_quality.yaw_std, 4),
                }
            )
        if self.latest_map_score is not None:
            payload["map_match_score"] = round(self.latest_map_score, 4)
        message.data = json.dumps(payload, ensure_ascii=False)
        self.state_publisher.publish(message)

    def _scan_callback(self, message):
        self.latest_scan_at = time.monotonic()
        self.latest_scan = message

    def _map_callback(self, message):
        self.map_message = message

    def _cmd_vel_callback(self, message):
        self.commanded_motion = (
            math.hypot(float(message.linear.x), float(message.linear.y)) > 0.02
            or abs(float(message.angular.z)) > 0.03
        )
        if self.commanded_motion:
            self.stationary_odom_anchor = None
            self.stationary_amcl_anchor = None

    def _motion_inhibited_callback(self, message):
        self.motion_inhibited = bool(message.data)

    def _odom_callback(self, message):
        pose = message.pose.pose
        current = (
            float(pose.position.x),
            float(pose.position.y),
            quaternion_to_yaw(pose.orientation),
        )
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
            map_match_ok = (
                self.latest_map_score is not None
                and self.latest_map_score >= self.map_match_min_score
                and now - self.latest_map_score_at <= 2.0 * self.map_match_period
            )
            qualified = (
                quality.healthy
                and reference_ok
                and map_match_ok
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

    def _external_initial_pose_callback(self, _message):
        if self.manual_initial_pose_pending:
            self.manual_initial_pose_pending = False
            return
        if self.state == "MANUAL_REQUIRED":
            self.stable_samples = 0
            self.last_stable_map_score_at = self.latest_map_score_at
            self.verify_reference_required = False
            self._set_state("VERIFYING", "收到人工初始位置，正在驗證")

    def _scan_is_fresh(self, now=None):
        now = time.monotonic() if now is None else now
        return self.latest_scan_at > 0.0 and now - self.latest_scan_at <= self.scan_timeout

    def _tick(self):
        now = time.monotonic()
        if not self._scan_is_fresh(now):
            if self.state == "LOCALIZED":
                self._begin_recovery("雷達資料逾時")
            elif self.state == "UNINITIALIZED":
                self._set_state("WAITING_FOR_SENSORS", "等待 /scan_filtered")
            self._publish_state()
            return

        if now - self.last_map_match_check_at >= self.map_match_period:
            self.last_map_match_check_at = now
            self._update_map_match_score(now)

        if self.state in {"UNINITIALIZED", "WAITING_FOR_SENSORS"}:
            if not self.auto_initialize:
                self._require_manual("自動初始位置已停用")
                return
            if self.initial_pose_publisher.get_subscription_count() < 1:
                self._set_state("WAITING_FOR_SENSORS", "等待 AMCL /initialpose 訂閱者")
                return
            self._publish_initial_pose()
            return

        if self.state == "VERIFYING":
            if now - self.last_nomotion_request_at >= 1.0:
                self._request_nomotion_update()
            if (
                self.last_amcl_at < self.initial_pose_last_published
                and now - self.initial_pose_last_published >= 2.0
                and self.initial_pose_publisher.get_subscription_count() >= 1
            ):
                self._publish_initial_pose(retry=True)
            if now - self.state_since >= self.verify_timeout:
                self._begin_recovery("初始位置驗證逾時")
            return

        if self.state == "LOCALIZED":
            if self.latest_quality is None:
                self._begin_recovery("AMCL 位姿資料逾時")
                return
            amcl_pose_age = now - self.last_amcl_at
            if (
                not self.commanded_motion
                and amcl_pose_age >= self.amcl_nomotion_refresh
                and now - self.last_nomotion_request_at >= self.amcl_nomotion_refresh
            ):
                self._request_nomotion_update()
            if amcl_pose_age > self.amcl_pose_timeout:
                self._begin_recovery("AMCL 位姿資料逾時")
                return
            match_status = self._map_match_status(now)
            map_match_bad = match_status in {"critical", "degraded"}
            if self.latest_quality.critical or map_match_bad:
                if self.suspect_since is None:
                    self.suspect_since = now
                    self.stable_samples = 0
                    self.last_stable_map_score_at = self.latest_map_score_at
                    if match_status == "critical":
                        reason = f"雷達地圖吻合度過低 ({self.latest_map_score:.2f})"
                    elif match_status == "degraded":
                        reason = (
                            f"雷達地圖吻合度未達可信門檻 "
                            f"({self.latest_map_score:.2f} < {self.map_match_min_score:.2f})"
                        )
                    else:
                        reason = "AMCL 不確定度持續偏高"
                    self._set_state("SUSPECT", reason)
            else:
                self.suspect_since = None
            return

        if self.state == "SUSPECT":
            match_status = self._map_match_status(now)
            qualified = (
                self.latest_quality is not None
                and self.latest_quality.healthy
                and match_status == "healthy"
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
            elif (
                not qualified
                and self.suspect_since is not None
                and now - self.suspect_since >= self.suspect_hold
            ):
                if match_status in {"critical", "degraded", "unknown"}:
                    self._begin_recovery("雷達地圖吻合度持續低於可信門檻")
                else:
                    self._begin_recovery("AMCL 不確定度超過保守門檻")
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
        except TransformException:
            return

        valid = []
        for index, distance in enumerate(scan.ranges):
            distance = float(distance)
            if not math.isfinite(distance):
                continue
            if distance < float(scan.range_min) or distance >= float(scan.range_max) * 0.995:
                continue
            valid.append((index, distance))
        if len(valid) < self.map_match_min_beams:
            return
        stride = max(1, math.ceil(len(valid) / self.map_match_max_beams))
        sampled = valid[::stride][: self.map_match_max_beams]

        laser_yaw = quaternion_to_yaw(transform.transform.rotation)
        cosine = math.cos(laser_yaw)
        sine = math.sin(laser_yaw)
        translation = transform.transform.translation
        endpoints = []
        for index, distance in sampled:
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
            self.latest_map_score = score
            self.latest_map_score_at = now

    def _map_match_status(self, now):
        return map_match_status(
            self.latest_map_score,
            now - self.latest_map_score_at,
            self.map_match_min_score,
            self.map_match_critical_score,
            2.0 * self.map_match_period,
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
        self.last_stable_map_score_at = self.latest_map_score_at
        self.verify_reference_required = True
        if not retry:
            self._set_state("SEEDING", "發布固定開機位置 (0,0,0)")
        self.initial_pose_publisher.publish(message)
        self.initial_pose_last_published = time.monotonic()
        if retry:
            self.get_logger().warning("尚未收到 AMCL 位姿，重新發布固定初始位置")
        else:
            self._set_state("VERIFYING", "等待 AMCL 收斂")

    def _mark_localized(self):
        self.suspect_since = None
        self.recovery_step = None
        self.spin_sequence = []
        self.spin_in_progress = False
        self.stationary_odom_anchor = self.last_odom_pose
        self.stationary_amcl_anchor = self.latest_pose
        quality = self.latest_quality
        reason = "AMCL 已穩定"
        if quality is not None:
            reason += (
                f"，xy_std={quality.xy_std:.3f} m，"
                f"yaw_std={math.degrees(quality.yaw_std):.1f} deg"
            )
        self._set_state("LOCALIZED", reason)

    def _cancel_navigation(self):
        request = CancelGoal.Request()
        for client in (self.navigate_cancel_client, self.poses_cancel_client):
            if client.service_is_ready():
                client.call_async(request)

    def _begin_recovery(self, reason):
        if self.state in {"RECOVERING_LOCAL", "RECOVERING_GLOBAL", "MANUAL_REQUIRED"}:
            return
        self._cancel_navigation()
        self.stable_samples = 0
        self.last_stable_map_score_at = self.latest_map_score_at
        self.stationary_odom_anchor = None
        self.stationary_amcl_anchor = None
        self.verify_reference_required = False
        self.recovery_reason = reason
        self.recovery_step = "cancel_wait"
        self.recovery_step_started = time.monotonic()
        self._set_state("RECOVERING_LOCAL", reason)

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
        if now - self.last_nomotion_request_at >= self.amcl_nomotion_refresh:
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
        elapsed = now - self.recovery_step_started
        if self.recovery_step == "cancel_wait" and elapsed >= self.cancel_grace:
            self._request_nomotion_update()
            self.recovery_step = "nomotion_wait"
            self.recovery_step_started = now
        elif self.recovery_step == "nomotion_wait":
            self._request_nomotion_if_due(now)
            if elapsed >= self.local_wait:
                if not self.auto_motion_recovery:
                    self._require_manual("自動旋轉恢復已停用")
                else:
                    self._start_small_sweep()
        elif self.recovery_step == "post_small":
            self._request_nomotion_if_due(now)
            if elapsed >= self.local_wait:
                self._start_global_recovery()
        elif self.recovery_step == "motion_inhibited" and not self.motion_inhibited:
            self._start_small_sweep()

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
        self.recovery_step = "small_spin"
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
        if not self.global_localization_client.service_is_ready():
            self._require_manual("AMCL 全域重新定位服務未就緒")
            return
        self.global_localization_client.call_async(Empty.Request())
        self.stable_samples = 0
        self.last_stable_map_score_at = self.latest_map_score_at
        self.recovery_step = "global_settle"
        self.recovery_step_started = time.monotonic()
        self._set_state("RECOVERING_GLOBAL", "全域粒子重新定位")

    def _tick_global_recovery(self, now):
        elapsed = now - self.recovery_step_started
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
            self.spin_sequence = [self.full_spin_angle]
            self.get_logger().warning("開始碰撞檢查的 360 度低速定位自旋")
            self._send_next_spin()
        elif self.recovery_step == "post_global":
            self._request_nomotion_if_due(now)
            if elapsed >= self.global_wait:
                self._require_manual("全域重新定位後仍未通過可信度檢查")

    def _send_next_spin(self):
        if self.spin_in_progress:
            return
        if not self.spin_sequence:
            self._request_nomotion_update()
            self.recovery_step = (
                "post_global" if self.state == "RECOVERING_GLOBAL" else "post_small"
            )
            self.recovery_step_started = time.monotonic()
            return
        goal = Spin.Goal()
        goal.target_yaw = float(self.spin_sequence.pop(0))
        goal.time_allowance = Duration(sec=self.spin_time_allowance)
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
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self._require_manual(
                f"Spin 未安全完成（action status={result.status}），停止自動恢復"
            )
            return
        self._send_next_spin()

    def _require_manual(self, reason):
        self.spin_sequence = []
        self.spin_in_progress = False
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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
