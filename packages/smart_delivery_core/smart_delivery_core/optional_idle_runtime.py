"""Optional motions owned by the existing delivery navigator, never a rival node."""

import json
import math
import os
from pathlib import Path
import time

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import TaskResult
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import TransformException

from smart_delivery_core.optional_idle import DirectionalPeople, HomeVoltageLatch, occupancy_fingerprint

STATE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)


def yaw_of(quaternion):
    """Quaternion yaw without optional numerical dependencies."""
    q = quaternion
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class OptionalIdleRuntime:
    """Run at most one bounded, interruptible auxiliary NavigateToPose goal."""

    def __init__(self, navigator, buffer, config, share, ready, navigation_active, suspend_lease):
        self.navigator, self.buffer, self.config = navigator, buffer, config
        self.share, self.ready, self.navigation_active = Path(share), ready, navigation_active
        self.suspend_lease = suspend_lease
        self.state = 'idle'
        self.motion = False
        self.reason = ''
        self.goal = None
        self.deadline = 0.0
        self.cancel_requested = False
        self.cancel_started = None
        self.goal_request = None
        self.cancel_future = None
        self.last_cancel_at = 0.0
        self.last_odom = None
        self.last_scan = None
        self.map_matches = False
        self.idle_since = None
        self.next_people_at = 0.0
        self.people = None
        self.spin_yaw = None
        self.spin_travel = 0.0
        self.stop_since = None
        self.settling_after_cancel = False
        self.parked_point = None
        self.latch_path = Path.home() / '.ros/smart_carrier_home_latch.json'
        self.home = None
        if config['home']['enabled']:
            saved = False
            if self.latch_path.exists():
                # An unreadable latch must fail closed, not silently admit tasks.
                saved = bool(json.loads(self.latch_path.read_text())['latched'])
            self.home = HomeVoltageLatch(config['home'], latched=saved)
            navigator.create_subscription(String, config['home']['battery_topic'], self._battery, 10)
            self.admission_pub = navigator.create_publisher(
                String, '/smart_carrier/task_admission', STATE_QOS)
            navigator.create_service(Trigger, '/smart_carrier/reset_home_latch', self._reset)
        else:
            self.admission_pub = None
        self.status_pub = navigator.create_publisher(String, '/smart_carrier/optional_idle_state', STATE_QOS)
        navigator.create_subscription(OccupancyGrid, '/map', self._map, STATE_QOS)
        navigator.create_subscription(Odometry, '/odom', self._odom, 10)
        navigator.create_subscription(Bool, '/localization/motion_inhibited', self._inhibited, STATE_QOS)
        self.inhibited = False
        if config['people']['enabled']:
            navigator.create_subscription(LaserScan, '/scan_filtered', self._scan, qos_profile_sensor_data)
            navigator.create_subscription(String, '/vision/semantic_info', self._vision, 10)
        navigator.create_timer(0.5, self._heartbeat)

    @property
    def home_requested(self):
        return self.home is not None and self.home.latched

    @property
    def accepting_tasks(self):
        if self.home is None:
            return True
        self.home.expire(time.monotonic())
        return self.map_matches and self.home.healthy and not self.home.latched

    def _map(self, message):
        self.map_matches = occupancy_fingerprint(message) == self.config['map']['occupancy_fingerprint']

    def _inhibited(self, message):
        self.inhibited = bool(message.data)

    def _scan(self, message):
        self.last_scan = (time.monotonic(), message)

    def _odom(self, message):
        now = time.monotonic()
        previous = self.last_odom
        self.last_odom = (now, message)
        if self.state == 'observing' and previous is not None:
            old = yaw_of(previous[1].pose.pose.orientation)
            new = yaw_of(message.pose.pose.orientation)
            self.spin_travel += math.atan2(math.sin(new - old), math.cos(new - old))

    def _battery(self, message):
        try:
            payload = json.loads(message.data)
        except (ValueError, TypeError):
            payload = {}
        was_latched = self.home.latched
        self.home.observe(payload, time.monotonic(), self.navigator.get_clock().now().nanoseconds / 1e9)
        if self.home.latched and not was_latched:
            self._save_latch(True)
            self.navigator.get_logger().warning('大電池持續低電壓：停止新接單，當前任務結束後回家')

    def _save_latch(self, latched):
        self.latch_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.latch_path.with_suffix('.tmp')
        temporary.write_text(json.dumps({'latched': latched}), encoding='utf-8')
        os.replace(temporary, self.latch_path)

    def _reset(self, request, response):
        self.home.expire(time.monotonic())
        if (self.motion or self.state != 'home_arrived' or not self.home.healthy
                or self.home.last_voltage < self.config['home']['low_voltage_v'] + 0.5
                or not self._stationary()):
            response.success = False
            response.message = '需已停妥在家且確認大電池電壓恢復後，才能人工解除回家鎖定'
            return response
        self._save_latch(False)
        self.home.latched = False
        self.home.low_since = None
        self.state = 'idle'
        response.success, response.message = True, '回家鎖定已人工解除'
        return response

    def _heartbeat(self):
        if self.home:
            self.home.expire(time.monotonic())
        stamp = self.navigator.get_clock().now().nanoseconds / 1e9
        if self.admission_pub:
            message = String()
            message.data = json.dumps({'source': 'optional_idle', 'stamp_sec': stamp,
                                       'accepting': self.accepting_tasks,
                                       'reason': 'home_latched' if self.home_requested else 'battery_health'})
            self.admission_pub.publish(message)
        message = String()
        message.data = json.dumps({'stamp_sec': stamp, 'state': self.state, 'reason': self.reason,
                                   'home_latched': self.home_requested, 'map_matches': self.map_matches,
                                   'accepting_tasks': self.accepting_tasks})
        self.status_pub.publish(message)

    def _stamp_fresh(self, receipt, stamp, limit=0.30):
        now = self.navigator.get_clock().now().nanoseconds
        source = stamp.sec * 1_000_000_000 + stamp.nanosec
        return (0 <= time.monotonic() - receipt <= limit and source > 0
                and -0.1 <= (now - source) / 1e9 <= limit)

    def _pose(self, stamp=None):
        try:
            transform = self.buffer.lookup_transform('map', 'base_footprint', stamp or Time())
        except TransformException:
            return None
        now = self.navigator.get_clock().now().nanoseconds
        source = transform.header.stamp.sec * 1_000_000_000 + transform.header.stamp.nanosec
        if not -0.1 <= (now - source) / 1e9 <= 0.30:
            return None
        pose = {'x': transform.transform.translation.x, 'y': transform.transform.translation.y,
                'yaw': yaw_of(transform.transform.rotation)}
        return pose if all(math.isfinite(value) for value in pose.values()) else None

    def _stationary(self):
        if not self.last_odom or not self._stamp_fresh(self.last_odom[0], self.last_odom[1].header.stamp):
            return False
        velocity = self.last_odom[1].twist.twist
        return max(abs(velocity.linear.x), abs(velocity.linear.y)) <= 0.02 and abs(velocity.angular.z) <= 0.03

    def _safe(self):
        if self.home is not None:
            self.home.expire(time.monotonic())
            if not self.home.healthy:
                return False
        return (self.map_matches and self.ready() and not self.inhibited
                and self.last_odom is not None
                and self._stamp_fresh(self.last_odom[0], self.last_odom[1].header.stamp)
                and self._pose() is not None)

    def _clear_to_spin(self, require_stationary=True):
        if (not self._safe() or (require_stationary and not self._stationary())
                or self.last_scan is None):
            return False
        receipt, scan = self.last_scan
        if not self._stamp_fresh(receipt, scan.header.stamp) or not scan.ranges:
            return False
        try:
            transform = self.buffer.lookup_transform('base_footprint', scan.header.frame_id, Time())
        except TransformException:
            return False
        q = transform.transform.rotation
        tx, ty = transform.transform.translation.x, transform.transform.translation.y
        if not all(math.isfinite(value) for value in (
                q.x, q.y, q.z, q.w, tx, ty, scan.angle_min, scan.angle_increment,
                scan.range_min, scan.range_max)):
            return False
        if abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1.) > .05:
            return False
        if not 6.0 <= abs(scan.angle_increment) * len(scan.ranges) <= 2 * math.pi + .1:
            return False
        xx, xy = 1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y - q.z * q.w)
        yx, yy = 2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q.x * q.x + q.z * q.z)
        radius = math.hypot(0.22, 0.16) + self.config['people']['scan_clearance_m']
        valid = 0
        known = []
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max:
                known.append(False)
                continue
            known.append(True)
            valid += 1
            angle = scan.angle_min + index * scan.angle_increment
            lx, ly = distance * math.cos(angle), distance * math.sin(angle)
            if math.hypot(tx + xx * lx + xy * ly, ty + yx * lx + yy * ly) <= radius:
                return False
        longest, run = 0, 0
        for available in known + known:
            run = 0 if available else run + 1
            longest = max(longest, run)
        unknown_arc = min(longest, len(known)) * abs(scan.angle_increment) * 180 / math.pi
        return (valid / len(scan.ranges) >= 0.75
                and unknown_arc <= self.config['people']['max_unknown_arc_deg'])

    def _vision(self, message):
        if self.state != 'observing' or self.people is None:
            return
        try:
            payload = json.loads(message.data)
            stamp = payload['stamp']
            ns = int(stamp['sec']) * 1_000_000_000 + int(stamp['nanosec'])
            age = (self.navigator.get_clock().now().nanoseconds - ns) / 1e9
            if ns <= 0 or not -0.1 <= age <= 0.50 or not isinstance(payload['detections'], list):
                return
            pose = self._pose(Time(nanoseconds=ns))
            if pose is not None:
                self.people.observe(payload['detections'], pose['yaw'], ns)
        except (ValueError, KeyError, TypeError, OverflowError):
            return

    def _start(self, point, state, tree, timeout):
        if not self._safe() or not self.navigator.nav_to_pose_client.server_is_ready():
            self.reason = 'waiting_for_safe_nav2'
            return False
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.navigator.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = float(point['x']), float(point['y'])
        goal.pose.orientation.z = math.sin(point['yaw'] / 2)
        goal.pose.orientation.w = math.cos(point['yaw'] / 2)
        self.navigation_active(True)
        request = NavigateToPose.Goal()
        request.pose, request.behavior_tree = goal, tree
        try:
            self.goal_request = self.navigator.nav_to_pose_client.send_goal_async(
                request, self.navigator._feedbackCallback)
        except Exception:
            # A transport error does not prove the server never received the
            # goal. Leave the guard armed and stop renewing its lease.
            self.suspend_lease()
            raise
        self.goal, self.state, self.motion = dict(point), state, True
        self.deadline = time.monotonic() + timeout
        self.cancel_requested, self.cancel_started = False, None
        self.cancel_future, self.last_cancel_at = None, 0.0
        self.reason = ''
        return True

    def _people_finished(self, reason):
        self.state, self.reason = 'idle', reason
        self.next_people_at = time.monotonic() + self.config['people']['cooldown_sec']
        self.idle_since = None

    def poll(self, has_work):
        """Cancel on unsafe data/new service work and drain before any new goal."""
        if has_work:
            self.idle_since = None
        if (not self.motion and self.state in (
                'waiting_observation_stop', 'people_destination_selected', 'people_arrival_check')
                and (has_work or self.home_requested)):
            # A service can start between auxiliary actions. Never resume an
            # old observation/parking decision from a different position.
            self._people_finished('observation_preempted_between_actions')
        if not self.motion:
            if self.settling_after_cancel:
                if self._stationary():
                    self.stop_since = self.stop_since or time.monotonic()
                    if time.monotonic() - self.stop_since >= 1.0:
                        self.settling_after_cancel = False
                        self.stop_since = None
                        return False
                else:
                    self.stop_since = None
                return True
            return False
        interrupt = (not self._safe() or time.monotonic() >= self.deadline
                     or (self.state != 'going_home' and (has_work or self.home_requested))
                     or (self.home is not None and not self.home.healthy)
                     or (self.state == 'observing' and not self._clear_to_spin(False)))
        if interrupt and not self.cancel_requested:
            self.cancel_requested = True
            self.cancel_started = time.monotonic()
        if self.goal_request is not None:
            if not self.goal_request.done():
                if self.cancel_started and time.monotonic() - self.cancel_started > 5:
                    self.suspend_lease()
                    self.reason = 'goal_acceptance_pending_manual_attention'
                return True
            try:
                handle = self.goal_request.result()
            except Exception:
                self.suspend_lease()
                self.reason = 'goal_request_error_manual_attention'
                return True
            self.goal_request = None
            if not handle.accepted:
                self.motion = False
                self.navigation_active(False)
                if self.state == 'going_home':
                    self.state, self.reason = 'home_failed', 'goal_rejected'
                else:
                    self._people_finished('goal_rejected')
                return False
            self.navigator.goal_handle = handle
            self.navigator.result_future = handle.get_result_async()
        if (self.cancel_requested and time.monotonic() - self.last_cancel_at >= 1.0
                and (self.cancel_future is None or self.cancel_future.done())):
            self.last_cancel_at = time.monotonic()
            try:
                self.cancel_future = self.navigator.goal_handle.cancel_goal_async()
            except Exception:
                self.suspend_lease()
                self.reason = 'goal_cancel_error_manual_attention'
                return True
        if not self.navigator.isTaskComplete():
            # Stop renewing if cancellation stalls; the existing delivery goal
            # guard also cancels this NavigateToPose (including its Spin child).
            if self.cancel_started and time.monotonic() - self.cancel_started > 5:
                self.suspend_lease()
                self.reason = 'cancellation_pending_manual_attention'
            return True
        self.navigation_active(False)
        self.motion = False
        succeeded = (not self.cancel_requested and self.navigator.getResult() == TaskResult.SUCCEEDED)
        if not succeeded:
            self.settling_after_cancel = True
            self.stop_since = None
            if self.state == 'going_home':
                self.state, self.reason = 'home_failed', 'navigation_interrupted_or_failed'
            else:
                self._people_finished('observation_interrupted_or_failed')
        elif self.state == 'going_home':
            self.state, self.stop_since = 'verifying_home', None
            self.deadline = time.monotonic() + 5.0
        elif self.state == 'going_observation':
            self.state = 'waiting_observation_stop'
            self.deadline = time.monotonic() + 5.0
        elif self.state == 'observing':
            if abs(self.spin_travel) < 5.8:
                self._people_finished('insufficient_rotation_coverage')
            else:
                selected = self.people.select(self.config['people']['observation_pose'])
                if selected is None:
                    self._people_finished('insufficient_directional_demand')
                else:
                    self.goal, self.state = selected, 'people_destination_selected'
        elif self.state == 'going_people_standby':
            self.state = 'people_arrival_check'
            self.deadline = time.monotonic() + 5.0
        return self.settling_after_cancel

    def _at_goal(self, point, xy, yaw):
        pose = self._pose()
        return (pose is not None and math.hypot(pose['x'] - point['x'], pose['y'] - point['y']) <= xy
                and abs(math.atan2(math.sin(pose['yaw'] - point['yaw']),
                                   math.cos(pose['yaw'] - point['yaw']))) <= yaw)

    def idle(self):
        """Return True when optional policy owns idle flow, even while stopped."""
        now = time.monotonic()
        if self.home_requested:
            if self.motion or self.state in ('home_arrived', 'home_failed'):
                return True
            if self.state == 'verifying_home':
                home = self.config['home']
                if self._safe() and self._stationary() and self._at_goal(
                        home['pose'], home['xy_tolerance_m'], home['yaw_tolerance_rad']):
                    self.stop_since = self.stop_since or now
                    if now - self.stop_since >= home['stopped_duration_sec']:
                        self.state = 'home_arrived'
                else:
                    self.stop_since = None
                if self.state != 'home_arrived' and now >= self.deadline:
                    self.state, self.reason = 'home_failed', 'precise_arrival_not_verified'
                return True
            if self.home.healthy:
                self._start(self.config['home']['pose'], 'going_home',
                            str(self.share / 'behavior_trees/smart_home_navigate.xml'),
                            self.config['home']['navigation_timeout_sec'])
            return True
        if self.home and not self.accepting_tasks:
            self.reason = 'waiting_verified_main_battery'
            return True
        if not self.config['people']['enabled']:
            return False
        people = self.config['people']
        if self.motion:
            return True
        if self.state == 'waiting_observation_stop':
            if self._clear_to_spin():
                self.people = DirectionalPeople(people)
                self.spin_travel = 0.0
                self._start(people['observation_pose'], 'observing',
                            str(self.share / 'behavior_trees/people_observe.xml'),
                            people['observation_timeout_sec'])
            elif now >= self.deadline:
                self._people_finished('unsafe_spin_clearance_or_sensor_data')
            return True
        if self.state == 'people_destination_selected':
            self._start(self.goal, 'going_people_standby', '', people['navigation_timeout_sec'])
            return True
        if self.state == 'people_arrival_check':
            if self._safe() and self._stationary() and self._at_goal(self.goal, 0.10, 0.10):
                self.parked_point = dict(self.goal)
                self._people_finished('parked_at_selected_standby')
                # Let the ordinary delivery flow know that parking is complete.
                return True
            if now >= self.deadline:
                self._people_finished('standby_arrival_not_verified')
            return True
        self.idle_since = self.idle_since or now
        if now < self.next_people_at or now - self.idle_since < people['idle_before_observation_sec']:
            return False
        self._start(people['observation_pose'], 'going_observation', '', people['navigation_timeout_sec'])
        return True
