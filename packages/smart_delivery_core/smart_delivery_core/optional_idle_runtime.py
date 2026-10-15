"""Optional motions owned by the existing delivery navigator, never a rival node."""

import json
import math
import os
from pathlib import Path
import time
import tempfile
from collections import deque

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
from smart_delivery_core.standby_idle_clock import StandbyIdleClock
from smart_delivery_core.spin_evidence import ScanEvidence, aligned_spin_report

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
        self.idle_clock = StandbyIdleClock()
        self.scan_history = deque(maxlen=5)
        self.spin_cache = None
        self.scan_evidence_error = ''
        self.next_people_at = 0.0
        self.people = None
        self.spin_yaw = None
        self.spin_travel = 0.0
        self.stop_since = None
        self.settling_after_cancel = False
        self.parked_point = None
        self.guard_report = {}
        self.safe_scan_count = 0
        self.safe_scan_stamp = None
        self.safe_stop_since = None
        self.safe_scan_since = None
        self.spin_resumes = 0
        self.spin_resume_requests = 0
        self.spin_resume_pending = False
        self.spin_resume_anchor = 0.
        self.people_failures = 0
        self.observation_collecting = False
        self.observation_session_deadline = None
        self.observation_tree_path = None
        self.interrupt_reason = None
        self.log_key = None
        self.last_progress_log = 0.0
        self.latch_path = Path.home() / '.ros/smart_carrier_home_latch.json'
        self.home = None
        if config['home']['enabled']:
            saved = False
            saved_admission = False
            if self.latch_path.exists():
                # An unreadable latch must fail closed, not silently admit tasks.
                previous = json.loads(self.latch_path.read_text())
                saved = bool(previous['latched'])
                saved_admission = bool(previous.get('admission_latched', saved))
            self.home = HomeVoltageLatch(config['home'], latched=saved)
            self.home.admission_latched = saved or saved_admission
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
        return self.map_matches and self.home.healthy and not self.home.admission_latched and not self.home.latched

    @property
    def stop_accepting(self):
        return self.home is not None and (self.home.admission_latched or self.home.latched)

    def service_may_resume(self, task):
        if self.home is None:
            return True
        physical = task.get('_resume_state') in {'waiting_action', 'wrong_slot', 'verifying_action'}
        if self.home.latched:
            return physical
        return self.home.healthy and (not self.home.admission_latched or bool(task.get('_resume_state')))

    def _map(self, message):
        self.map_matches = occupancy_fingerprint(message) == self.config['map']['occupancy_fingerprint']

    def _inhibited(self, message):
        self.inhibited = bool(message.data)

    def _scan(self, message):
        now = time.monotonic()
        self.last_scan = (now, message)
        self.spin_cache = None
        if self.state not in ('waiting_observation_stop', 'waiting_observation_retry', 'observing'):
            return
        stamp = Time(nanoseconds=message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec)
        try:
            sensor = self.buffer.lookup_transform('odom',message.header.frame_id,stamp).transform
            base = self.buffer.lookup_transform('odom','base_footprint',stamp).transform
            for q in (sensor.rotation,base.rotation):
                if (abs(q.x)>1e-4 or abs(q.y)>1e-4
                        or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.01):
                    raise ValueError('unsupported laser/base tilt or invalid quaternion')
            evidence = ScanEvidence.from_scan(message, now,
                (sensor.translation.x,sensor.translation.y,yaw_of(sensor.rotation)),
                (base.translation.x,base.translation.y,yaw_of(base.rotation)))
            self.scan_history.append(evidence)
            self.scan_evidence_error = ''
        except (TransformException, ValueError, TypeError, AttributeError) as exc:
            self.scan_evidence_error = str(exc)

    def mark_parked(self, point):
        """Only the existing successful arrival flow grants this parking proof."""
        self.idle_clock.mark_parked(point)
        self.idle_since = None

    def _odom(self, message):
        now = time.monotonic()
        previous = self.last_odom
        self.last_odom = (now, message)
        if self.observation_collecting and previous is not None:
            old = yaw_of(previous[1].pose.pose.orientation)
            new = yaw_of(message.pose.pose.orientation)
            self.spin_travel += math.atan2(math.sin(new - old), math.cos(new - old))
            if (self.spin_resume_pending and self.state=='observing' and not self.cancel_requested
                    and self.spin_travel-self.spin_resume_anchor >= .05):
                self.spin_resumes += 1
                self.spin_resume_pending = False

    def _battery(self, message):
        try:
            payload = json.loads(message.data)
        except (ValueError, TypeError):
            payload = {}
        previous = (self.home.latched, self.home.admission_latched)
        self.home.observe(payload, time.monotonic(), self.navigator.get_clock().now().nanoseconds / 1e9)
        if previous != (self.home.latched, self.home.admission_latched):
            self._save_latch(self.home.latched)
            self.navigator.get_logger().warning('大電池狀態更新：stop_claim=%s, return_home=%s' %
                                                (self.home.admission_latched, self.home.latched))

    def _save_latch(self, latched):
        self.latch_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.latch_path.with_suffix('.tmp')
        temporary.write_text(json.dumps({'latched': latched,
            'admission_latched': self.home.admission_latched}), encoding='utf-8')
        os.replace(temporary, self.latch_path)

    def _reset(self, request, response):
        self.home.expire(time.monotonic())
        if (self.motion or self.state != 'home_arrived' or not self.home.healthy
                or not self.home.reset_ready(time.monotonic())
                or not self._safe() or not self._stationary()
                or not self._at_goal(self.config['home']['pose'],
                                     self.config['home']['xy_tolerance_m'],
                                     self.config['home']['yaw_tolerance_rad'])):
            response.success = False
            response.message = '需已停妥在家且確認大電池電壓恢復後，才能人工解除回家鎖定'
            return response
        self.home.latched = False
        self.home.admission_latched = False
        try:
            self._save_latch(False)
        except OSError:
            self.home.latched = self.home.admission_latched = True
            response.success = False
            response.message = '鎖定狀態儲存失敗，維持停車與回家鎖定'
            return response
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
                                       'reason': 'home_latched' if self.home_requested else
                                       'low_battery_stop_claim' if self.stop_accepting else 'battery_health'})
            self.admission_pub.publish(message)
        message = String()
        message.data = json.dumps({'stamp_sec': stamp, 'state': self.state, 'reason': self.reason,
                                   'home_latched': self.home_requested, 'map_matches': self.map_matches,
                                   'stop_claim_latched': self.stop_accepting,
                                   'accepting_tasks': self.accepting_tasks,
                                   'spin_progress_deg': round(math.degrees(self.spin_travel), 1),
                                   'spin_resume_count': self.spin_resumes,
                                   'spin_resume_requests': self.spin_resume_requests,
                                   'spin_resume_pending': self.spin_resume_pending,
                                   'continuous_safe_scan_sec': (0. if self.safe_scan_since is None
                                       else max(0.,time.monotonic()-self.safe_scan_since)),
                                   'consecutive_observation_failures': self.people_failures,
                                   'cooldown_remaining_sec': round(max(0., self.next_people_at-time.monotonic()), 1),
                                   'idle_remaining_sec': None if self.idle_since is None else round(max(0.,
                                       self.config['people']['idle_before_observation_sec']-self.idle_clock.elapsed), 1),
                                   'idle_clock_reason': self.idle_clock.reason,
                                   'idle_elapsed_sec': round(self.idle_clock.elapsed,1),
                                   'spin_guard': self.guard_report})
        payload = json.loads(message.data)
        payload['observed_heading_coverage'] = None if self.people is None else round(
            len(self.people.headings)/self.people.count, 3)
        message.data = json.dumps(payload)
        self.status_pub.publish(message)
        self._log_people_state()

    def _log_people_state(self):
        """Log transitions and bounded progress, never every detector frame."""
        if not self.config['people']['enabled']:
            return
        labels = {'idle': '待機／冷卻', 'going_observation': '前往人流分析點',
                  'waiting_observation_stop': '已到分析點，等待停妥及連續安全雷達',
                  'observing': '人流自旋分析', 'waiting_observation_retry': '自旋已取消，停妥後等待安全續轉',
                  'people_destination_selected': '已選出人流待機點',
                  'going_people_standby': '前往選定待機點', 'people_arrival_check': '確認待機點抵達'}
        key = (self.state, self.reason)
        now = time.monotonic()
        transition = key != self.log_key
        progress = self.state in ('observing', 'waiting_observation_retry') and now-self.last_progress_log >= 5.
        if not transition and not progress:
            return
        self.log_key, self.last_progress_log = key, now
        point = self.goal or {}
        self.navigator.get_logger().info(
            '人流｜%s；原因=%s；目標=%s；進度=%.1f°；續轉=%d/%d；冷卻剩餘=%.0fs；安全=%s' %
            (labels.get(self.state, self.state), self.reason or '正常', point.get('name', '-'),
             math.degrees(self.spin_travel), self.spin_resumes, self.config['people']['max_spin_resumes'],
             max(0., self.next_people_at-now), json.dumps(self.guard_report, ensure_ascii=False)))

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
        values=(velocity.linear.x,velocity.linear.y,velocity.angular.z)
        return (all(math.isfinite(v) for v in values)
                and max(abs(values[0]),abs(values[1])) <= .02 and abs(values[2]) <= .03)

    def _safe(self):
        if self.home is not None:
            self.home.expire(time.monotonic())
            if not self.home.healthy or self.home.cutoff:
                return False
        return (self.map_matches and self.ready() and not self.inhibited
                and self.last_odom is not None
                and self._stamp_fresh(self.last_odom[0], self.last_odom[1].header.stamp)
                and self._pose() is not None)

    def _clear_to_spin(self, require_stationary=True):
        self.guard_report = self._spin_guard_report(require_stationary)
        return self.guard_report['clear']

    def _spin_guard_report(self, require_stationary=True):
        # Freshness/safety is rechecked even when geometry has a short cache.
        if (not self._safe() or (require_stationary and not self._stationary())):
            return {'clear':False,'reason':'unsafe_localization_or_odom'}
        if self.last_scan is None or not self._stamp_fresh(self.last_scan[0], self.last_scan[1].header.stamp):
            return {'clear':False,'reason':'scan_stale_or_absent'}
        if self.scan_evidence_error:
            return {'clear':False,'reason':'laser_tf_unavailable','detail':self.scan_evidence_error}
        if not self.scan_history:
            # No historical permission: retain original conservative single scan.
            return self._single_spin_guard_report(require_stationary)
        scan = self.last_scan[1]
        source = scan.header.stamp.sec+scan.header.stamp.nanosec/1e9
        if self.scan_history[-1].source_sec != source or self.scan_evidence_error:
            return {'clear':False,'reason':'laser_tf_unavailable','detail':self.scan_evidence_error}
        try:
            base = self.buffer.lookup_transform('odom','base_footprint',Time())
            if not self._stamp_fresh(time.monotonic(),base.header.stamp):
                return {'clear':False,'reason':'odom_tf_stale'}
            t = base.transform
            q = t.rotation
            if (not all(math.isfinite(v) for v in (q.x,q.y,q.z,q.w))
                    or abs(q.x)>1e-4 or abs(q.y)>1e-4
                    or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.01):
                return {'clear':False,'reason':'invalid_base_orientation'}
            pose = (t.translation.x,t.translation.y,yaw_of(t.rotation))
            now = time.monotonic()
            if self.spin_cache is not None:
                key,at,old_pose,report = self.spin_cache
                angle = abs(math.atan2(math.sin(pose[2]-old_pose[2]),math.cos(pose[2]-old_pose[2])))
                if (source==key and 0<=now-at<=.04
                        and self.navigator.get_clock().now().nanoseconds/1e9 <= report.get('evidence_deadline_sec',0.)
                        and math.hypot(pose[0]-old_pose[0],pose[1]-old_pose[1])<=.005 and angle<=.01):
                    return report
            report = aligned_spin_report(self.scan_history,pose,now,
                self.navigator.get_clock().now().nanoseconds/1e9,
                math.hypot(.22,.16)+self.config['people']['scan_clearance_m'],
                self.config['people']['max_unknown_arc_deg'])
            self.spin_cache=(source,now,pose,report)
            return report
        except (TransformException, ValueError, TypeError, AttributeError) as exc:
            return {'clear':False,'reason':'laser_tf_unavailable','detail':str(exc)}

    def _single_spin_guard_report(self, require_stationary=True):
        report = {'clear': False, 'reason': 'unsafe_localization_or_odom'}
        if (not self._safe() or (require_stationary and not self._stationary())
                or self.last_scan is None):
            return report
        receipt, scan = self.last_scan
        if not self._stamp_fresh(receipt, scan.header.stamp) or not scan.ranges:
            return dict(report, reason='scan_stale_or_absent')
        try:
            transform = self.buffer.lookup_transform('base_footprint', scan.header.frame_id, Time())
        except TransformException:
            return dict(report, reason='laser_tf_unavailable')
        q = transform.transform.rotation
        tx, ty = transform.transform.translation.x, transform.transform.translation.y
        if not all(math.isfinite(value) for value in (
                q.x, q.y, q.z, q.w, tx, ty, scan.angle_min, scan.angle_increment,
                scan.range_min, scan.range_max)):
            return dict(report, reason='invalid_laser_geometry')
        if abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1.) > .05:
            return dict(report, reason='invalid_laser_geometry')
        if not 6.0 <= abs(scan.angle_increment) * len(scan.ranges) <= 2 * math.pi + .1:
            return dict(report, reason='incomplete_scan_circle')
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
                return dict(report, reason='near_obstacle', obstacle_distance_m=round(
                    math.hypot(tx + xx * lx + xy * ly, ty + yx * lx + yy * ly), 3))
        longest, run = 0, 0
        for available in known + known:
            run = 0 if available else run + 1
            longest = max(longest, run)
        unknown_arc = min(longest, len(known)) * abs(scan.angle_increment) * 180 / math.pi
        clear = (valid / len(scan.ranges) >= 0.75
                 and unknown_arc <= self.config['people']['max_unknown_arc_deg'])
        return {'clear': clear, 'reason': 'clear' if clear else 'unknown_scan_coverage',
                'valid_fraction': round(valid/len(scan.ranges), 4), 'unknown_arc_deg': round(unknown_arc, 3)}

    def _spin_start_ready(self):
        """Require independent safe scans AND continuously stopped feedback."""
        now = time.monotonic()
        clear = self._clear_to_spin()
        if not self._stationary():
            self.safe_scan_count = 0
            self.safe_scan_stamp = self.safe_stop_since = None
            self.safe_scan_since = None
            return False
        self.safe_stop_since = self.safe_stop_since if self.safe_stop_since is not None else now
        if not clear:
            self.safe_scan_since = None
            self.safe_scan_count = 0
            self.safe_scan_stamp = None
            return False
        self.safe_scan_since = self.safe_scan_since if self.safe_scan_since is not None else now
        scan = self.last_scan[1]
        stamp = (scan.header.stamp.sec, scan.header.stamp.nanosec)
        if stamp != self.safe_scan_stamp:
            self.safe_scan_count += 1
            self.safe_scan_stamp = stamp
        return (self.safe_scan_count >= self.config['people']['safe_scan_samples']
                and now-self.safe_scan_since >= .75
                and now-self.safe_stop_since >= self.config['people']['stopped_before_spin_sec'])

    def _vision(self, message):
        if not self.observation_collecting or self.people is None:
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

    def _start(self, point, state, tree, timeout, resume=False):
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
        self.interrupt_reason = None
        if state=='observing':
            self.spin_resume_pending = resume
            self.spin_resume_anchor = self.spin_travel
            if resume:
                self.spin_resume_requests += 1
        if state == 'going_observation':
            self.spin_travel = 0.0
            self.spin_resumes = 0
            self.spin_resume_requests = 0
            self.people = None
            self.guard_report = {}
        if state == 'observing' and self.observation_session_deadline is not None:
            self.deadline = min(self.deadline, self.observation_session_deadline)
        self._log_people_state()
        return True

    def _people_finished(self, reason):
        successful = reason in ('parked_at_selected_standby', 'insufficient_directional_demand')
        preempted = reason.startswith('observation_preempted')
        self.people_failures = 0 if successful else self.people_failures if preempted else self.people_failures+1
        self.state, self.reason = 'idle', reason
        long_wait = successful or self.people_failures >= self.config['people']['max_consecutive_failures']
        self.next_people_at = time.monotonic() + self.config['people'][
            'cooldown_sec' if long_wait else 'failure_cooldown_sec']
        if long_wait:
            self.people_failures = 0
        self.idle_since = None
        self.idle_clock.reset()
        self.scan_history.clear()
        self.spin_cache = None
        self.observation_collecting = False
        self.spin_resume_pending = False
        self.observation_session_deadline = None
        self._remove_spin_tree()
        self._log_people_state()

    def _remove_spin_tree(self):
        if self.observation_tree_path is not None:
            try:
                Path(self.observation_tree_path).unlink(missing_ok=True)
            except OSError as exc:
                self.navigator.get_logger().warning('人流暫存 BT 清理失敗：%s' % exc)
            self.observation_tree_path = None

    def _resume_spin_tree(self):
        """A remaining-angle BT, never a fresh extra full revolution."""
        remaining = max(0., 2*math.pi-self.spin_travel)
        self._remove_spin_tree()
        with tempfile.NamedTemporaryFile(prefix='robot_people_remaining_', suffix='.xml',
                                         mode='w', encoding='utf-8', delete=False) as stream:
            stream.write('<root BTCPP_format="4" main_tree_to_execute="MainTree">'
                         '<BehaviorTree ID="MainTree"><Spin spin_dist="%.9f" time_allowance="30.0" '
                         'error_code_id="{spin_error}"/></BehaviorTree></root>' % remaining)
            self.observation_tree_path = stream.name
        return stream.name

    def _finish_people_scan(self):
        self.observation_collecting = False
        self._remove_spin_tree()
        selected = self.people.select(self.config['people']['observation_pose']) if self.people else None
        if selected is None:
            self._people_finished('insufficient_directional_demand')
        else:
            self.goal, self.state = selected, 'people_destination_selected'
            self.reason = 'full_scan_completed'
            self._log_people_state()

    def poll(self, has_work):
        """Cancel on unsafe data/new service work and drain before any new goal."""
        if has_work:
            self.idle_since = None
            self.idle_clock.reset()
        if (not self.motion and self.state in (
                'waiting_observation_stop', 'waiting_observation_retry',
                'people_destination_selected', 'people_arrival_check')
                and (has_work or self.stop_accepting)):
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
        reason = None
        if self.state != 'going_home' and (has_work or self.stop_accepting):
            reason = 'observation_preempted_by_service_or_battery'
        elif not self._safe():
            reason = 'unsafe_localization_or_odom_or_battery'
        elif time.monotonic() >= self.deadline:
            reason = 'observation_or_navigation_timeout'
        elif self.state == 'observing' and not self._clear_to_spin(False):
            reason = self.guard_report.get('reason', 'spin_safety_rejected')
        if reason and not self.cancel_requested:
            self.cancel_requested = True
            self.cancel_started = time.monotonic()
            self.interrupt_reason = reason
            self.reason = reason
            self._log_people_state()
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
            elif (self.state == 'observing' and self.interrupt_reason == 'unknown_scan_coverage'
                  and self.spin_resumes < self.config['people']['max_spin_resumes']
                  and self.spin_resume_requests < 4
                  and self.observation_session_deadline is not None
                  and time.monotonic() < self.observation_session_deadline):
                self.state, self.reason = 'waiting_observation_retry', self.interrupt_reason
                self.safe_scan_count = 0
                self.safe_scan_stamp = self.safe_stop_since = None
                self.safe_scan_since = None
                self.spin_resume_pending = False
                self.deadline = min(time.monotonic()+self.config['people']['spin_ready_wait_sec'],
                                    self.observation_session_deadline)
                self._log_people_state()
            else:
                self._people_finished(self.interrupt_reason or 'nav2_failed')
        elif self.state == 'going_home':
            self.state, self.stop_since = 'verifying_home', None
            self.deadline = time.monotonic() + 5.0
        elif self.state == 'going_observation':
            self.state = 'waiting_observation_stop'
            self.deadline = time.monotonic() + self.config['people']['spin_ready_wait_sec']
            self.safe_scan_count = 0
            self.safe_scan_stamp = self.safe_stop_since = None
            self.safe_scan_since = None
        elif self.state == 'observing':
            self.settling_after_cancel = True
            self.stop_since = None
            if abs(self.spin_travel) < 5.8:
                self._people_finished('insufficient_rotation_coverage')
            else:
                self._finish_people_scan()
        elif self.state == 'going_people_standby':
            self.state = 'people_arrival_check'
            self.deadline = time.monotonic() + 5.0
        return self.settling_after_cancel

    def _at_goal(self, point, xy, yaw):
        pose = self._pose()
        return (pose is not None and math.hypot(pose['x'] - point['x'], pose['y'] - point['y']) <= xy
                and abs(math.atan2(math.sin(pose['yaw'] - point['yaw']),
                                   math.cos(pose['yaw'] - point['yaw']))) <= yaw)

    def idle(self, parked=False):
        """Return True when optional policy owns idle flow, even while stopped."""
        now = time.monotonic()
        if self.settling_after_cancel:
            return True
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
                if self.home.cutoff:
                    self.reason = 'battery_cutoff_safe_stop'
                    return True
                if not self._safe() or not self._stationary():
                    self.stop_since = None
                    return True
                self.stop_since = self.stop_since if self.stop_since is not None else now
                if now-self.stop_since >= self.config['home']['stopped_duration_sec']:
                    self.stop_since = None
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
        if self.state in ('waiting_observation_stop', 'waiting_observation_retry'):
            if self.settling_after_cancel:
                return True
            retry = self.state == 'waiting_observation_retry'
            if (retry and (not self._safe() or self.observation_session_deadline is None
                          or now >= self.observation_session_deadline)):
                self._people_finished('retry_localization_or_session_unavailable')
                return True
            if now >= self.deadline:
                self._people_finished('spin_ready_timeout:'+self.guard_report.get('reason','unknown'))
                return True
            pose = self._pose()
            point = people['observation_pose']
            if pose is None or math.hypot(pose['x']-point['x'],pose['y']-point['y']) > .10:
                if now >= self.deadline:
                    self._people_finished('observation_position_not_verified')
                return True
            if retry and self.spin_travel >= 5.8 and self._stationary():
                self._finish_people_scan()
                return True
            if self._spin_start_ready():
                if retry:
                    tree = self._resume_spin_tree()
                    self._start(point, 'observing', tree, people['observation_timeout_sec'],resume=True)
                    return True
                self.people = DirectionalPeople(people)
                self.spin_travel = 0.0
                self.spin_resumes = 0
                self.observation_session_deadline = now+people['observation_session_timeout_sec']
                self.observation_collecting = True
                self._start(people['observation_pose'], 'observing',
                            str(self.share / 'behavior_trees/people_observe.xml'),
                            people['observation_timeout_sec'])
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
        pose = self._pose()
        odom = None if self.last_odom is None else self.last_odom[1].pose.pose
        odom_pose = None if odom is None else (odom.position.x,odom.position.y,yaw_of(odom.orientation))
        if not self.idle_clock.update(now,parked=parked,safe=self._safe(),
                                     stationary=self._stationary(),pose=pose,odom=odom_pose):
            if self.idle_clock.elapsed == 0.:
                self.idle_since = None
            return False
        self.idle_since = self.idle_since if self.idle_since is not None else now
        if now < self.next_people_at or self.idle_clock.elapsed < people['idle_before_observation_sec']:
            return False
        self._start(people['observation_pose'], 'going_observation', '', people['navigation_timeout_sec'])
        return True
