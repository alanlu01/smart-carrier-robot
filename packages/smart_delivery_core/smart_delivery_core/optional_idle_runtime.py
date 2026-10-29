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
from tf2_ros import ExtrapolationException, TransformException

from smart_delivery_core.optional_idle import (
    DirectionalPeople, HomeVoltageLatch, map_alignment_correction, occupancy_fingerprint,
)
from smart_delivery_core.standby_idle_clock import StandbyIdleClock
from smart_delivery_core.spin_evidence import ScanEvidence, aligned_spin_report
from smart_delivery_core.standby_retry import StandbyRetryPolicy
from smart_delivery_core.delivery_console import people_status, safety_status, reason_label
from smart_delivery_core.pipeline_health import DurationMetrics, timed_operation

STATE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)

# Pair only the latest scan, without blocking the executor that receives TF.
# The original 0.30 s source/receipt freshness budget still applies.
SPIN_TF_PAIR_WAIT_SEC = 0.20
RETRYABLE_SPIN_STOPS = frozenset({
    'unknown_scan_coverage', 'near_obstacle', 'recent_near_obstacle',
    'laser_tf_pending', 'laser_tf_wait_timeout',
})
DATA_SPIN_STOPS = frozenset({'scan_stale_or_absent','odom_tf_stale','laser_tf_unavailable',
                             'laser_tf_pending','laser_tf_wait_timeout'})


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
        self.scan_evidence_reason = ''
        self.pending_scan = None
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
        self.spin_ready_blocker = 'not_waiting'
        self.spin_resumes = 0
        self.spin_resume_requests = 0
        self.spin_resume_pending = False
        self.spin_resume_anchor = 0.
        self.people_failures = 0
        self.analysis_completed = False
        self.analysis_completed_at = None
        self.selected_standby = None
        self.parking_retry = StandbyRetryPolicy(retry_delay=2.)
        self.parking_attempts = 0
        self.parking_outcome = 'not_started'
        self.parking_last_failure = ''
        self.parking_deadline = 0.
        self.parking_stop_since = None
        self.observation_collecting = False
        self.observation_session_deadline = None
        self.observation_tree_path = None
        self.interrupt_reason = None
        self.log_key = None
        self.last_progress_log = 0.0
        self.localization_status = None
        self.localization_pauses = 0
        self.localization_restarts = 0
        self.localization_resume_state = None
        self.localization_resume_deadline = None
        self.localization_wait_deadline = None
        self.observation_alignment = None
        self.safety_report = {}
        self.duration_metrics = DurationMetrics()
        self.last_unsafe_report = None
        self.pose_report = {}
        self.safety_log_key = None
        self.safety_log_at = -float('inf')
        self.parking_recovery_deadline = None
        self.parking_recovery_pauses = 0
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
            navigator.create_subscription(String, '/localization/state', self._localization, STATE_QOS)
        navigator.create_timer(0.5, self._heartbeat)
        if config['people']['enabled']:
            navigator.create_timer(0.02, self._retry_scan_transform)

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

    def _localization(self, message):
        """Observe phase only; never publish localization readiness ourselves."""
        try:
            payload = json.loads(message.data)
            stamp = payload['sample_ros_stamp_ns']
            age = (self.navigator.get_clock().now().nanoseconds-stamp)/1e9
            if (not isinstance(stamp, int) or isinstance(stamp, bool) or stamp <= 0
                    or not -.1 <= age <= 2. or not isinstance(payload.get('state'), str)):
                return
            if self.localization_status and stamp <= self.localization_status[1]:
                return
            self.localization_status = (time.monotonic(), stamp, payload['state'])
        except (ValueError, KeyError, TypeError, OverflowError):
            return

    def _localization_phase(self):
        if self.localization_status is None:
            return None
        at, stamp, phase = self.localization_status
        age = (self.navigator.get_clock().now().nanoseconds-stamp)/1e9
        return phase if 0 <= time.monotonic()-at <= 2. and -.1 <= age <= 2. else None

    def _map_alignment(self):
        """Compare map-to-odom correction, not physical tail rotation while stopping."""
        try:
            tr = self.buffer.lookup_transform('map', 'odom', Time()).transform
            value = (tr.translation.x, tr.translation.y, yaw_of(tr.rotation))
            return value if all(math.isfinite(v) for v in value) else None
        except TransformException:
            return None

    def _can_wait_localization(self):
        """Only brief SUSPECT can wait; stale sensors, manual/global recovery cannot."""
        return (not self.ready() and self._localization_phase() in ('SUSPECT', 'LOCALIZED')
                and self.map_matches and not self.inhibited
                and (self.home is None or (self.home.healthy and not self.home.cutoff))
                and self.last_odom is not None
                and self._stamp_fresh(self.last_odom[0], self.last_odom[1].header.stamp)
                and self.last_scan is not None
                and self._stamp_fresh(self.last_scan[0], self.last_scan[1].header.stamp)
                and self._pose() is not None)

    def _can_pause_for_data(self):
        """Stop on jitter first; fresh evidence is required only to RESUME."""
        return (self._localization_phase() in ('SUSPECT','LOCALIZED')
                and self.map_matches and not self.inhibited and not self.stop_accepting
                and (self.home is None or (self.home.healthy and not self.home.cutoff)))

    def _begin_localization_wait(self, resume_state, deadline):
        self.localization_pauses += 1
        self.localization_resume_state = resume_state
        self.localization_resume_deadline = deadline
        limit = time.monotonic()+self.config['people'].get('localization_wait_sec', 20.)
        self.localization_wait_deadline = min(limit, deadline)
        if self.observation_session_deadline is not None:
            self.localization_wait_deadline = min(self.localization_wait_deadline,
                                                  self.observation_session_deadline)

    def _poll_localization_wait(self):
        """Own a bounded safe stop even while Delivery waits for readiness."""
        now = time.monotonic()
        if now >= self.localization_wait_deadline:
            self._people_finished('people_localization_wait_timeout')
            return True
        phase = self._localization_phase()
        if phase not in (None, 'SUSPECT', 'LOCALIZED'):
            self._people_finished('people_localization_escalated:'+phase)
            return True
        if (phase != 'LOCALIZED' or not self._safe() or not self._stationary()
                or self.last_scan is None
                or not self._stamp_fresh(self.last_scan[0], self.last_scan[1].header.stamp)):
            return True
        target = self.localization_resume_state
        changed = False
        if target == 'going_observation':
            self._start(self.config['people']['observation_pose'], target, '',
                        self.localization_resume_deadline-now, resume=True)
            return True
        if target == 'observing':
            alignment = self._map_alignment()
            if alignment is None or self.observation_alignment is None:
                self._people_finished('people_localization_alignment_unavailable')
                return True
            old = self.observation_alignment
            position = self.last_odom[1].pose.pose.position
            try:
                distance, angle = map_alignment_correction(old, alignment, position.x, position.y)
            except ValueError:
                self._people_finished('people_localization_alignment_invalid')
                return True
            changed = distance > .10 or angle > .10
            if changed:
                if self.localization_restarts >= 1:
                    self._people_finished('people_localization_repeated_alignment_change')
                    return True
                self.people = DirectionalPeople(self.config['people'])
                self.spin_travel = 0.
                self.localization_restarts += 1
                self.observation_alignment = alignment
            self.state = 'waiting_observation_retry'
        else:
            self.state = 'waiting_observation_stop'
        self.reason = 'localization_recovered_recollecting' if target == 'observing' and changed else 'localization_recovered'
        self.scan_history.clear()
        self.pending_scan = None
        self.scan_evidence_error = self.scan_evidence_reason = ''
        self.spin_cache = None
        self._reset_spin_ready()
        self.deadline = now+self.config['people']['spin_ready_wait_sec']
        if self.observation_session_deadline is not None:
            self.deadline = min(self.deadline, self.observation_session_deadline)
        self._log_people_state()
        return True

    def _scan(self, message):
        now = time.monotonic()
        self.last_scan = (now, message)
        self.spin_cache = None
        # New evidence always supersedes an older pending scan. Never retain an
        # old clear scan as permission while the latest one lacks its TF.
        self.pending_scan = None
        if self.state not in ('waiting_observation_stop', 'waiting_observation_retry', 'observing'):
            return
        self._transform_scan(message, now)

    def _transform_scan(self, message, received_at):
        if not self._stamp_fresh(received_at, message.header.stamp):
            self.pending_scan = None
            self.scan_evidence_reason = 'scan_stale_or_absent'
            self.scan_evidence_error = 'Scan expired before its timestamped TF was available'
            return
        stamp = Time(nanoseconds=message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec)
        try:
            sensor = self.buffer.lookup_transform('odom',message.header.frame_id,stamp).transform
            base = self.buffer.lookup_transform('odom','base_footprint',stamp).transform
            for q in (sensor.rotation,base.rotation):
                if (abs(q.x)>1e-4 or abs(q.y)>1e-4
                        or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.01):
                    raise ValueError('unsupported laser/base tilt or invalid quaternion')
            evidence = ScanEvidence.from_scan(message, received_at,
                (sensor.translation.x,sensor.translation.y,yaw_of(sensor.rotation)),
                (base.translation.x,base.translation.y,yaw_of(base.rotation)))
            self.scan_history.append(evidence)
            self.scan_evidence_error = ''
            self.scan_evidence_reason = ''
            self.pending_scan = None
            self.spin_cache = None
        except ExtrapolationException as exc:
            self.scan_evidence_error = str(exc)
            if 'extrapolation into the future' in str(exc).lower():
                if time.monotonic()-received_at < SPIN_TF_PAIR_WAIT_SEC:
                    self.pending_scan = (received_at, message)
                    self.scan_evidence_reason = 'laser_tf_pending'
                else:
                    self.pending_scan = None
                    self.scan_evidence_reason = 'laser_tf_wait_timeout'
            else:
                self.pending_scan = None
                self.scan_evidence_reason = 'laser_tf_unavailable'
        except (TransformException, ValueError, TypeError, AttributeError) as exc:
            self.pending_scan = None
            self.scan_evidence_error = str(exc)
            self.scan_evidence_reason = 'laser_tf_unavailable'

    def _retry_scan_transform(self):
        """Retry a future-TF pairing briefly, never restamp or extrapolate."""
        if self.pending_scan is None:
            return
        if self.state not in ('waiting_observation_stop', 'waiting_observation_retry', 'observing'):
            self.pending_scan = None
            return
        received_at, message = self.pending_scan
        if time.monotonic()-received_at >= SPIN_TF_PAIR_WAIT_SEC:
            self.pending_scan = None
            self.scan_evidence_reason = 'laser_tf_wait_timeout'
            self.scan_evidence_error = 'Timed out waiting for the scan timestamp TF'
            return
        self._transform_scan(message, received_at)

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
            self.navigator.get_logger().warning('🔋 '+('低電量返航已鎖定' if self.home.latched
                else '低電量，暫停接單' if self.home.admission_latched else '電池狀態已恢復'))

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
                                   'analysis_completed': self.analysis_completed,
                                   'selected_standby': self.selected_standby,
                                   'parking_outcome': self.parking_outcome,
                                   'parking_attempts': self.parking_attempts,
                                   'parking_max_attempts': self.parking_retry.max_attempts,
                                   'parking_last_failure': self.parking_last_failure,
                                   'parking_retry_in_sec': (max(0.,self.parking_retry.retry_at-time.monotonic())
                                       if self.state=='people_parking_retry_wait' else None),
                                   'spin_progress_deg': round(math.degrees(self.spin_travel), 1),
                                   'spin_resume_count': self.spin_resumes,
                                   'spin_resume_requests': self.spin_resume_requests,
                                   'spin_resume_pending': self.spin_resume_pending,
                                   'continuous_safe_scan_sec': (0. if self.safe_scan_since is None
                                       or self.state not in ('waiting_observation_stop', 'waiting_observation_retry')
                                       else max(0.,time.monotonic()-self.safe_scan_since)),
                                   'spin_ready_blocker': self.spin_ready_blocker,
                                   'pending_scan_age_sec': (None if self.pending_scan is None else
                                       round(max(0.,time.monotonic()-self.pending_scan[0]),3)),
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
        payload['people_demand'] = (self.people.summary(self.config['people']['observation_pose'])
            if self.people is not None and hasattr(self.people, 'summary') else None)
        payload['localization_pauses'] = self.localization_pauses
        payload['localization_restarts'] = self.localization_restarts
        payload['localization_wait_remaining_sec'] = (None if self.state != 'waiting_people_localization'
            else round(max(0., self.localization_wait_deadline-time.monotonic()), 2))
        payload['safety_check'] = self.safety_report
        payload['wall_durations'] = self.duration_metrics.snapshot(time.monotonic())
        payload['last_unsafe_check'] = self.last_unsafe_report
        payload['parking_recovery_pauses'] = self.parking_recovery_pauses
        payload['parking_recovery_remaining_sec'] = (None if self.parking_recovery_deadline is None
            else round(max(0.,self.parking_recovery_deadline-time.monotonic()),2))
        message.data = json.dumps(payload)
        self.status_pub.publish(message)
        self._log_people_state()

    def _log_people_state(self):
        """Log transitions and bounded progress, never every detector frame."""
        if not self.config['people']['enabled']:
            return
        key = (self.state, self.reason, self.parking_attempts, self.parking_outcome)
        now = time.monotonic()
        transition = key != self.log_key
        progress = self.state in ('observing', 'waiting_observation_stop', 'waiting_observation_retry',
            'waiting_people_localization','waiting_people_parking_recovery') and now-self.last_progress_log >= 5.
        idle_progress = (self.state=='idle' and self.idle_clock.point is not None
            and self.idle_clock.reason=='counting_verified_stationary_parking'
            and self.ready() and self._localization_phase() in (None,'LOCALIZED')
            and not self.inhibited and now-self.last_progress_log>=30.)
        if not transition and not progress and not idle_progress:
            return
        self.log_key, self.last_progress_log = key, now
        point = self.goal or {}
        text = ('🛑 人流動作停止中：'+reason_label(self.reason) if self.cancel_requested and self.motion
                else people_status(self.state,self.reason,point,
                    math.degrees(self.spin_travel),self.parking_attempts,
                    self.spin_ready_blocker if self.spin_ready_blocker!='not_waiting' else ''))
        if idle_progress:
            cooldown=max(0.,self.next_people_at-now)
            remaining=max(0.,self.config['people']['idle_before_observation_sec']-self.idle_clock.elapsed)
            text='💤 已停妥：'+str(self.idle_clock.point['name'])+'｜'+(
                f'人流冷卻 {cooldown:.0f} 秒' if cooldown>0 else f'人流待機剩餘 {remaining:.0f} 秒')
        self.navigator.get_logger().info(text)

    def _stamp_fresh(self, receipt, stamp, limit=0.30):
        now = self.navigator.get_clock().now().nanoseconds
        source = stamp.sec * 1_000_000_000 + stamp.nanosec
        return (0 <= time.monotonic() - receipt <= limit and source > 0
                and -0.1 <= (now - source) / 1e9 <= limit)

    @timed_operation('people_pose_lookup')
    def _pose(self, stamp=None):
        try:
            transform = self.buffer.lookup_transform('map', 'base_footprint', stamp or Time())
        except TransformException as exc:
            self.pose_report = {'valid':False,'reason':'tf_lookup_failed','detail':str(exc)[:500]}
            return None
        now = self.navigator.get_clock().now().nanoseconds
        source = transform.header.stamp.sec * 1_000_000_000 + transform.header.stamp.nanosec
        age = (now-source)/1e9
        self.pose_report = {'valid':False,'reason':'tf_stale','source_age_sec':round(age,4)}
        if not -0.1 <= age <= 0.30:
            return None
        pose = {'x': transform.transform.translation.x, 'y': transform.transform.translation.y,
                'yaw': yaw_of(transform.transform.rotation)}
        valid = all(math.isfinite(value) for value in pose.values())
        self.pose_report.update(valid=valid,reason='healthy' if valid else 'tf_nonfinite')
        return pose if valid else None

    def _stationary(self):
        if not self.last_odom or not self._stamp_fresh(self.last_odom[0], self.last_odom[1].header.stamp):
            return False
        velocity = self.last_odom[1].twist.twist
        values=(velocity.linear.x,velocity.linear.y,velocity.angular.z)
        return (all(math.isfinite(v) for v in values)
                and max(abs(values[0]),abs(values[1])) <= .02 and abs(values[2]) <= .03)

    def _safe(self):
        battery_ok = True
        if self.home is not None:
            self.home.expire(time.monotonic())
            battery_ok = self.home.healthy and not self.home.cutoff
        ready = bool(self.ready())
        odom_ok = (self.last_odom is not None
                   and self._stamp_fresh(self.last_odom[0], self.last_odom[1].header.stamp))
        pose_ok = self._pose() is not None
        checks = {'map_matches':bool(self.map_matches),'delivery_ready':ready,
                  'motion_allowed':not self.inhibited,'battery_healthy':bool(battery_ok),
                  'odom_fresh':bool(odom_ok),'map_pose_valid':pose_ok}
        failed = [key for key,value in checks.items() if not value]
        now = time.monotonic()
        self.safety_report = {'sample_ros_stamp_ns':self.navigator.get_clock().now().nanoseconds,
            'safe':not failed,'failed_checks':failed,'checks':checks,
            'localization_phase':self._localization_phase(),'tf':dict(self.pose_report),
            'odom_receive_age_sec':None if self.last_odom is None else round(now-self.last_odom[0],4),
            'odom_source_age_sec':None if self.last_odom is None else round(
                (self.navigator.get_clock().now().nanoseconds-(self.last_odom[1].header.stamp.sec*1_000_000_000
                 +self.last_odom[1].header.stamp.nanosec))/1e9,4)}
        if failed:
            self.last_unsafe_report = dict(self.safety_report)
            key = tuple(failed)
            if now-self.safety_log_at >= 5.:
                message = safety_status(failed,self.state,self.motion)
                logger = self.navigator.get_logger()
                # rclpy caches severity by physical call site. Keep distinct lines.
                if self.motion:
                    logger.warning(message)
                else:
                    logger.info(message)
                self.safety_console_waiting = True
                self.safety_log_key,self.safety_log_at = key,now
        else:
            self.safety_log_key = None
            if getattr(self,'safety_console_waiting',False):
                self.navigator.get_logger().info('✅ 定位／感測已恢復')
                self.safety_console_waiting = False
        return not failed

    def _clear_to_spin(self, require_stationary=True):
        self.guard_report = self._spin_guard_report(require_stationary)
        return self.guard_report['clear']

    @timed_operation('people_spin_guard')
    def _spin_guard_report(self, require_stationary=True):
        # Freshness/safety is rechecked even when geometry has a short cache.
        if (not self._safe() or (require_stationary and not self._stationary())):
            return {'clear':False,'reason':'unsafe_localization_or_odom'}
        if self.last_scan is None or not self._stamp_fresh(self.last_scan[0], self.last_scan[1].header.stamp):
            return {'clear':False,'reason':'scan_stale_or_absent'}
        if self.scan_evidence_error:
            return {'clear':False,'reason':self.scan_evidence_reason or 'laser_tf_unavailable',
                    'detail':self.scan_evidence_error}
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

    def _reset_spin_ready(self):
        self.safe_scan_count = 0
        self.safe_scan_stamp = self.safe_stop_since = self.safe_scan_since = None
        self.spin_ready_blocker = 'not_waiting'

    def _spin_start_ready(self):
        """Require independent safe scans AND continuously stopped feedback."""
        now = time.monotonic()
        clear = self._clear_to_spin()
        if not self._stationary():
            self.safe_scan_count = 0
            self.safe_scan_stamp = self.safe_stop_since = None
            self.safe_scan_since = None
            self.spin_ready_blocker = 'vehicle_not_stopped'
            return False
        self.safe_stop_since = self.safe_stop_since if self.safe_stop_since is not None else now
        if not clear:
            self.safe_scan_since = None
            self.safe_scan_count = 0
            self.safe_scan_stamp = None
            self.spin_ready_blocker = self.guard_report.get('reason', 'scan_not_clear')
            return False
        self.safe_scan_since = self.safe_scan_since if self.safe_scan_since is not None else now
        scan = self.last_scan[1]
        stamp = (scan.header.stamp.sec, scan.header.stamp.nanosec)
        if stamp != self.safe_scan_stamp:
            self.safe_scan_count += 1
            self.safe_scan_stamp = stamp
        if self.safe_scan_count < self.config['people']['safe_scan_samples']:
            self.spin_ready_blocker = 'insufficient_independent_scans'
        elif now-self.safe_stop_since < self.config['people']['stopped_before_spin_sec']:
            self.spin_ready_blocker = 'stopped_duration_insufficient'
        elif now-self.safe_scan_since < .75:
            self.spin_ready_blocker = 'continuous_safe_duration_insufficient'
        else:
            self.spin_ready_blocker = 'ready'
            return True
        return False

    def _vision(self, message):
        if (not self.observation_collecting or self.people is None
                or self.state != 'observing' or self.cancel_requested or not self._safe()):
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
            self._reset_spin_ready()
            self.spin_resume_pending = resume
            self.spin_resume_anchor = self.spin_travel
            if resume:
                self.spin_resume_requests += 1
        if state == 'going_observation' and not resume:
            self.analysis_completed, self.analysis_completed_at = False, None
            self.selected_standby = None
            self.parking_retry.reset()
            self.parking_attempts, self.parking_outcome = 0, 'not_started'
            self.parking_last_failure, self.parking_stop_since = '', None
            self.parking_recovery_pauses = 0
            self.parking_recovery_deadline = None
            self.spin_travel = 0.0
            self.spin_resumes = 0
            self.spin_resume_requests = 0
            self.people = None
            self.guard_report = {}
            self.localization_pauses = self.localization_restarts = 0
            self.localization_wait_deadline = self.localization_resume_deadline = None
            self.observation_alignment = None
        if state == 'observing' and self.observation_session_deadline is not None:
            self.deadline = min(self.deadline, self.observation_session_deadline)
        self._log_people_state()
        return True

    def _people_finished(self, reason, terminal_state='idle'):
        self.parking_recovery_deadline = None
        successful = self.analysis_completed or reason in (
            'parked_at_selected_standby', 'insufficient_directional_demand')
        preempted = reason.startswith('observation_preempted')
        if reason == 'parked_at_selected_standby':
            self.parking_outcome = 'parked'
        elif reason == 'insufficient_directional_demand':
            self.parking_outcome = 'not_needed'
        elif self.analysis_completed and preempted:
            self.parking_outcome = 'preempted'
        elif self.analysis_completed and self.parking_outcome not in ('parked','not_needed','failed'):
            self.parking_outcome = 'interrupted'
        self.parking_stop_since = None
        self.people_failures = 0 if successful else self.people_failures if preempted else self.people_failures+1
        self.state, self.reason = terminal_state, reason
        long_wait = successful or self.people_failures >= self.config['people']['max_consecutive_failures']
        if self.analysis_completed and self.analysis_completed_at is not None:
            # A later parking failure/preemption must not turn a completed
            # scan into a 30-second failure cycle, nor keep extending its age.
            self.next_people_at = max(self.next_people_at,
                self.analysis_completed_at+self.config['people']['cooldown_sec'])
        else:
            self.next_people_at = time.monotonic() + self.config['people'][
                'cooldown_sec' if long_wait else 'failure_cooldown_sec']
        if long_wait:
            self.people_failures = 0
        self.idle_since = None
        self.idle_clock.reset()
        self.scan_history.clear()
        self.pending_scan = None
        self.scan_evidence_error = self.scan_evidence_reason = ''
        self.spin_cache = None
        self._reset_spin_ready()
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
                self.navigator.get_logger().warning('⚠️ 人流暫存 BT 清理失敗：%s' % exc)
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
        self.analysis_completed, self.analysis_completed_at = True, time.monotonic()
        self.people_failures = 0
        self.next_people_at = self.analysis_completed_at+self.config['people']['cooldown_sec']
        self._reset_spin_ready()
        self._remove_spin_tree()
        selected = self.people.select(self.config['people']['observation_pose']) if self.people else None
        # Complete scores/counters stay in optional_idle_state.people_demand;
        # the console transition below shows only the selected point/outcome.
        if selected is None:
            self._people_finished('insufficient_directional_demand')
        else:
            self.selected_standby = dict(selected)
            self.goal, self.state = dict(selected), 'people_destination_selected'
            self.parking_retry.select_goal(selected)
            self.parking_attempts, self.parking_outcome = 0, 'selected'
            self.parking_stop_since = None
            # First route retains its normal timeout; at most two 30s
            # corrections, verification/backoff and stop checks fit in +90s.
            self.parking_deadline = time.monotonic()+self.config['people']['navigation_timeout_sec']+90.
            self.reason = 'full_scan_completed'
            self._log_people_state()

    def _parking_failed(self, reason):
        """Retry only the selected parking pose, never redo a completed scan."""
        self.parking_last_failure = reason
        self.parking_stop_since = None
        self.parking_retry.failed(time.monotonic())
        if (self.parking_retry.failures >= self.parking_retry.max_attempts
                or time.monotonic() >= self.parking_deadline):
            self.parking_outcome = 'failed'
            self._people_finished('selected_standby_parking_failed:'+reason,
                                  terminal_state='people_parking_blocked')
            return
        self.state, self.reason = 'people_parking_retry_wait', reason
        self.parking_outcome = 'retry_wait'
        self._log_people_state()

    def _parking_recovery_allowed(self):
        """Retain only a completed map-bound destination, never old spin evidence."""
        return (self.analysis_completed and self.selected_standby is not None
                and self.selected_standby in self.config['people']['standby_points']
                and self.map_matches and not self.inhibited and not self.stop_accepting
                and (self.home is None or (self.home.healthy and not self.home.cutoff))
                and self._localization_phase() not in ('MANUAL_REQUIRED','UNINITIALIZED','SEEDING'))

    def _begin_parking_recovery(self, reason):
        now = time.monotonic()
        if (not self._parking_recovery_allowed() or self.parking_recovery_pauses >= 2
                or now >= self.parking_deadline):
            self._people_finished('selected_standby_recovery_unavailable:'+reason)
            return False
        self.parking_recovery_pauses += 1
        self.parking_recovery_deadline = min(self.parking_deadline,now+90.)
        self.state,self.reason = 'waiting_people_parking_recovery',reason
        self.parking_outcome,self.parking_stop_since = 'waiting_recovery',None
        self.observation_collecting = False
        self._log_people_state()
        return True

    def _poll_parking_recovery(self):
        now = time.monotonic()
        if not self._parking_recovery_allowed():
            self._people_finished('selected_standby_recovery_not_permitted')
            return True
        if now >= self.parking_recovery_deadline:
            self._people_finished('selected_standby_recovery_timeout')
            return True
        if (self.settling_after_cancel or self._localization_phase() != 'LOCALIZED'
                or not self._safe() or not self._stationary()
                or self.last_scan is None
                or not self._stamp_fresh(self.last_scan[0],self.last_scan[1].header.stamp)):
            self.parking_stop_since = None
            return True
        # Independent stop proof after recovery, then a new goal in the SAME
        # verified map. Never resume the canceled action or redo the crowd scan.
        self.parking_stop_since = self.parking_stop_since if self.parking_stop_since is not None else now
        if now-self.parking_stop_since < 1.:
            return True
        self.state,self.reason = 'people_parking_retry_wait','selected_standby_recovery_ready'
        self.parking_outcome = 'retry_wait'
        self.parking_recovery_deadline = None
        self.parking_retry.retry_at = max(self.parking_retry.retry_at,now)
        self._log_people_state()
        return True

    @timed_operation('optional_poll')
    def poll(self, has_work):
        """Cancel on unsafe data/new service work and drain before any new goal."""
        if has_work:
            self.idle_since = None
            self.idle_clock.reset()
        if (not self.motion and self.state in (
                'waiting_observation_stop', 'waiting_observation_retry',
                'waiting_people_localization',
                'people_destination_selected', 'people_arrival_check', 'people_parking_retry_wait',
                'people_parking_blocked','waiting_people_parking_recovery')
                and (has_work or self.stop_accepting)):
            # A service can start between auxiliary actions. Never resume an
            # old observation/parking decision from a different position.
            self._people_finished('observation_preempted_between_actions')
        if not self.motion and self.state == 'waiting_people_parking_recovery':
            # Cancellation/stop drainage below must still complete before any
            # new motion, but the wait deadline must continue to expire.
            if (time.monotonic() >= self.parking_recovery_deadline
                    or not self._parking_recovery_allowed()):
                self._poll_parking_recovery()
            elif not self.settling_after_cancel:
                return self._poll_parking_recovery()
        if not self.motion and self.state == 'waiting_people_localization':
            phase = self._localization_phase()
            if (time.monotonic() >= self.localization_wait_deadline
                    or phase not in (None, 'SUSPECT', 'LOCALIZED')):
                # Expiration/escalation is not paused by an external recovery
                # spin. Still keep the independent stop proof before service.
                reason = ('people_localization_wait_timeout' if time.monotonic() >= self.localization_wait_deadline
                          else 'people_localization_escalated:'+phase)
                self._people_finished(reason)
                return True
            if self.settling_after_cancel:
                pass  # Keep the existing stop/drain proof below before resuming.
            else:
                return self._poll_localization_wait()
        if (not self.motion and self.state in ('waiting_observation_stop', 'waiting_observation_retry')
                and self.localization_pauses < 2 and self._can_wait_localization()):
            self._begin_localization_wait('observing' if self.people else 'waiting_observation_stop',
                self.observation_session_deadline or time.monotonic()+self.config['people'].get('localization_wait_sec', 20.))
            self.state, self.reason = 'waiting_people_localization', 'localization_suspect'
            return True
        if (not self.motion and self.state in ('waiting_observation_stop', 'waiting_observation_retry')
                and not self._safe()):
            phase = self._localization_phase()
            hard_failure = (phase not in (None,'SUSPECT','LOCALIZED')
                or not self.map_matches or self.inhibited or self.stop_accepting
                or (self.home is not None and (not self.home.healthy or self.home.cutoff)))
            if hard_failure:
                # Never preserve directional evidence through global/manual recovery.
                self._people_finished('observation_wait_unsafe')
                self.settling_after_cancel, self.stop_since = True, None
            elif time.monotonic() >= self.deadline:
                self._people_finished('spin_ready_timeout:unsafe_data')
            else:
                # No motion here: use the ORIGINAL 10s deadline for transient
                # TF/receipt jitter; do not lengthen source freshness budgets.
                self._reset_spin_ready()
                self.spin_ready_blocker = 'waiting_fresh_safety_data'
            return True
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
            if (self.state in ('going_observation', 'observing') and self.localization_pauses < 2
                    and self._can_wait_localization()):
                reason = 'localization_suspect'
            elif (self.state in ('going_observation','observing') and self.localization_pauses < 2
                    and self._can_pause_for_data()):
                reason = 'people_data_pause'
            elif self.state == 'going_people_standby' and self._parking_recovery_allowed():
                reason = 'selected_standby_safety_pause'
            else:
                reason = 'unsafe_localization_or_odom_or_battery'
        elif time.monotonic() >= self.deadline:
            reason = 'observation_or_navigation_timeout'
        elif self.state == 'observing' and not self._clear_to_spin(False):
            reason = self.guard_report.get('reason', 'spin_safety_rejected')
            if (reason in DATA_SPIN_STOPS and self.localization_pauses < 2
                    and self._can_pause_for_data()):
                reason = 'people_data_pause'
        if reason and not self.cancel_requested:
            if reason in ('localization_suspect','people_data_pause'):
                self._begin_localization_wait(self.state, self.deadline if self.state == 'going_observation'
                    else self.observation_session_deadline or self.deadline)
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
                elif self.state == 'going_people_standby':
                    self._parking_failed('goal_rejected')
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
            elif self.interrupt_reason in ('localization_suspect','people_data_pause'):
                self.state, self.reason = 'waiting_people_localization', self.interrupt_reason
                self._reset_spin_ready()
                self.spin_resume_pending = False
                self._log_people_state()
            elif self.state == 'going_people_standby' and self.interrupt_reason == 'selected_standby_safety_pause':
                self._begin_parking_recovery(self.interrupt_reason)
            elif (self.state == 'going_people_standby' and self.interrupt_reason in (
                    None, 'observation_or_navigation_timeout')):
                self._parking_failed(self.interrupt_reason or 'parking_nav2_failed')
            elif (self.state == 'observing' and self.interrupt_reason in RETRYABLE_SPIN_STOPS
                  and self.spin_resumes < self.config['people']['max_spin_resumes']
                  and self.spin_resume_requests < 4
                  and self.observation_session_deadline is not None
                  and time.monotonic() < self.observation_session_deadline):
                self.state, self.reason = 'waiting_observation_retry', self.interrupt_reason
                self._reset_spin_ready()
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
            self._reset_spin_ready()
        elif self.state == 'observing':
            self.settling_after_cancel = True
            self.stop_since = None
            if abs(self.spin_travel) < 5.8:
                self._people_finished('insufficient_rotation_coverage')
            else:
                self._finish_people_scan()
        elif self.state == 'going_people_standby':
            self.state = 'people_arrival_check'
            self.parking_outcome = 'verifying'
            self.parking_stop_since = None
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
        if self.state == 'waiting_people_localization':
            return self._poll_localization_wait()
        if self.state == 'waiting_people_parking_recovery':
            return self._poll_parking_recovery()
        if self.state in ('waiting_observation_stop', 'waiting_observation_retry'):
            if self.settling_after_cancel:
                return True
            retry = self.state == 'waiting_observation_retry'
            if (retry and (self.observation_session_deadline is None
                          or now >= self.observation_session_deadline)):
                self._people_finished('retry_localization_or_session_unavailable')
                return True
            if not self._safe():
                phase = self._localization_phase()
                if (phase not in (None,'SUSPECT','LOCALIZED') or not self.map_matches
                        or self.inhibited or self.stop_accepting
                        or (self.home is not None and (not self.home.healthy or self.home.cutoff))):
                    self._people_finished('observation_wait_unsafe')
                elif now >= self.deadline:
                    self._people_finished('spin_ready_timeout:unsafe_data')
                else:
                    self._reset_spin_ready()
                    self.spin_ready_blocker = 'waiting_fresh_safety_data'
                return True
            if now >= self.deadline:
                self._people_finished('spin_ready_timeout:'+self.spin_ready_blocker)
                return True
            pose = self._pose()
            point = people['observation_pose']
            if pose is None or math.hypot(pose['x']-point['x'],pose['y']-point['y']) > .10:
                self.spin_ready_blocker = 'observation_position_not_verified'
                if now >= self.deadline:
                    self._people_finished('observation_position_not_verified')
                return True
            if retry and self.spin_travel >= 5.8 and self._stationary():
                self._finish_people_scan()
                return True
            if retry and (self.spin_resumes >= people['max_spin_resumes'] or self.spin_resume_requests >= 4):
                self._people_finished('spin_resume_budget_exhausted')
                return True
            if self._spin_start_ready():
                if retry:
                    tree = self._resume_spin_tree()
                    if not self._start(point, 'observing', tree, people['observation_timeout_sec'],resume=True):
                        self.spin_ready_blocker = 'nav2_not_ready'
                    return True
                self.people = DirectionalPeople(people)
                self.observation_alignment = self._map_alignment()
                self.spin_travel = 0.0
                self.spin_resumes = 0
                self.observation_session_deadline = now+people['observation_session_timeout_sec']
                self.observation_collecting = True
                if not self._start(people['observation_pose'], 'observing',
                                  str(self.share / 'behavior_trees/people_observe.xml'),
                                  people['observation_timeout_sec']):
                    self.spin_ready_blocker = 'nav2_not_ready'
            return True
        if self.state in ('people_destination_selected', 'people_parking_retry_wait'):
            if now >= self.parking_deadline or self.parking_attempts >= self.parking_retry.max_attempts:
                self.parking_outcome = 'failed'
                self._people_finished('selected_standby_parking_timeout_or_attempt_limit',
                                      terminal_state='people_parking_blocked')
                return True
            if not self._safe():
                self._begin_parking_recovery('selected_standby_parking_unsafe')
                return True
            if not self._stationary():
                self.parking_stop_since = None
                return True
            self.parking_stop_since = self.parking_stop_since if self.parking_stop_since is not None else now
            if now-self.parking_stop_since < 1. or not self.parking_retry.can_attempt(now):
                return True
            timeout = people['navigation_timeout_sec'] if self.parking_attempts==0 else 30.
            if self._start(self.selected_standby, 'going_people_standby', '',
                           min(timeout,self.parking_deadline-now)):
                self.parking_attempts += 1
                self.parking_outcome = 'navigating' if self.parking_attempts==1 else 'correcting'
                self.parking_stop_since = None
                self._log_people_state()
            return True
        if self.state in ('people_arrival_check', 'people_parking_blocked'):
            if self._safe() and self._stationary() and self._at_goal(self.goal, 0.10, 0.10):
                self.parking_stop_since = self.parking_stop_since if self.parking_stop_since is not None else now
                if now-self.parking_stop_since >= 1.:
                    self.parked_point = dict(self.goal)
                    self._people_finished('parked_at_selected_standby')
                    # Let ordinary delivery know parking is verified, not just
                    # that Nav2 returned SUCCEEDED a few milliseconds earlier.
                    return True
            else:
                self.parking_stop_since = None
            if self.state == 'people_arrival_check' and now >= self.deadline:
                self._parking_failed('standby_arrival_not_verified')
            return True
        pose = self._pose()
        odom = None if self.last_odom is None else self.last_odom[1].pose.pose
        odom_pose = None if odom is None else (odom.position.x,odom.position.y,yaw_of(odom.orientation))
        safe=self._safe()
        stationary=self._stationary()
        transient=(not safe or pose is None) and (
            self.safety_report.get('failed_checks') in ([],['map_pose_valid'])
            and self._localization_phase() == 'LOCALIZED')
        if not self.idle_clock.update(now,parked=parked,safe=safe,
                                     stationary=stationary,pose=pose,odom=odom_pose,
                                     transient_tf_gap=transient):
            if self.idle_clock.elapsed == 0.:
                self.idle_since = None
            return False
        self.idle_since = self.idle_since if self.idle_since is not None else now
        if now < self.next_people_at or self.idle_clock.elapsed < people['idle_before_observation_sec']:
            return False
        self._start(people['observation_pose'], 'going_observation', '', people['navigation_timeout_sec'])
        return True
