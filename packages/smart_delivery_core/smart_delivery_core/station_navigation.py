"""Station staging using owned, executor-driven NavigateToPose actions only."""
import math
import time
from copy import deepcopy
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from rclpy.task import Future
from std_msgs.msg import String
from tf2_ros import TransformException

from smart_delivery_core.delivery_inputs import shared_inputs
from smart_delivery_core.station_geometry import StationGrid, angle_error


def coordinates(message):
    p,q = message.pose.position,message.pose.orientation
    return (p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))


class StationNavigation:
    """No new action/executor, no velocity bypass and no simultaneous goals."""

    def __init__(self, navigator, buffer, share):
        self.nav,self.buffer,self.share = navigator,buffer,Path(share)
        self.grid,self.grid_at,self.grid_stamp = None,-math.inf,0
        self.odom,self.odom_at = None,-math.inf
        self.running = self.cancelled = False
        self.phase,self.result,self.parked = 'idle',TaskResult.UNKNOWN,None
        self.stage,self.stop_since = None,None
        self.active_goal = False
        self.last_report = None
        self.last_wait_publish = self.last_console_at = -math.inf
        self.grid_gate,self.pose_gate,self.odom_gate = {},{},{}
        self.grid_error = ''
        self.preflight_wait_sec = 30.
        self.publisher = navigator.create_publisher(String,'/delivery/station_state',10)
        navigator.create_subscription(OccupancyGrid,'/global_costmap/costmap',self._grid,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        shared_inputs(navigator).subscribe(Odometry,'/odom',self._odom,10)

    def _grid(self, message):
        try:
            self.grid = StationGrid(message) if message.header.frame_id=='map' else None
            self.grid_at = time.monotonic()
            self.grid_stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
            self.grid_error = '' if self.grid is not None else 'frame_not_map'
        except (ValueError,AttributeError) as exc:
            self.grid = None
            self.grid_error = str(exc)

    def _odom(self, message):
        self.odom,self.odom_at = message,time.monotonic()

    def _fresh(self, receipt, stamp_ns, limit):
        age = (self.nav.get_clock().now().nanoseconds-stamp_ns)/1e9
        return time.monotonic()-receipt <= limit and -.1 <= age <= limit

    def _ages(self, receipt, stamp_ns, limit):
        receipt_age = time.monotonic()-receipt
        source_age = (self.nav.get_clock().now().nanoseconds-stamp_ns)/1e9
        return dict(receipt_age_sec=round(receipt_age,4) if math.isfinite(receipt_age) else None,
                    source_age_sec=round(source_age,4) if stamp_ns>0 else None,
                    max_age_sec=limit)

    def _pose(self):
        try:
            tr = self.buffer.lookup_transform('map','base_footprint',Time())
            ns = tr.header.stamp.sec*10**9+tr.header.stamp.nanosec
            age = self._ages(time.monotonic(),ns,1.)
            age.pop('receipt_age_sec')  # Buffer lookup does not expose receipt time.
            self.pose_gate = dict(available=True,receipt_age_observed=False,**age)
            if not self._fresh(time.monotonic(),ns,1.): return None
            p,q = tr.transform.translation,tr.transform.rotation
            pose = (p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
            self.pose_gate['finite'] = all(math.isfinite(v) for v in pose)
            return pose if self.pose_gate['finite'] else None
        except TransformException as exc:
            self.pose_gate = dict(available=False,error=str(exc)[:250])
            return None

    def _stopped(self):
        if self.odom is None:
            self.odom_gate = dict(available=False)
            return False
        stamp = self.odom.header.stamp
        ns = stamp.sec*10**9+stamp.nanosec
        self.odom_gate = dict(available=True,**self._ages(self.odom_at,ns,.4))
        if not self._fresh(self.odom_at,ns,.4):
            self.odom_gate['fresh'] = False
            return False
        v = self.odom.twist.twist
        finite = all(math.isfinite(value) for value in (v.linear.x,v.linear.y,v.angular.z))
        self.odom_gate.update(fresh=True,finite=finite)
        if not finite: return False
        self.odom_gate.update(linear_speed_mps=round(math.hypot(v.linear.x,v.linear.y),4),
                              angular_speed_rad_s=round(v.angular.z,4))
        return (math.hypot(v.linear.x,v.linear.y) <= .02 and abs(v.angular.z) <= .03)

    def _data(self):
        self.grid_gate = dict(available=self.grid is not None,error=self.grid_error,
                              **self._ages(self.grid_at,self.grid_stamp,2.5))
        return self.grid is not None and self._fresh(self.grid_at,self.grid_stamp,2.5)

    def _report(self, phase, reason='', *, force=False):
        self.phase = phase
        transition = self.last_report is None or self.last_report[0]!=phase
        same = (phase,reason)==self.last_report
        if same and not force: return
        self.last_report = phase,reason
        labels = {'waiting_data':'⏳ 進出站等待資料','departure':'↔️ 低速出站',
            'staging':'➡️ 前往進站準備點','entry':'↔️ 低速進站','endpoint':'🐢 末段停靠',
            'settling':'⏳ 確認停妥','arrived':'✅ 停靠完成','failed':'⚠️ 進出站中止',
            'cancelled':'⏸️ 進出站已取消','transit':'➡️ 繼續導航','reached':'✅ 已抵達目標'}
        if transition or time.monotonic()-self.last_console_at>=5.:
            self.nav.get_logger().info(labels.get(phase,phase)+(f'：{reason}' if reason else ''))
            self.last_console_at = time.monotonic()
        import json
        self.publisher.publish(String(data=json.dumps(dict(phase=phase,reason=reason,
            target=getattr(self,'target',None),stage=self.stage,
            goals_submitted=getattr(self,'goals_submitted',0),
            preflight_timeout=getattr(self,'preflight_timeout',False),
            inputs=dict(costmap=self.grid_gate,tf=self.pose_gate,odom=self.odom_gate),
            wait_blockers=getattr(self,'wait_blockers',[]),
            deadline_remaining_sec=round(max(0.,getattr(self,'deadline',0.)-time.monotonic()),2),
            stamp_ns=self.nav.get_clock().now().nanoseconds),ensure_ascii=False)))

    def _waiting(self, pose, data_ok, stopped, *, position_ok=True):
        blockers = []
        if not data_ok: blockers.append('costmap_missing' if self.grid is None else 'costmap_stale')
        if pose is None: blockers.append('tf_missing_or_stale')
        if not stopped:
            blockers.append('odom_missing' if self.odom is None else
                            'odom_stale' if self.odom_gate.get('fresh') is False else
                            'odom_invalid' if self.odom_gate.get('finite') is False else 'vehicle_not_stopped')
        if not position_ok: blockers.append('stage_pose_pending')
        self.wait_blockers = blockers or ['continuous_stopped_confirmation']
        if time.monotonic()-self.last_wait_publish<1.: return
        self.last_wait_publish = time.monotonic()
        labels = {'costmap_missing':'等待代價地圖','costmap_stale':'等待代價地圖更新',
                  'tf_missing_or_stale':'等待車輛座標更新','odom_missing':'等待里程計',
                  'odom_stale':'等待里程計更新','vehicle_not_stopped':'等待車輛停妥',
                  'odom_invalid':'等待有效里程計',
                  'stage_pose_pending':'等待實際位置確認',
                  'continuous_stopped_confirmation':'確認連續停妥'}
        self._report(self.phase,'、'.join(labels[b] for b in self.wait_blockers),force=True)

    def _pump_callbacks(self):
        # spin_once executes ONE callback, even with a nonzero timeout. Drain
        # inputs/timers for a bounded interval instead, like BasicNavigator's
        # native action wait. No new executor/thread and no polling sleep.
        pending = Future()
        try:
            rclpy.spin_until_future_complete(self.nav,pending,timeout_sec=.10)
        finally:
            pending.cancel()

    def start(self, goal, permitted=lambda:True, *, park=True, behavior_tree=''):
        if self.running: raise RuntimeError('station navigation still owns an action')
        self.goal,self.target,self.permitted = deepcopy(goal),coordinates(goal),permitted
        if goal.header.frame_id!='map' or not all(math.isfinite(v) for v in self.target):
            raise ValueError('station target must be finite and in map frame')
        self.park,self.tree = park,behavior_tree
        self.running,self.cancelled,self.active_goal = True,False,False
        self.result,self.stage,self.stop_since = TaskResult.UNKNOWN,None,None
        self.failure = ''
        self.preflight_timeout = False
        self.goals_submitted = 0
        self.wait_blockers = []
        self.last_wait_publish = -math.inf
        self.data_unavailable = False
        self.translation_start = None
        self.native_error = (0,'')
        self.after_stop,self.start_at,self.deadline = 'prepare',time.monotonic(),time.monotonic()+self.preflight_wait_sec
        self._report('waiting_data')

    def _pose_message(self, point):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.nav.get_clock().now().to_msg()
        goal.pose.position.x,goal.pose.position.y = point[:2]
        goal.pose.orientation.z,goal.pose.orientation.w = math.sin(point[2]/2),math.cos(point[2]/2)
        return goal

    def _send(self, point, phase, tree='', next_phase=None):
        # Native goToPose pumps all lease/safety callbacks while accepting goals.
        if self.cancelled or not self.permitted():
            self._finish(TaskResult.CANCELED,'cancelled'); return
        self.stage,self.next_phase = point,next_phase
        self.native_error = (0,'')
        self.stop_since = None
        self.deadline = time.monotonic()+(25. if phase in ('departure','entry','endpoint') else 180.)
        self.goals_submitted += 1
        self._report(phase)
        accepted = self.nav.goToPose(self._pose_message(point),behavior_tree=tree)
        self.active_goal = accepted is not False
        if not self.active_goal: self._finish(TaskResult.FAILED,'failed','Nav2 拒絕目標')

    def _translation(self, start, end, phase, next_phase):
        if (not self._data() or not self.grid.segment(start,end)
                or abs(angle_error(start[2],end[2]))>.12):
            self._finish(TaskResult.FAILED,'failed','平移通道未通過檢查'); return
        dx,dy = end[0]-start[0],end[1]-start[1]
        along = math.cos(end[2])*dx+math.sin(end[2])*dy
        across = -math.sin(end[2])*dx+math.cos(end[2])*dy
        axis = 'x' if abs(along)>=abs(across) else 'y'
        self.translation_start = start
        slow = phase=='endpoint'
        tree = str(self.share/'behavior_trees'/f'station_{axis}{"_slow" if slow else ""}.xml')
        self._send(end,phase,tree,next_phase)

    def _prepare(self, pose):
        # A remembered parking pose is only a hint; after a lift/relocalization
        # recompute from the live pose. Also support restart next to a wall.
        near_park = self.parked is not None and math.hypot(pose[0]-self.parked[0],pose[1]-self.parked[1])<.25
        distance = math.hypot(pose[0]-self.target[0],pose[1]-self.target[1])
        already_there = distance<=.10 and abs(angle_error(pose[2],self.target[2]))<=.10
        if not already_there and ((near_park and distance>.3) or not self.grid.clear(pose,rotation=True)):
            departure = self.grid.departure(pose,self.target)
            if departure is None:
                self._finish(TaskResult.FAILED,'failed','沒有安全出站方向'); return
            self.parked = None
            self._translation(pose,departure,'departure','approach'); return
        self.parked = None
        self._approach(pose)

    def _approach(self, pose):
        if (self.park and math.hypot(pose[0]-self.target[0],pose[1]-self.target[1])<=.10
                and abs(angle_error(pose[2],self.target[2]))<=.10):
            self.stage,self.after_stop,self.stop_since = self.target,'finish',None
            self.deadline = time.monotonic()+5.
            self._report('settling'); return
        if not self.grid.clear(pose,rotation=True):
            self._finish(TaskResult.FAILED,'failed','出站後旋轉空間仍不足'); return
        if not self.park:
            self._send(self.target,'transit',self.tree,'finish'); return
        stage = self.grid.staging(self.target,pose)
        if stage is None:
            self._finish(TaskResult.FAILED,'failed','沒有安全進站方向'); return
        if (math.hypot(pose[0]-stage[0],pose[1]-stage[1])<=.10
                and abs(angle_error(pose[2],stage[2]))<=.10):
            self.after_stop = 'entry'
            self._advance(pose)
            return
        self._send(stage,'staging','', 'entry')

    def _advance(self, pose):
        phase = self.after_stop
        if phase=='prepare': self._prepare(pose)
        elif phase=='approach': self._approach(pose)
        elif phase=='entry':
            # Ensure rotation finished in the open staging area. The final
            # approach never fixes a large heading error beside a wall.
            if abs(angle_error(pose[2],self.target[2]))>.12:
                self._finish(TaskResult.FAILED,'failed','進站車頭尚未對正'); return
            length = math.hypot(self.target[0]-pose[0],self.target[1]-pose[1])
            if length > .30:
                ratio = (length-.20)/length
                middle = (pose[0]+(self.target[0]-pose[0])*ratio,
                          pose[1]+(self.target[1]-pose[1])*ratio,self.target[2])
                self._translation(pose,middle,'entry','endpoint')
            else: self._translation(pose,self.target,'endpoint','finish')
        elif phase=='endpoint': self._translation(pose,self.target,'endpoint','finish')
        elif phase=='finish':
            if self.park: self.parked = self.target
            self._finish(TaskResult.SUCCEEDED,'arrived' if self.park else 'reached')

    def _finish(self, result, phase, reason=''):
        self.running,self.active_goal,self.result = False,False,result
        self._report(phase,reason)

    def cancelTask(self):
        self.cancelled = True
        if self.active_goal: self.nav.cancelTask()
        else: self._finish(TaskResult.FAILED if self.failure else TaskResult.CANCELED,
                           'failed' if self.failure else 'cancelled',self.failure)

    def isTaskComplete(self):
        if not self.running: return True
        # Process queued sensor and cancellation callbacks BEFORE evaluating
        # freshness. The production caller has many more inputs than a fake
        # three-topic fixture and must never age its own queued messages.
        completed = self.nav.isTaskComplete() if self.active_goal else None
        if not self.active_goal: self._pump_callbacks()
        if not self.running: return True
        pose,data_ok = self._pose(),self._data()
        # Pump native actions even when cancelling; do not release the mission
        # lease or submit a successor until the server reports a terminal state.
        if not self.cancelled and time.monotonic()>self.deadline:
            self.preflight_timeout = self.goals_submitted==0 and self.phase=='waiting_data'
            self.failure = ('導航前置確認逾時，尚未送出目標' if self.preflight_timeout
                            else '階段逾時，未確認停妥')
            self.data_unavailable = not data_ok or pose is None
            self.cancelTask()
        if self.active_goal and self.phase in ('departure','entry','endpoint') and not self.cancelled:
            if pose is None or not data_ok:
                self.failure,self.data_unavailable = '平移資料逾時',True
                self.cancelTask()
            elif abs(angle_error(pose[2],self.stage[2]))>.20:
                self.failure = '平移車頭方向未通過檢查'
                self.cancelTask()
            else:
                sx,sy,_ = self.translation_start
                dx,dy = self.stage[0]-sx,self.stage[1]-sy
                length = math.hypot(dx,dy)
                along = ((pose[0]-sx)*dx+(pose[1]-sy)*dy)/max(length,1e-6)
                across = abs((pose[0]-sx)*dy-(pose[1]-sy)*dx)/max(length,1e-6)
                if across>.10 or not -.10<=along<=length+.10:
                    self.failure = '平移偏離已檢查通道'
                    self.cancelTask()
        if not self.cancelled and not self.permitted():
            self.cancelTask()
        if self.active_goal:
            if not completed:
                return False
            self.active_goal = False
            result = self.nav.getResult()
            if result != TaskResult.SUCCEEDED:
                try:
                    message = self.nav.result_future.result().result
                    self.native_error = int(message.error_code),str(message.error_msg)
                except (AttributeError,TypeError,ValueError):
                    pass
            if self.cancelled or result != TaskResult.SUCCEEDED:
                self._finish(TaskResult.FAILED if self.failure else TaskResult.CANCELED if self.cancelled else result,
                             'failed' if self.failure or not self.cancelled else 'cancelled',self.failure)
                return True
            self.after_stop,self.stop_since = self.next_phase,None
            self.deadline = time.monotonic()+6.
            self._report('settling')
        if self.cancelled or not self.permitted():
            self._finish(TaskResult.FAILED if self.failure else TaskResult.CANCELED,
                         'failed' if self.failure else 'cancelled',self.failure); return True
        stopped = self._stopped()
        position_ok = self.stage is None or (pose is not None
            and math.hypot(pose[0]-self.stage[0],pose[1]-self.stage[1])<=.10
            and abs(angle_error(pose[2],self.stage[2]))<=.10)
        self._waiting(pose,data_ok,stopped,position_ok=position_ok)
        if pose is None or not data_ok or not stopped or not position_ok:
            # Action results and TF are separate DDS streams. Wait inside the
            # bounded settling phase rather than treating their arrival order
            # as physical failure (or accepting an old pose as arrival).
            self.stop_since = None
            return False
        self.stop_since = time.monotonic() if self.stop_since is None else self.stop_since
        if time.monotonic()-self.stop_since >= .5: self._advance(pose)
        return not self.running

    def getResult(self):
        return self.result

    def error_details(self):
        return (0,self.failure or self.last_report[1]) if self.failure or self.last_report[1] else self.native_error
