"""Station staging using owned, executor-driven NavigateToPose actions only."""
import math
import time
from copy import deepcopy
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
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
        self.last_plan = None
        self.chassis_command,self.chassis_command_at=None,-math.inf
        self.static_grid = None
        navigator.create_subscription(NavPath,'/plan',self._plan,1)
        navigator.create_subscription(OccupancyGrid,'/map',self._map,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.publisher = navigator.create_publisher(String,'/delivery/station_state',10)
        navigator.create_subscription(OccupancyGrid,'/global_costmap/costmap',self._grid,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        shared_inputs(navigator).subscribe(Odometry,'/odom',self._odom,10)
        shared_inputs(navigator).subscribe(Twist,'/chassis_cmd_vel',self._chassis_command,1)

    def _plan(self, message):
        self.last_plan = time.monotonic(),message

    def _map(self, message):
        try:self.static_grid=StationGrid(message) if message.header.frame_id=='map' else None
        except (ValueError,AttributeError):self.static_grid=None

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
        stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
        if self.odom is not None:
            old=self.odom.header.stamp.sec*10**9+self.odom.header.stamp.nanosec
            if stamp<=old:
                if self.nav.get_clock().now().nanoseconds>=old-100_000_000:return
                # Clock epoch changed: never compare a new pose to an old anchor.
                if self.running:
                    self.failure,self.data_unavailable='里程計時間基準更新，等待資料恢復',True
                    self.cancelTask()
        self.odom,self.odom_at = message,time.monotonic()

    def _chassis_command(self, message):
        now=time.monotonic();rate=message.angular.z
        if self.phase=='aligning' and (now-self.chassis_command_at>.3
                or not math.isfinite(rate) or rate*getattr(self,'align_sign',0.)<.12):
            self.align_progress_at=now
        self.chassis_command,self.chassis_command_at=rate,now

    def _fresh(self, receipt, stamp_ns, limit):
        age = (self.nav.get_clock().now().nanoseconds-stamp_ns)/1e9
        return 0 <= time.monotonic()-receipt <= limit and -.1 <= age <= limit

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
            'staging':'➡️ 前往進站準備點','entry':'↔️ 連續低速進站',
            'direct':'➡️ 直接導航','aligning':'🧭 車頭對正',
            'waiting_action_health':'⏳ 確認導航回應',
            'waiting_clearance':'⏳ 等待進出站通道',
            'settling':'⏳ 確認停妥','arrived':'✅ 停靠完成','failed':'⚠️ 進出站中止',
            'cancelled':'⏸️ 進出站已取消','transit':'➡️ 繼續導航','reached':'✅ 已抵達目標'}
        if transition or time.monotonic()-self.last_console_at>=5.:
            label=getattr(self,'target_label','')
            if phase=='aligning':
                degrees = math.degrees(angle_error(self.stage[2], self.align_anchor[2]))
                label = f'{degrees:+.0f}°'+(f' → {label}' if label else '')
            if phase=='departure':
                source=getattr(self,'source_station',None)
                label=(f'離開 ({source[0]:.2f}, {source[1]:.2f}) → {label}'
                       if source else f'離開目前位置 → {label}')
            suffix=(f'：{label}' if label else '')+(f'｜{reason}' if reason else '')
            self.nav.get_logger().info(labels.get(phase,phase)+suffix)
            self.last_console_at = time.monotonic()
        import json
        self.publisher.publish(String(data=json.dumps(dict(phase=phase,reason=reason,
            target=getattr(self,'target',None),stage=self.stage,
            target_name=getattr(self,'target_label',''),
            goals_submitted=getattr(self,'goals_submitted',0),
            preflight_timeout=getattr(self,'preflight_timeout',False),
            mode=getattr(self,'mode',None),mode_reason=getattr(self,'mode_reason',''),
            mode_switches=getattr(self,'mode_switches',0),
            mode_switch_scope=getattr(self,'mode_switch_scope',None),
            decision_time_ms=getattr(self,'decision_time_ms',None),
            alignment_attempts=getattr(self,'alignment_attempts',0),
            final_corrections=getattr(self,'final_corrections',0),
            remote_preparation=getattr(self,'remote_preparation',False),
            clearance_scope=getattr(self,'clearance_scope',''),
            clearance_kind=getattr(self,'clearance_kind',''),
            motion_stalled=getattr(self,'motion_stalled',False),
            clearance_timeout=getattr(self,'clearance_timeout',False),
            native_error=getattr(self,'native_error',(0,'')),
            inputs=dict(costmap=self.grid_gate,tf=self.pose_gate,odom=self.odom_gate),
            motion_check=getattr(self,'motion_check',{}),
            clearance_retries=getattr(self,'clearance_retries',0),
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
        if self.phase=='waiting_clearance':blockers.append('station_clearance')
        self.wait_blockers = blockers or ['continuous_stopped_confirmation']
        if time.monotonic()-self.last_wait_publish<1.: return
        self.last_wait_publish = time.monotonic()
        labels = {'costmap_missing':'等待代價地圖','costmap_stale':'等待代價地圖更新',
                  'tf_missing_or_stale':'等待車輛座標更新','odom_missing':'等待里程計',
                  'odom_stale':'等待里程計更新','vehicle_not_stopped':'等待車輛停妥',
                  'odom_invalid':'等待有效里程計',
                  'station_clearance':getattr(self,'clearance_detail','等待進出站通道'),
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

    def start(self, goal, permitted=lambda:True, *, park=True, behavior_tree='',label=''):
        if self.running: raise RuntimeError('station navigation still owns an action')
        self.goal,self.target,self.permitted = deepcopy(goal),coordinates(goal),permitted
        if goal.header.frame_id!='map' or not all(math.isfinite(v) for v in self.target):
            raise ValueError('station target must be finite and in map frame')
        self.source_station = self.parked
        self.park,self.tree = park,behavior_tree
        self.target_label=' '.join(str(label).split())[:40]
        self.running,self.cancelled,self.active_goal = True,False,False
        self.result,self.stage,self.stop_since = TaskResult.UNKNOWN,None,None
        self.failure = ''
        self.preflight_timeout = False
        self.goals_submitted = 0
        self.wait_blockers = []
        self.last_wait_publish = -math.inf
        self.data_unavailable = False
        self.translation_start = None
        self.motion_odom_anchor = None
        self.motion_check = {}
        self.clearance_retries = 0
        self.clearance_pending = None
        self.mode,self.mode_reason,self.mode_switches = None,'',0
        self.decision_time_ms = None
        self.replan_pending = False
        self.final_heading_pending = False
        self.final_heading_since = None
        self.checked_plan = None
        self.mode_switch_scope = None
        self.alignment_attempts,self.final_corrections = 0,0
        self.align_anchor = None
        self.align_odom_anchor = None
        self.align_progress,self.align_progress_at = 0.,0.
        self.motion_stalled = False
        self.remote_preparation = False
        self.clearance_scope,self.clearance_detail,self.clearance_kind = '','',''
        self.clearance_timeout = False
        self.clearance_deadline,self.clearance_grid_stamp = None,None
        self.sequence_deadline = time.monotonic()+240.
        self.native_error = (0,'')
        self.pending_unknown_failure = None
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
        allowance = (min(22., max(12., abs(angle_error(point[2], self.align_anchor[2]))/.20+6.))
                     if phase=='aligning' else 25. if phase in ('departure','entry') else 180.)
        self.deadline = min(self.sequence_deadline,time.monotonic()+allowance)
        self.clearance_deadline = None
        self.clearance_scope,self.clearance_detail,self.clearance_kind='','',''
        self.route_origin = self._pose()
        self.route_sent_at = time.monotonic()
        self.goals_submitted += 1
        self._report(phase)
        goal=self._pose_message(point)
        if hasattr(self.nav,'configure_navigation_goal'):
            tree=self.nav.configure_navigation_goal(goal,tree)
        accepted = self.nav.goToPose(goal,behavior_tree=tree)
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
        self.motion_odom_anchor = self._odom_pose()
        tree = str(self.share/'behavior_trees'/f'station_{"exit_" if phase=="departure" else ""}{axis}.xml')
        self._send(end,phase,tree,next_phase)

    def _wait_clearance(self, reason, resume, scope='local_departure'):
        if self.clearance_deadline is None:
            self.clearance_deadline = min(self.sequence_deadline,time.monotonic()+30.)
        self.deadline = self.clearance_deadline
        self.clearance_grid_stamp = self.grid_stamp
        self.clearance_resume = resume
        self.clearance_scope,self.clearance_detail=scope,reason
        point=self.target if scope.startswith('destination') else self._pose()
        self.clearance_kind=('static_map_unavailable' if self.static_grid is None else
            'static_or_unknown' if point and not self.static_grid.clear(point) else 'dynamic_costmap')
        self.after_stop = 'clearance'
        self._report('waiting_clearance',reason)

    def _align(self, pose, resume):
        if self.alignment_attempts>=2:
            self._finish(TaskResult.FAILED,'failed','車頭對正仍未達要求');return
        if not self.grid.turn_clear(pose,self.target[2]):
            self._wait_clearance('等待車頭對正空間',
                'final_align' if resume=='final_translation' else resume);return
        self.alignment_attempts += 1
        self.align_anchor = pose
        self.motion_odom_anchor = self._odom_pose()
        self.align_odom_anchor = self._odom_heading()
        self.align_sign = 1. if angle_error(self.target[2],pose[2])>0 else -1.
        self.align_progress,self.align_progress_at=0.,time.monotonic()
        self._send((pose[0],pose[1],self.target[2]),'aligning',
            str(self.share/'behavior_trees/station_align.xml'),resume)

    def _choose_mode(self, pose):
        if self.mode is not None:return
        started = time.perf_counter()
        ordinary = self.grid.normal_route_safe(pose,self.target)
        self.decision_time_ms = round((time.perf_counter()-started)*1000.,3)
        self.mode = 'normal' if ordinary else 'station'
        self.mode_reason = 'verified_normal_clearance' if ordinary else 'restricted_station_turn'

    def _prepare(self, pose):
        self._choose_mode(pose)
        distance = math.hypot(pose[0]-self.target[0],pose[1]-self.target[1])
        already_there = distance<=.10 and abs(angle_error(pose[2],self.target[2]))<=.10
        if self.mode=='normal' or already_there:
            self.parked=None;self._approach(pose);return
        if not self.grid.clear(pose,rotation=True):
            departure = self.grid.departure(pose,self.target)
            if departure is None:
                self._wait_clearance('等待安全離站方向','prepare'); return
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
        if self.mode=='normal':
            self._send(self.target,'direct',self.tree,'finish');return
        if not self.park:
            if not self.grid.clear(pose,rotation=True) and not self.grid.aligned_route(pose,self.target):
                self._wait_clearance('等待安全轉向空間','prepare');return
            self._send(self.target,'transit',self.tree,'finish'); return
        self.remote_preparation=math.hypot(pose[0]-self.target[0],pose[1]-self.target[1])>1.
        coarse=self.remote_preparation and self.static_grid is not None
        stage = self.grid.staging(self.target,pose,final_clear=not coarse,
                                  static_grid=self.static_grid)
        if stage is None:
            self._wait_clearance('目的地準備區受阻','approach','destination_preparation'); return
        if (math.hypot(pose[0]-stage[0],pose[1]-stage[1])<=.20
                and math.hypot(pose[0]-self.target[0],pose[1]-self.target[1])<=.75
                and self.grid.turn_clear(pose,self.target[2])):
            self.after_stop = 'entry'
            self._advance(pose)
            return
        distant=math.hypot(stage[0]-self.target[0],stage[1]-self.target[1])>.65+1e-6
        if distant and math.hypot(pose[0]-stage[0],pose[1]-stage[1])<=.20:
            self._wait_clearance('等待目的地重新觀測','approach','destination_preparation');return
        self._send(stage,'staging','', 'approach' if distant else 'entry')

    def _advance(self, pose):
        phase = self.after_stop
        if phase=='prepare': self._prepare(pose)
        elif phase=='approach': self._approach(pose)
        elif phase=='entry':
            # Ensure rotation finished in the open staging area. The final
            # approach never fixes a large heading error beside a wall.
            if abs(angle_error(pose[2],self.target[2]))>.10:
                self._align(pose,'entry');return
            length = math.hypot(self.target[0]-pose[0],self.target[1]-pose[1])
            if length>.75:
                self._finish(TaskResult.FAILED,'failed','準備點偏離過遠');return
            self.remote_preparation=False
            if not self.grid.segment(pose,self.target):
                self._wait_clearance('目的地末段通道受阻','entry','destination_final');return
            self._translation(pose,self.target,'entry','finish')
        elif phase=='final_translation':
            if (math.hypot(pose[0]-self.target[0],pose[1]-self.target[1])<=.10
                    and abs(angle_error(pose[2],self.target[2]))<=.10):
                self._advance_finish()
            else:self._translation(pose,self.target,'entry','finish')
        elif phase=='final_align':self._align(pose,'final_translation')
        elif phase=='translation_clearance':
            self._wait_clearance('等待平移通道恢復','translation_resume','local_translation')
        elif phase=='translation_resume':
            moving,end,resume=self.clearance_pending
            if not self.grid.segment(pose,end):
                self._wait_clearance('等待平移通道恢復','translation_resume','local_translation');return
            self.clearance_pending=None
            self._translation(pose,end,moving,resume)
        elif phase=='finish':
            self._advance_finish()

    def _advance_finish(self):
        if self.park:self.parked=self.target
        self._finish(TaskResult.SUCCEEDED,'arrived' if self.park else 'reached')

    def _normal_plan_unsafe(self, pose):
        if (self.mode!='normal' or self.mode_switches or not self.last_plan
                or self.last_plan[0]<self.route_sent_at):return False
        _,path = self.last_plan
        key=(path.header.stamp.sec,path.header.stamp.nanosec,len(path.poses))
        if key==self.checked_plan or path.header.frame_id!='map' or len(path.poses)<2:return False
        self.checked_plan=key
        points=[(p.pose.position.x,p.pose.position.y) for p in path.poses]
        if math.hypot(points[-1][0]-self.target[0],points[-1][1]-self.target[1])>.15:return False
        if (self.park and math.hypot(pose[0]-self.target[0],pose[1]-self.target[1])<=1.
                and not self.grid.clear(self.target,rotation=True)):
            near=next((p for p in reversed(points[:-1])
                       if math.hypot(p[0]-points[-1][0],p[1]-points[-1][1])>.20),None)
            if near:
                heading=math.atan2(points[-1][1]-near[1],points[-1][0]-near[0])
                if not self.grid.turn_clear((self.target[0],self.target[1],heading),self.target[2]):
                    self.mode_switch_scope='destination'
                    return True
        if self.route_origin and math.hypot(pose[0]-self.route_origin[0],pose[1]-self.route_origin[1])<.6:
            ahead=next((p for p in points if math.hypot(p[0]-pose[0],p[1]-pose[1])>.25),None)
            if ahead and not self.grid.turn_clear(pose,math.atan2(ahead[1]-pose[1],ahead[0]-pose[0])):
                self.mode_switch_scope='departure'
                return True
        return False

    def _request_station_fallback(self):
        self.mode,self.mode_reason,self.mode_switches = 'station','actual_route_requires_station',1
        self.replan_pending=True
        self.nav.cancelTask()

    def _finish_heading_stalled(self, pose):
        # Native ordinary MPPI can reach XY yet produce a yaw command below
        # this chassis' breakaway torque. Use the existing bounded shim only
        # after fresh odometry proves it is stopped, in verified turning space.
        eligible=(self.park and self.phase=='direct' and self.mode=='normal'
            and math.hypot(pose[0]-self.target[0],pose[1]-self.target[1])<=.10
            and .10<abs(angle_error(pose[2],self.target[2]))<=.35
            and self._stopped() and self.grid.turn_clear(pose,self.target[2]))
        if not eligible:
            self.final_heading_since=None
            return False
        now=time.monotonic()
        self.final_heading_since=now if self.final_heading_since is None else self.final_heading_since
        return now-self.final_heading_since>=.5

    @staticmethod
    def _blocked_failure(code, message):
        from nav2_msgs.action import FollowPath
        blocked={getattr(FollowPath.Result,k,-999) for k in
                 ('FAILED_TO_MAKE_PROGRESS','PATIENCE_EXCEEDED','NO_VALID_CONTROL')}
        return code in blocked or any(text in message.lower() for text in
            ('failed to make progress','no valid trajectories','patience exceeded'))

    def _finish(self, result, phase, reason=''):
        self.running,self.active_goal,self.result = False,False,result
        self._report(phase,reason)

    def _odom_pose(self):
        if self.odom is None:return None
        stamp=self.odom.header.stamp
        if not self._fresh(self.odom_at,stamp.sec*10**9+stamp.nanosec,.4):return None
        p,q=self.odom.pose.pose.position,self.odom.pose.pose.orientation
        pose=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        return pose if all(math.isfinite(v) for v in pose) else None

    def _physical_pose(self, anchor):
        origin=getattr(self,'motion_odom_anchor',None)
        if origin is None:return self._pose()  # Conservative legacy map-only guard.
        current=self._odom_pose()
        if current is None:return None
        offset=anchor[2]-origin[2];c,s=math.cos(offset),math.sin(offset)
        dx,dy=current[0]-origin[0],current[1]-origin[1]
        return (anchor[0]+c*dx-s*dy,anchor[1]+s*dx+c*dy,
                anchor[2]+angle_error(current[2],origin[2]))

    def _translation_blocker(self, start, end):
        if abs(angle_error(start[2],end[2]))>.12:
            return dict(reason='heading_mismatch',yaw_error_rad=angle_error(start[2],end[2]))
        steps=max(1,math.ceil(math.hypot(end[0]-start[0],end[1]-start[1])/.02))
        for i in range(steps+1):
            p=(start[0]+(end[0]-start[0])*i/steps,start[1]+(end[1]-start[1])*i/steps,end[2])
            if not self.grid.clear(p):
                return dict(reason='occupied_or_unknown_sweep',pose=p,
                    static_clear=self.static_grid.clear(p) if self.static_grid else None)
        return dict(reason='changed_snapshot')

    def _pause_translation(self):
        if self.clearance_retries>=2:
            self.failure='平移通道持續受阻';self.cancelTask();return
        self.clearance_retries+=1
        self.clearance_pending=(self.phase,self.stage,self.next_phase)
        self.replan_pending=True
        self.nav.cancelTask()

    def _odom_heading(self):
        if self.odom is None:return None
        q=self.odom.pose.pose.orientation
        return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

    def _alignment_stalled(self):
        heading=self._odom_heading()
        if heading is None or self.align_odom_anchor is None:return False
        stamp=self.odom.header.stamp;ns=stamp.sec*10**9+stamp.nanosec
        if not self._fresh(self.odom_at,ns,.4):
            self.failure,self.data_unavailable='對正輪速資料過期',True
            self.cancelTask();return False
        # No-progress is a chassis symptom only with continuous effective
        # FINAL commands. Yielding/near-stop/expired command streams must not
        # permanently block parking as if the actuator had stalled.
        if (self.chassis_command is None or time.monotonic()-self.chassis_command_at>.3
                or not math.isfinite(self.chassis_command)
                or self.chassis_command*self.align_sign<.12):
            self.align_progress_at=time.monotonic();return False
        progress=self.align_sign*angle_error(heading,self.align_odom_anchor)
        if progress-self.align_progress>=math.radians(.5):
            self.align_progress,self.align_progress_at=progress,time.monotonic()
        return time.monotonic()-self.align_progress_at>=2.5

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
        pending=getattr(self,'pending_unknown_failure',None)
        if pending is not None:
            if not self.nav.navigation_actions_ready():
                self.failure,self.data_unavailable='導航回應異常，等待導航恢復',True
            elif time.monotonic()<pending[1] and not self.cancelled:
                return False
            result=TaskResult.CANCELED if self.cancelled else pending[0]
            self.pending_unknown_failure=None
            self._finish(TaskResult.FAILED if self.failure else result,
                         'cancelled' if self.cancelled and not self.failure else 'failed',self.failure)
            return True
        pose,data_ok = self._pose(),self._data()
        # Pump native actions even when cancelling; do not release the mission
        # lease or submit a successor until the server reports a terminal state.
        if not self.cancelled and time.monotonic()>self.deadline:
            self.preflight_timeout = self.goals_submitted==0 and self.phase=='waiting_data'
            self.clearance_timeout = self.phase=='waiting_clearance'
            self.failure = ('導航前置確認逾時，尚未送出目標' if self.preflight_timeout
                            else '進出站通道持續受阻' if self.clearance_timeout
                            else '階段逾時，未確認停妥')
            self.data_unavailable = not data_ok or pose is None
            self.cancelTask()
        if (self.active_goal and not completed and self.phase in ('departure','entry')
                and not self.cancelled and not self.replan_pending):
            if pose is None or not data_ok:
                self.failure,self.data_unavailable = '平移資料逾時',True
                self.cancelTask()
            elif abs(angle_error(pose[2],self.stage[2]))>.20:
                self.failure = '平移車頭方向未通過檢查'
                self.cancelTask()
            else:
                physical=self._physical_pose(self.translation_start)
                if physical is None:
                    self.failure,self.data_unavailable='平移輪速資料過期',True
                    self.cancelTask();return False
                sx,sy,_ = self.translation_start
                dx,dy = self.stage[0]-sx,self.stage[1]-sy
                length = math.hypot(dx,dy)
                along = ((physical[0]-sx)*dx+(physical[1]-sy)*dy)/max(length,1e-6)
                across = abs((physical[0]-sx)*dy-(physical[1]-sy)*dx)/max(length,1e-6)
                self.motion_check=dict(along_m=round(along,4),cross_m=round(across,4),
                    source='odom' if self.motion_odom_anchor else 'map',
                    costmap_stamp_ns=self.grid_stamp)
                if across>.10 or not -.10<=along<=length+.10:
                    self.failure = '平移偏離已檢查通道'
                    self.cancelTask()
                elif not self.grid.segment(pose,self.stage):
                    self.motion_check.update(blocker=self._translation_blocker(pose,self.stage))
                    self._pause_translation()
        align_physical=(self._physical_pose(self.align_anchor)
                        if self.phase=='aligning' and self.align_anchor else None)
        if (self.active_goal and not completed and self.phase=='aligning' and not self.cancelled
                and (pose is None or not data_ok or
                     align_physical is None or
                     math.hypot(align_physical[0]-self.align_anchor[0],
                                align_physical[1]-self.align_anchor[1])>.05
                     or not self.grid.turn_clear(pose,self.stage[2]))):
            self.failure = '對正位置或資料未通過檢查'
            self.data_unavailable = pose is None or not data_ok or align_physical is None
            self.cancelTask()
        align_physical=(self._physical_pose(self.align_anchor)
                        if self.phase=='aligning' and self.align_anchor else None)
        if (self.active_goal and not completed and self.phase=='aligning' and not self.cancelled
                and self._alignment_stalled()):
            self.motion_stalled=True
            self.failure='對正命令無有效輪速進展，停止補停'
            self.cancelTask()
        if not self.cancelled and not self.permitted():
            self.cancelTask()
        if (not self.cancelled and hasattr(self.nav,'navigation_actions_ready')
                and not self.nav.navigation_actions_ready()):
            self.failure,self.data_unavailable='導航回應異常，等待導航恢復',True
            self.cancelTask()
        if (self.active_goal and not completed and not self.cancelled and not self.replan_pending
                and pose is not None and data_ok):
            if self._finish_heading_stalled(pose):
                self.final_heading_pending=True
                self.mode,self.mode_reason,self.mode_switches='station','final_heading_stopped',1
                self.replan_pending=True
                self.nav.cancelTask()
            elif self._normal_plan_unsafe(pose):
                self._request_station_fallback()
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
                if (hasattr(self.nav,'navigation_actions_ready')
                        and not self.nav.navigation_actions_ready()):
                    self.failure,self.data_unavailable='導航回應異常，等待導航恢復',True
                elif (result==TaskResult.FAILED and not self.cancelled and not self.replan_pending
                        and self.native_error==(0,'')
                        and hasattr(self.nav,'navigation_actions_ready')):
                    # Native ACK timeout can return FAILED with error_code=0.
                    # Let the bounded stderr observer report the actual fault.
                    self.pending_unknown_failure=(result,time.monotonic()+1.5)
                    self._report('waiting_action_health')
                    return False
            if (not self.cancelled and self.mode=='normal' and not self.mode_switches
                    and result==TaskResult.FAILED and self._blocked_failure(*self.native_error)):
                self.mode,self.mode_reason,self.mode_switches='station','normal_route_blocked',1
                self.replan_pending=True
            if self.replan_pending and not self.cancelled:
                self.replan_pending=False
                self.stage=None
                self.after_stop,self.stop_since=('translation_clearance' if self.clearance_pending else
                    'final_align' if self.final_heading_pending else
                    'approach' if self.mode_switch_scope=='destination' else 'prepare'),None
                self.deadline=min(self.sequence_deadline,time.monotonic()+6.)
                self._report('settling','通道受阻，確認停妥後重驗' if self.clearance_pending
                             else '確認停妥後改用進出站模式')
            elif self.cancelled or result != TaskResult.SUCCEEDED:
                self._finish(TaskResult.FAILED if self.failure else TaskResult.CANCELED if self.cancelled else result,
                             'failed' if self.failure or not self.cancelled else 'cancelled',self.failure)
                return True
            else:
                self.after_stop,self.stop_since = self.next_phase,None
                self.deadline = min(self.sequence_deadline,time.monotonic()+6.)
                self._report('settling')
        if self.cancelled or not self.permitted():
            self._finish(TaskResult.FAILED if self.failure else TaskResult.CANCELED,
                         'failed' if self.failure else 'cancelled',self.failure); return True
        stopped = self._stopped()
        if self.phase=='waiting_clearance':
            self._waiting(pose,data_ok,stopped)
            if (pose is not None and data_ok and stopped
                    and self.grid_stamp!=self.clearance_grid_stamp):
                self.clearance_grid_stamp=self.grid_stamp
                self.after_stop=self.clearance_resume
                self._advance(pose)
            return not self.running
        intermediate = self.after_stop in ('entry','approach','prepare','translation_clearance')
        position_ok = self.after_stop=='translation_clearance' or self.stage is None or (pose is not None
            and math.hypot(pose[0]-self.stage[0],pose[1]-self.stage[1])<=(.20 if intermediate else .10)
            and (intermediate or abs(angle_error(pose[2],self.stage[2]))<=.10))
        if (not position_ok and self.after_stop=='finish' and self.park and self.final_corrections<1
                and pose is not None and data_ok and stopped
                and math.hypot(pose[0]-self.target[0],pose[1]-self.target[1])<=.20):
            self.stop_since = time.monotonic() if self.stop_since is None else self.stop_since
            if time.monotonic()-self.stop_since>=.5:
                self.final_corrections+=1
                if abs(angle_error(pose[2],self.target[2]))>.10:
                    self._align(pose,'final_translation')
                else:self._translation(pose,self.target,'entry','finish')
            return not self.running
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
