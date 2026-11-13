"""Observe navigation and recover diagnosed delivery-owned action faults safely."""
import json
import time
from collections import deque
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from nav2_msgs.srv import ManageLifecycleNodes
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatusArray
from bond.msg import Status
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from smart_delivery_core.pipeline_health import ReceiptMetrics, timed_operation
from smart_delivery_core.navigation_action_health import ActionRecovery
from smart_delivery_core.serial_recovery import fresh_feedback

TARGETS = ('map_server','amcl','planner_server','controller_server','bt_navigator','behavior_server','collision_monitor')


def navigation_snapshot(records, now, *, fresh_safety=False):
    states = {name: dict(record, age_sec=max(0.,now-record.get('at',now)))
              for name,record in records.items()}
    ready = len(states)==len(TARGETS) and all(
        p.get('id')==3 and 0 <= now-p.get('at',-1e6) <= 3. for p in states.values())
    inactive = [name for name,p in states.items() if p.get('id') != 3]
    missing = [name for name in TARGETS if name not in states]
    stale = [name for name,p in states.items() if not 0 <= now-p.get('at',-1e6) <= 3.]
    grace = (fresh_safety and not inactive and not missing and stale == ['map_server']
             and 3. < states['map_server']['age_sec'] <= 4.5)
    ready = ready or grace
    reason = ('late_map_server_response' if grace else 'lifecycle_inactive' if inactive else 'awaiting_responses' if missing
              else 'response_stale' if stale else 'ready')
    return dict(state='ready' if ready else 'unavailable', nodes=states, reason=reason,
                inactive_nodes=inactive, missing_nodes=missing, stale_nodes=stale,
                response_grace=bool(grace))


class NavigationHealth(Node):
    def __init__(self):
        super().__init__('navigation_health')
        self.declare_parameter('action_health_file','')
        self.action=ActionRecovery(self.get_parameter('action_health_file').value)
        self.manage=self.create_client(ManageLifecycleNodes,'/lifecycle_manager_navigation/manage_nodes')
        self.lease_active=None
        self.lease_windows=deque(maxlen=8)
        self.lease_start=None
        self.owned_fault=False
        self.feedback,self.feedback_at={},-float('inf')
        self.active_actions={}
        self.safety_receipts = {}
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
            lambda m:self._safety_sample('amcl', m), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom',
            lambda m:self._safety_sample('odom', m), qos_profile_sensor_data)
        state_qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool,'/smart_carrier/delivery_navigation_active',
            self._lease,state_qos)
        self.create_subscription(String,'/chassis/serial_rx_health',self._feedback,qos_profile_sensor_data)
        for action in ('navigate_to_pose','navigate_through_poses','spin'):
            self.create_subscription(GoalStatusArray,'/'+action+'/_action/status',
                lambda m,a=action:self.active_actions.update({a:any(s.status in (1,2,3) for s in m.status_list)}),
                state_qos)
        self.create_service(Trigger,'/navigation/recover_actions',self._request_recovery)
        self.last_action_probe=-float('inf')
        self.health_clients = {name:self.create_client(GetState,'/'+name+'/get_state') for name in TARGETS}
        self.pending, self.records = {}, {}
        self.timeouts = {name:0 for name in TARGETS}
        self.errors = {name:0 for name in TARGETS}
        self.response_max = {name:0. for name in TARGETS}
        self.receipts = ReceiptMetrics()
        self.bonds = {}
        self.last_probe = self.last_publish = -float('inf')
        self.publisher = self.create_publisher(String,'/navigation/pipeline_health',1)
        self.detail_publisher = self.create_publisher(String,'/navigation/diagnostics',
            QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.last_detail = -float('inf')
        # Raw bond/TF are already in rosbag. Do not deserialize every high-rate
        # transport sample in another Python process during normal operation.
        self.create_timer(.1,self.poll)

    def _feedback(self, message):
        try:
            payload=json.loads(message.data)
            self.feedback,self.feedback_at=(payload if isinstance(payload,dict) else {}),time.monotonic()
        except (ValueError,TypeError):self.feedback={}

    def _safety_sample(self, name, message):
        stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        previous = self.safety_receipts.get(name)
        if previous is None or stamp > previous[1]:
            self.safety_receipts[name] = (time.monotonic(), stamp)

    def _fresh_safety(self, now):
        ros_now = self.get_clock().now().nanoseconds
        for name, limit in (('amcl', .5), ('odom', .25)):
            receipt = self.safety_receipts.get(name)
            if receipt is None or not (0 <= now-receipt[0] <= limit
                    and -.1 <= (ros_now-receipt[1])/1e9 <= limit):
                return False
        return fresh_feedback(self.feedback, ros_now, now-self.feedback_at)

    def _lease(self, message):
        now=time.time()
        if message.data and self.lease_active is not True:self.lease_start=now
        if not message.data and self.lease_active is True:
            self.lease_windows.append((self.lease_start,now))
            self.lease_start=None
        self.lease_active=bool(message.data)

    def _observe_action(self):
        before=self.action.sequence
        self.action.observe()
        if self.action.sequence!=before:
            at=self.action.fault_wall_time or 0.
            windows=list(self.lease_windows)
            if self.lease_start is not None:windows.append((self.lease_start,float('inf')))
            self.owned_fault=any(start-.2<=at<=end+.2 for start,end in windows)

    def _stopped_for_recovery(self, now):
        return (self.lease_active is False and not any(self.active_actions.values())
            and self.feedback.get('stop_proven') is True
            and fresh_feedback(self.feedback,self.get_clock().now().nanoseconds,now-self.feedback_at))

    def _request_recovery(self, request, response):
        self._observe_action()
        if not self.owned_fault and self.action.state=='fault':
            self.action.state,self.action.reason='manual_required','non_delivery_fault_requires_operator'
        response.success=self.owned_fault and self.action.request(time.monotonic())
        response.message=('導航恢復已排入，等待停車證據' if response.success
                          else '沒有可恢復的 action 故障，或已達恢復上限')
        return response

    def _manage(self, command):
        request=ManageLifecycleNodes.Request();request.command=command
        return self.manage.call_async(request)

    def observe_bond(self, message):
        if message.id not in TARGETS:
            return
        key = (message.id,message.instance_id)
        if key not in self.bonds and len(self.bonds) >= 32:
            oldest = min(self.bonds,key=lambda k:self.bonds[k]['at'])
            del self.bonds[oldest]
        now = time.monotonic()
        stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
        previous = self.bonds.get(key,{})
        self.bonds[key] = dict(at=now,stamp=stamp,count=previous.get('count',0)+1,
            max_receive_gap_sec=max(previous.get('max_receive_gap_sec',0.),now-previous.get('at',now)),
            max_source_gap_sec=max(previous.get('max_source_gap_sec',0.),stamp-previous.get('stamp',stamp)),
            active=bool(message.active))

    def observe_tf(self, message):
        now = time.monotonic()
        for tf in message.transforms:
            pair = (tf.header.frame_id.lstrip('/'),tf.child_frame_id.lstrip('/'))
            if pair in (('map','odom'),('odom','base_footprint')):
                age = (self.get_clock().now().nanoseconds-
                       (tf.header.stamp.sec*1_000_000_000+tf.header.stamp.nanosec))/1e9
                self.receipts.observe('->'.join(pair),now,age)

    @timed_operation('poll')
    def poll(self):
        now = time.monotonic()
        for name,(future,start) in list(self.pending.items()):
            if future.done():
                try:
                    result = future.result().current_state
                    self.records[name] = dict(id=result.id,label=result.label,at=now,
                                              response_sec=now-start)
                    self.response_max[name] = max(self.response_max[name],now-start)
                except Exception:
                    self.errors[name] += 1
                self.pending.pop(name)
            elif now-start>1.25:
                self.timeouts[name] += 1
                future.cancel()
                self.health_clients[name].remove_pending_request(future)
                self.pending.pop(name)
        if now-self.last_probe>=1.:
            self.last_probe=now
            for name,client in self.health_clients.items():
                if name not in self.pending and client.service_is_ready():
                    self.pending[name]=(client.call_async(GetState.Request()),now)
        # The file changes only on native action faults; one stat per second,
        # rather than another high-rate rosout/TF parser or polling ROS CLI.
        if now-self.last_action_probe>=1.:
            self.last_action_probe=now
            self._observe_action()
        if self.action.state in ('waiting_stop','resetting','waiting_start','starting','verifying'):
            snapshot=navigation_snapshot(self.records,now)
            recent=all(p.get('at',0)>=getattr(self.action,'verification_since',float('inf'))
                       for p in self.records.values())
            self.action.tick(now,stopped=self._stopped_for_recovery(now),
                lifecycle_ready=snapshot['state']=='ready' and recent,
                service_ready=self.manage.service_is_ready(),call=self._manage)
        if now-self.last_publish>=1.:
            self.last_publish=now
            payload=navigation_snapshot(self.records,now,fresh_safety=self._fresh_safety(now))
            payload['sample_ros_stamp_ns']=self.get_clock().now().nanoseconds
            payload['diagnostic_only']=True
            payload['action']=self.action.snapshot()
            payload['action']['delivery_owned_fault']=self.owned_fault
            self.publisher.publish(String(data=json.dumps(payload)))
        if now-self.last_detail>=5.:
            self.last_detail=now
            payload=dict(sample_ros_stamp_ns=self.get_clock().now().nanoseconds)
            payload['service_diagnostics']=dict(timeouts=dict(self.timeouts),errors=dict(self.errors),
                response_max_sec=dict(self.response_max),pending_age_sec={n:now-start for n,(_,start) in self.pending.items()})
            payload['bonds']=[dict(node=name,instance=instance,age_sec=now-record['at'],**record)
                              for (name,instance),record in self.bonds.items()]
            payload['tf_receipts']=self.receipts.snapshot(now)
            payload['wall_durations']=self.duration_metrics.snapshot(now)
            self.detail_publisher.publish(String(data=json.dumps(payload)))


def main(args=None):
    rclpy.init(args=args);node=NavigationHealth()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
