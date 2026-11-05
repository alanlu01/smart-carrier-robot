"""Observe lifecycle services without managing or restarting navigation."""
import json
import time
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState
from std_msgs.msg import String
from bond.msg import Status
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, ReliabilityPolicy
from smart_delivery_core.pipeline_health import ReceiptMetrics, timed_operation

TARGETS = ('map_server','amcl','controller_server','bt_navigator','behavior_server','collision_monitor')


def navigation_snapshot(records, now):
    states = {name: dict(record, age_sec=max(0.,now-record.get('at',now)))
              for name,record in records.items()}
    ready = len(states)==len(TARGETS) and all(
        p.get('id')==3 and 0 <= now-p.get('at',-1e6) <= 3. for p in states.values())
    return dict(state='ready' if ready else 'unavailable', nodes=states)


class NavigationHealth(Node):
    def __init__(self):
        super().__init__('navigation_health')
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
            elif now-start>.75:
                self.timeouts[name] += 1
                future.cancel()
                self.health_clients[name].remove_pending_request(future)
                self.pending.pop(name)
        if now-self.last_probe>=1.:
            self.last_probe=now
            for name,client in self.health_clients.items():
                if name not in self.pending and client.service_is_ready():
                    self.pending[name]=(client.call_async(GetState.Request()),now)
        if now-self.last_publish>=1.:
            self.last_publish=now
            payload=navigation_snapshot(self.records,now)
            payload['sample_ros_stamp_ns']=self.get_clock().now().nanoseconds
            payload['diagnostic_only']=True
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
