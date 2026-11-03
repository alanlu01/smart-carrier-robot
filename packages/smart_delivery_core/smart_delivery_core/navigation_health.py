"""Observe lifecycle services without managing or restarting navigation."""
import json
import time
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState
from std_msgs.msg import String

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
        self.last_probe = self.last_publish = -float('inf')
        self.publisher = self.create_publisher(String,'/navigation/pipeline_health',1)
        self.create_timer(.1,self.poll)

    def poll(self):
        now = time.monotonic()
        for name,(future,start) in list(self.pending.items()):
            if future.done():
                try:
                    result = future.result().current_state
                    self.records[name] = dict(id=result.id,label=result.label,at=now,
                                              response_sec=now-start)
                except Exception:
                    pass
                self.pending.pop(name)
            elif now-start>.75:
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


def main(args=None):
    rclpy.init(args=args);node=NavigationHealth()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
