"""Isolate public DDS writes from wheel feedback/integration in another process."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from smart_delivery_core.pipeline_health import DurationMetrics

LOCAL_ODOM_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


class OdomTransport(Node):
    """Keep /odom and dynamic TF reliable, drop old local snapshots, never restamp."""
    MAX_AGE = .20

    def __init__(self):
        super().__init__('odom_transport')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.health_pub = self.create_publisher(String, '/chassis/odometry_transport_health', LOCAL_ODOM_QOS)
        self.metrics = DurationMetrics()
        self.received = self.published = self.dropped = self.errors = 0
        self.last_source_ns = 0
        self.last_receive_at = self.last_publish_at = 0.
        self.create_subscription(Odometry, '/chassis/odom_local', self.receive, LOCAL_ODOM_QOS)
        self.create_timer(1., self.health)

    def fresh(self, message):
        ns = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-ns)/1e9
        p,q,v = message.pose.pose.position,message.pose.pose.orientation,message.twist.twist
        valid = all(math.isfinite(x) for x in (p.x,p.y,p.z,q.x,q.y,q.z,q.w,
            v.linear.x,v.linear.y,v.angular.z)) and abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) < .01
        return (ns>0 and -.1<=age<=self.MAX_AGE and valid
                and message.header.frame_id=='odom' and message.child_frame_id=='base_footprint')

    def receive(self, message):
        self.received += 1
        self.last_receive_at = time.monotonic()
        ns = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        if not self.fresh(message) or ns<=self.last_source_ns:
            self.dropped += 1
            return
        self.last_source_ns = ns
        tf = TransformStamped()
        tf.header = message.header
        tf.child_frame_id = message.child_frame_id
        tf.transform.translation.x = message.pose.pose.position.x
        tf.transform.translation.y = message.pose.pose.position.y
        tf.transform.translation.z = message.pose.pose.position.z
        tf.transform.rotation = message.pose.pose.orientation
        try:
            with self.metrics.measure('tf_publish'):
                self.tf.sendTransform(tf)
            # A stalled writer cannot cause the paired odom to masquerade as new.
            if not self.fresh(message):
                self.dropped += 1
                return
            with self.metrics.measure('odom_publish'):
                self.odom_pub.publish(message)
            self.published += 1
            self.last_publish_at = time.monotonic()
        except Exception as exc:
            self.errors += 1
            if self.errors==1 or self.errors%100==0:
                self.get_logger().error('Odometry transport write failed: '+str(exc))

    def health(self):
        now = time.monotonic()
        age = (self.get_clock().now().nanoseconds-self.last_source_ns)/1e9 if self.last_source_ns else None
        msg = String()
        healthy = (age is not None and -.1<=age<=self.MAX_AGE
                   and self.last_publish_at>0 and now-self.last_publish_at<=.30)
        msg.data = json.dumps({'state':'healthy' if healthy else 'stale',
            'sample_ros_stamp_ns':self.get_clock().now().nanoseconds,
            'last_receive_age_sec':now-self.last_receive_at if self.last_receive_at else None,
            'last_publish_age_sec':now-self.last_publish_at if self.last_publish_at else None,
            'source_age_sec':age,'received':self.received,'published':self.published,
            'dropped':self.dropped,'write_errors':self.errors,'wall_durations':self.metrics.snapshot(now)})
        try:
            self.health_pub.publish(msg)
        except Exception:
            self.errors += 1


def main():
    rclpy.init()
    node = OdomTransport()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
