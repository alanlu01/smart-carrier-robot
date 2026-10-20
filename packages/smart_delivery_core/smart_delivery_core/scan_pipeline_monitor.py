"""Read-only scan observer in a separate process; never gates or relays scans."""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from smart_delivery_core.pipeline_health import DurationMetrics
from smart_delivery_core.scan_pipeline_metrics import ScanPipelineMetrics


class ScanPipelineMonitor(Node):
    """Measure original header and DDS timestamps with shallow best-effort readers."""

    def __init__(self):
        super().__init__('scan_pipeline_monitor')
        self.metrics = ScanPipelineMetrics()
        self.durations = DurationMetrics()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.tf_available = None
        self.tf_checks = self.tf_unavailable = 0
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(String, '/lidar/pipeline_health',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(LaserScan, '/scan', self._raw, qos)
        self.create_subscription(LaserScan, '/scan_filtered', self._filtered, qos)
        self.create_timer(1., self._publish)

    def _observe(self, name, message, info):
        stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        # Jazzy supplies a MessageInfo TypedDict; some adapters expose attrs.
        source = info.get('source_timestamp', 0) if isinstance(info, dict) else getattr(info, 'source_timestamp', 0)
        received = info.get('received_timestamp', 0) if isinstance(info, dict) else getattr(info, 'received_timestamp', 0)
        self.metrics.observe(name, stamp, time.monotonic(), self.get_clock().now().nanoseconds,
            source, received)
        if name == 'raw':
            # Nonblocking availability only, at the original timestamp and the
            # actual self-mask frame; it is NOT the filter's internal wait time.
            with self.durations.measure('box_tf_availability_check'):
                try:
                    self.tf_available = bool(stamp > 0 and message.header.frame_id and self.buffer.can_transform(
                        'base_footprint', message.header.frame_id, Time(nanoseconds=stamp)))
                except (ValueError, TransformException):
                    self.tf_available = False
            self.tf_checks += 1
            self.tf_unavailable += int(not self.tf_available)

    def _raw(self, message, info):
        with self.durations.measure('raw_observer_callback'):
            self._observe('raw', message, info)

    def _filtered(self, message, info):
        with self.durations.measure('filtered_observer_callback'):
            self._observe('filtered', message, info)

    def _publish(self):
        now = time.monotonic()
        payload = self.metrics.snapshot(now)
        payload.update(sample_ros_stamp_ns=self.get_clock().now().nanoseconds,
            filter_box_tf_available=self.tf_available, filter_box_tf_checks=self.tf_checks,
            filter_box_tf_unavailable=self.tf_unavailable,
            observer_wall_durations=self.durations.snapshot(now))
        self.publisher.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def main(args=None):
    """Start only the diagnostic subscriber; no robot commands are published."""
    rclpy.init(args=args)
    node = ScanPipelineMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
