"""Relay fresh complete groups; hardware reads and diagnostics are isolated."""
import json
import multiprocessing
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from smart_delivery_core.odom_transport import LOCAL_ODOM_QOS
from smart_delivery_core.pipeline_health import DurationMetrics
from smart_delivery_core.serial_mailbox import LatestMailbox
from smart_delivery_core.serial_workers import receive_worker, diagnostic_worker, stop_workers


class SerialToROS(Node):
    def __init__(self):
        super().__init__('serial_to_ros')
        self.declare_parameter('serial_port', '/dev/stm32')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('summary_log_period_sec', 5.)
        self.declare_parameter('log_raw_frames', False)
        ctx = multiprocessing.get_context('spawn')
        self.stop = ctx.Event()
        self.wheel, self.health, self.raw, self.relay = [LatestMailbox(ctx) for _ in range(4)]
        self.control = LatestMailbox(ctx, capacity=4096)
        self.control_pub = self.create_publisher(String, '/chassis/serial_feedback', LOCAL_ODOM_QOS)
        self.sample_pub = self.create_publisher(String, '/chassis/wheel_sample', LOCAL_ODOM_QOS)
        self.health_pub = self.create_publisher(String, '/chassis/serial_rx_health', LOCAL_ODOM_QOS)
        self.metrics = DurationMetrics()
        self.last_wheel = self.last_health = 0
        self.last_control = 0
        self.last_relay_at = 0.
        self.dropped_stale = 0
        self.workers = [
            ctx.Process(target=receive_worker, args=(str(self.get_parameter('serial_port').value),
                int(self.get_parameter('baud_rate').value), self.stop, self.wheel, self.health, self.raw,
                self.control), daemon=True),
            ctx.Process(target=diagnostic_worker, args=('rx', self.stop, self.health, self.relay, self.raw,
                max(1., float(self.get_parameter('summary_log_period_sec').value))), daemon=True),
        ]
        for worker in self.workers:
            worker.start()
        self.timer = self.create_timer(.05, self.relay_samples)

    def relay_samples(self):
        started = time.monotonic()
        for slot, attribute, publisher, limit, operation in (
            (self.control, 'last_control', self.control_pub, .35, 'control_feedback_publish'),
            (self.wheel, 'last_wheel', self.sample_pub, .65, 'wheel_publish'),
            (self.health, 'last_health', self.health_pub, .35, 'rx_health_publish'),
        ):
            item = slot.get(getattr(self, attribute))
            if not item:
                continue
            version, payload = item
            setattr(self, attribute, version)
            age = (self.get_clock().now().nanoseconds-payload['sample_ros_stamp_ns'])/1e9
            if not -.1 <= age <= limit:
                self.dropped_stale += 1
                continue
            with self.metrics.measure(operation):
                publisher.publish(String(data=json.dumps(payload)))
        if started-self.last_relay_at >= .2:
            self.relay.put(dict(sample_ros_stamp_ns=self.get_clock().now().nanoseconds,
                                wall_durations=self.metrics.snapshot(time.monotonic()),
                                stale_snapshots_dropped=self.dropped_stale,
                                io_alive=self.workers[0].is_alive()))
            self.last_relay_at = started

    def destroy_node(self):
        stop_workers(self.stop, self.workers)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialToROS()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
