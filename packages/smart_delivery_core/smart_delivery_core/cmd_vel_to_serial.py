"""Receive only latest commands; a ROS-free writer owns zero-speed watchdogs."""
import json
import math
import multiprocessing
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from smart_delivery_core.odom_transport import LOCAL_ODOM_QOS
from smart_delivery_core.pipeline_health import DurationMetrics
from smart_delivery_core.serial_io import ZERO
from smart_delivery_core.serial_mailbox import LatestMailbox
from smart_delivery_core.serial_workers import transmit_worker, diagnostic_worker, stop_workers


class CmdVelToSerialNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_serial_node')
        self.declare_parameter('serial_port', '/dev/stm32')
        self.declare_parameter('baud_rate', 115200)
        ctx = multiprocessing.get_context('spawn')
        self.stop = ctx.Event()
        self.commands, self.feedback, self.health, self.relay = [LatestMailbox(ctx) for _ in range(4)]
        self.metrics = DurationMetrics()
        self.subscription = self.create_subscription(Twist, '/chassis_cmd_vel', self.cmd_vel_callback, 1)
        self.create_subscription(String, '/chassis/serial_feedback', self.feedback_callback, LOCAL_ODOM_QOS)
        self.workers = [
            ctx.Process(target=transmit_worker, args=(str(self.get_parameter('serial_port').value),
                int(self.get_parameter('baud_rate').value), self.stop, self.commands, self.feedback, self.health), daemon=True),
            ctx.Process(target=diagnostic_worker, args=('tx', self.stop, self.health, self.relay), daemon=True),
        ]
        for worker in self.workers:
            worker.start()
        self.create_timer(1., self.update_metrics)

    def feedback_callback(self, message):
        with self.metrics.measure('feedback_callback'):
            try:
                payload = json.loads(message.data)
                if not isinstance(payload, dict):
                    return
                self.feedback.put(dict(payload=payload, received_at=time.monotonic()))
            except (ValueError, TypeError):
                pass

    def cmd_vel_callback(self, message, info):
        with self.metrics.measure('command_callback'):
            values = (message.linear.x, message.linear.y, message.angular.z)
            source = info.get('source_timestamp', 0) if isinstance(info, dict) else getattr(info, 'source_timestamp', 0)
            if all(math.isfinite(v) for v in values):
                self.commands.put(dict(values=values, source_stamp_ns=source,
                                       received_at=time.monotonic()))
            else:
                self.commands.put(dict(values=[0., 0., 0.], source_stamp_ns=0,
                                       received_at=time.monotonic()))

    def update_metrics(self):
        self.relay.put(dict(sample_ros_stamp_ns=self.get_clock().now().nanoseconds,
                            wall_durations=self.metrics.snapshot(time.monotonic()),
                            command_ipc_skips=self.commands.skipped.value,
                            feedback_ipc_skips=self.feedback.skipped.value,
                            io_alive=self.workers[0].is_alive()))

    def destroy_node(self):
        stop_workers(self.stop, self.workers)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSerialNode()
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
