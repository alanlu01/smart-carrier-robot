"""Read-only map fingerprint helper; never changes settings or sends goals."""

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from smart_delivery_core.optional_idle import occupancy_fingerprint


def main(args=None):
    rclpy.init(args=args)
    node = Node('optional_idle_map_inspect')
    received = []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, '/map', received.append, qos)
    try:
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.5)
            if received:
                print('occupancy_fingerprint:', occupancy_fingerprint(received[-1]), flush=True)
                print('Read-only. No poses/settings changed and no goals sent.', flush=True)
                return
        raise RuntimeError('No map received within 10 seconds')
    finally:
        node.destroy_node()
        rclpy.shutdown()
