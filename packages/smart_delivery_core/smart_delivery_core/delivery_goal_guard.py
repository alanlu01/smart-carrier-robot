"""Cancel delivery-owned Nav2 goals when the delivery process disappears."""

import time

import rclpy
from action_msgs.srv import CancelGoal
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from smart_delivery_core.mission_lease import MissionLeaseMonitor


LEASE_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class DeliveryGoalGuard(Node):
    """Cancel orphaned delivery goals without affecting unrelated Nav2 use."""

    def __init__(self):
        """Initialize the heartbeat subscriber and Nav2 cancel client."""
        super().__init__('delivery_goal_guard')
        self.declare_parameter('heartbeat_timeout_sec', 1.0)
        timeout = float(self.get_parameter('heartbeat_timeout_sec').value)
        self.lease = MissionLeaseMonitor(timeout)
        self.cancel_pending = False
        self.navigate_cancel_client = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal'
        )
        self.create_subscription(
            Bool,
            '/smart_carrier/delivery_navigation_active',
            self._lease_callback,
            LEASE_QOS,
        )
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f'配送導航租約監看已啟動（逾時 {timeout:.1f} 秒即取消目標）'
        )

    def _lease_callback(self, message):
        active = bool(message.data)
        self.lease.observe(active, time.monotonic())
        if not active:
            self.cancel_pending = False

    def _tick(self):
        if self.cancel_pending or not self.lease.has_expired(time.monotonic()):
            return
        if not self.navigate_cancel_client.service_is_ready():
            return
        self.cancel_pending = True
        self.get_logger().error(
            'smart_delivery 導航 heartbeat 逾時，正在取消殘留 Nav2 目標'
        )
        future = self.navigate_cancel_client.call_async(CancelGoal.Request())
        future.add_done_callback(self._cancel_done)

    def _cancel_done(self, future):
        self.cancel_pending = False
        try:
            response = future.result()
        except Exception as exc:  # middleware errors must keep the guard armed
            self.get_logger().error(f'取消殘留 Nav2 目標失敗，將重試：{exc}')
            return
        terminal_codes = {
            CancelGoal.Response.ERROR_NONE,
            CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID,
            CancelGoal.Response.ERROR_GOAL_TERMINATED,
        }
        if response.return_code in terminal_codes:
            self.get_logger().warning('已取消 smart_delivery 遺留的 Nav2 目標')
            self.lease.disarm()
        else:
            self.get_logger().warning(
                f'Nav2 取消要求未成功（return_code={response.return_code}），將重試'
            )


def main(args=None):
    """Run the delivery goal guard node."""
    rclpy.init(args=args)
    node = DeliveryGoalGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
