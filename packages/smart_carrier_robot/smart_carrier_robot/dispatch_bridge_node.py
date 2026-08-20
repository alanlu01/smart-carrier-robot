import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from smart_carrier_robot.api_client import ApiError, SmartCarrierApi
from smart_carrier_robot.power_status import payload_to_slots


class DispatchBridgeNode(Node):
    def __init__(self):
        super().__init__("dispatch_bridge_node")
        self.declare_parameter(
            "api_url", os.getenv("SMART_CARRIER_API_URL", "http://127.0.0.1:8000")
        )
        self.declare_parameter("robot_id", os.getenv("SMART_CARRIER_ROBOT_ID", "R1"))
        self.declare_parameter("robot_token", os.getenv("SMART_CARRIER_ROBOT_TOKEN", ""))
        self.declare_parameter("poll_interval", 2.0)
        self.declare_parameter("heartbeat_interval", 5.0)

        api_url = self.get_parameter("api_url").value
        robot_id = self.get_parameter("robot_id").value
        token = self.get_parameter("robot_token").value
        self.api = SmartCarrierApi(api_url, robot_id, token)
        self.task_publisher = self.create_publisher(String, "/smart_carrier/task", 10)
        self.create_subscription(String, "power_status", self.on_power_status, 10)
        self.create_subscription(String, "/smart_carrier/task_result", self.on_task_result, 10)
        self.create_timer(self.get_parameter("poll_interval").value, self.poll)
        self.create_timer(self.get_parameter("heartbeat_interval").value, self.send_heartbeat)
        self.slots = []
        self.active_task_id = None
        self.last_error_at = 0.0
        if not token:
            self.get_logger().warning(
                "SMART_CARRIER_ROBOT_TOKEN is empty; cloud requests will fail"
            )

    def on_power_status(self, message: String) -> None:
        try:
            self.slots = payload_to_slots(message.data)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().error(f"Invalid power_status payload: {exc}")

    def poll(self) -> None:
        if self.active_task_id:
            return
        try:
            task = self.api.claim_task()
            if not task:
                return
            self.active_task_id = task["id"]
            message = String()
            message.data = json.dumps(task)
            self.task_publisher.publish(message)
            self.get_logger().info(f"Claimed task {self.active_task_id}")
        except ApiError as exc:
            self.log_api_error(exc)

    def send_heartbeat(self) -> None:
        try:
            self.api.heartbeat(
                {
                    "mode": "dispatching" if self.active_task_id else "idle",
                    "slots": self.slots,
                }
            )
        except ApiError as exc:
            self.log_api_error(exc)

    def on_task_result(self, message: String) -> None:
        try:
            result = json.loads(message.data)
            task_id = result["task_id"]
            self.api.report_result(task_id, result["status"], result.get("note"))
            if task_id == self.active_task_id:
                self.active_task_id = None
        except (ApiError, KeyError, json.JSONDecodeError) as exc:
            self.log_api_error(exc)

    def log_api_error(self, error: Exception) -> None:
        now = time.monotonic()
        if now - self.last_error_at >= 10:
            self.get_logger().error(str(error))
            self.last_error_at = now


def main(args=None):
    rclpy.init(args=args)
    node = DispatchBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
