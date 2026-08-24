import json
import os
import time

import rclpy
from power_monitor.power_status import payload_to_slots
from rclpy.node import Node
from std_msgs.msg import String

from smart_carrier_api.api_client import ApiError, SmartCarrierApi


class ApiBridgeNode(Node):
    """Translate cloud tasks and robot state without owning hardware or navigation."""

    def __init__(self):
        super().__init__("smart_carrier_api_bridge")
        self.declare_parameter(
            "api_url", os.getenv("SMART_CARRIER_API_URL", "http://127.0.0.1:8000")
        )
        self.declare_parameter("robot_id", os.getenv("SMART_CARRIER_ROBOT_ID", "R1"))
        self.declare_parameter("robot_token", os.getenv("SMART_CARRIER_ROBOT_TOKEN", ""))
        self.declare_parameter("poll_interval", 2.0)
        self.declare_parameter("heartbeat_interval", 5.0)
        self.declare_parameter("request_timeout", 5.0)

        api_url = str(self.get_parameter("api_url").value)
        robot_id = str(self.get_parameter("robot_id").value)
        token = str(self.get_parameter("robot_token").value)
        request_timeout = float(self.get_parameter("request_timeout").value)
        self.api = SmartCarrierApi(api_url, robot_id, token, timeout=request_timeout)
        self.configured = bool(api_url and robot_id and token)

        self.order_publisher = self.create_publisher(String, "order", 10)
        self.cancel_publisher = self.create_publisher(String, "/smart_carrier/task_cancel", 10)
        self.create_subscription(String, "power_status", self.on_power_status, 10)
        self.create_subscription(String, "/smart_carrier/task_result", self.on_task_result, 10)
        self.create_timer(float(self.get_parameter("poll_interval").value), self.poll)
        self.create_timer(
            float(self.get_parameter("heartbeat_interval").value), self.send_heartbeat
        )

        self.slots = []
        self.power_healthy = False
        self.power_received = False
        self.heartbeat_confirmed = False
        self.active_task_id = None
        self.cancel_notified_task_id = None
        self.last_error_at = 0.0
        if not self.configured:
            self.get_logger().error(
                "API bridge 尚未完整設定；請載入 SMART_CARRIER_API_URL、"
                "SMART_CARRIER_ROBOT_ID、SMART_CARRIER_ROBOT_TOKEN"
            )
        else:
            self.get_logger().info(f"API bridge 已啟動：robot={robot_id}, api={api_url}")

    def on_power_status(self, message):
        try:
            self.slots = payload_to_slots(message.data)
            self.power_healthy = all(
                slot["sensor_ok"] and slot["status"] != "unknown" for slot in self.slots
            )
            if not self.power_received:
                self.get_logger().info("已收到 power_status，後續 heartbeat 將包含三個槽位")
                self.power_received = True
        except ValueError as exc:
            self.power_healthy = False
            self.get_logger().error(str(exc))

    def poll(self):
        if not self.configured:
            return
        if self.active_task_id:
            self.poll_active_task()
            return
        if not self.power_healthy:
            return
        if self.order_publisher.get_subscription_count() == 0:
            return
        try:
            task = self.api.claim_task(self.slots)
            if not task:
                return
            task_id = task.get("id")
            if not task_id:
                raise ApiError("API claimed task without an id")
            self.active_task_id = str(task_id)
            message = String()
            message.data = json.dumps(task, ensure_ascii=False)
            self.order_publisher.publish(message)
            self.get_logger().info(f"已領取雲端任務 {self.active_task_id}")
        except (ApiError, TypeError) as exc:
            self.log_api_error(exc)

    def poll_active_task(self):
        try:
            task = self.api.get_task(self.active_task_id)
            cancellation_requested = bool(task.get("cancel_requested_at"))
            if task.get("status") == "cancelled":
                cancellation_requested = True
            if not cancellation_requested or self.cancel_notified_task_id == self.active_task_id:
                return
            message = String()
            message.data = json.dumps(
                {
                    "task_id": self.active_task_id,
                    "reason": task.get("cancel_reason") or "Cancelled by administrator",
                },
                ensure_ascii=False,
            )
            self.cancel_publisher.publish(message)
            self.cancel_notified_task_id = self.active_task_id
            self.get_logger().warning(f"收到雲端取消要求：{self.active_task_id}")
        except (ApiError, AttributeError, TypeError) as exc:
            self.log_api_error(exc)

    def send_heartbeat(self):
        if not self.configured:
            return
        if self.active_task_id:
            mode = "dispatching"
        elif not self.power_healthy:
            mode = "power_sensor_error"
        else:
            mode = "idle"
        try:
            self.api.heartbeat({"mode": mode, "slots": self.slots})
            if not self.heartbeat_confirmed:
                self.get_logger().info("雲端 heartbeat 已成功")
                self.heartbeat_confirmed = True
        except ApiError as exc:
            self.log_api_error(exc)

    def on_task_result(self, message):
        if not self.configured:
            return
        try:
            result = json.loads(message.data)
            task_id = str(result["task_id"])
            if task_id != self.active_task_id:
                self.get_logger().warning(
                    f"忽略非目前任務的結果：{task_id}（目前：{self.active_task_id}）"
                )
                return
            status = str(result["status"])
            if status not in {"done", "failed", "cancelled", "released"}:
                raise ValueError(f"不支援的任務結果狀態：{status}")
            if status == "released":
                self.api.release_task(task_id, result.get("note"))
            else:
                self.api.report_result(task_id, status, result.get("note"))
            self.active_task_id = None
            self.cancel_notified_task_id = None
            self.get_logger().info(f"已更新雲端任務 {task_id}: {status}")
        except (ApiError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.log_api_error(exc)

    def log_api_error(self, error):
        now = time.monotonic()
        if now - self.last_error_at >= 10.0:
            self.get_logger().error(str(error))
            self.last_error_at = now


def main(args=None):
    rclpy.init(args=args)
    node = ApiBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
