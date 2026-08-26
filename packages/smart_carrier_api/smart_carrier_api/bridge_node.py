import json
import os
import time

import rclpy
from power_monitor.power_status import payload_to_slots
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from smart_carrier_api.api_client import ApiError, SmartCarrierApi
from smart_carrier_api.bridge_store import BridgeStore

STATE_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class ApiBridgeNode(Node):
    """Durable bridge between cloud task APIs and local ROS topics."""

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
        self.declare_parameter("order_republish_interval", 5.0)

        api_url = str(self.get_parameter("api_url").value)
        robot_id = str(self.get_parameter("robot_id").value)
        token = str(self.get_parameter("robot_token").value)
        request_timeout = float(self.get_parameter("request_timeout").value)
        self.api = SmartCarrierApi(api_url, robot_id, token, timeout=request_timeout)
        self.store = BridgeStore()
        self.configured = bool(api_url and robot_id and token)

        recovered = self.store.get_active_task()
        self.active_task = recovered
        self.active_task_id = str(recovered["id"]) if recovered else None
        self.delivery_confirmed_task_id = None
        self.last_order_publish_at = 0.0
        self.last_error_at = 0.0
        self.slots = []
        self.power_healthy = False
        self.power_received = False
        self.heartbeat_confirmed = False
        self.cancel_notified_task_id = None

        self.order_publisher = self.create_publisher(String, "order", STATE_QOS)
        self.cancel_publisher = self.create_publisher(
            String, "/smart_carrier/task_cancel", STATE_QOS
        )
        self.result_ack_publisher = self.create_publisher(
            String, "/smart_carrier/task_result_ack", STATE_QOS
        )
        self.create_subscription(String, "power_status", self.on_power_status, 10)
        self.create_subscription(
            String, "/smart_carrier/task_result", self.on_task_result, STATE_QOS
        )
        self.create_subscription(String, "/smart_carrier/task_state", self.on_task_state, STATE_QOS)
        self.create_timer(float(self.get_parameter("poll_interval").value), self.poll)
        self.create_timer(0.5, self.flush_result_outbox)
        self.create_timer(1.0, self.flush_progress)
        self.create_timer(
            float(self.get_parameter("heartbeat_interval").value), self.send_heartbeat
        )
        if not self.configured:
            self.get_logger().error(
                "API bridge 尚未完整設定；請載入 SMART_CARRIER_API_URL、"
                "SMART_CARRIER_ROBOT_ID、SMART_CARRIER_ROBOT_TOKEN"
            )
        else:
            self.get_logger().info(f"API bridge 已啟動：robot={robot_id}, api={api_url}")
            if recovered:
                self.get_logger().warning(
                    f"已復原尚未完成的任務 {self.active_task_id}，等待配送節點確認狀態"
                )

    def on_power_status(self, message):
        try:
            self.slots = payload_to_slots(message.data)
            self.power_healthy = all(
                (not slot.get("enabled", True))
                or (slot["sensor_ok"] and slot["status"] != "unknown")
                for slot in self.slots
            )
            if not self.power_received:
                self.get_logger().info("已收到 power_status，後續 heartbeat 將包含槽位狀態")
                self.power_received = True
        except ValueError as exc:
            self.power_healthy = False
            self.get_logger().error(str(exc))

    def publish_active_order(self):
        if not self.active_task:
            return
        message = String()
        message.data = json.dumps(self.active_task, ensure_ascii=False)
        self.order_publisher.publish(message)
        self.last_order_publish_at = time.monotonic()

    def poll(self):
        if not self.configured:
            return
        if self.active_task_id:
            self.poll_active_task()
            interval = float(self.get_parameter("order_republish_interval").value)
            if (
                self.delivery_confirmed_task_id != self.active_task_id
                and time.monotonic() - self.last_order_publish_at >= interval
            ):
                self.publish_active_order()
            return
        if self.store.has_pending_results() or not self.power_healthy:
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
            self.store.set_active_task(task)
            self.active_task = task
            self.active_task_id = str(task_id)
            self.delivery_confirmed_task_id = None
            self.publish_active_order()
            self.get_logger().info(f"已領取並保存雲端任務 {self.active_task_id}")
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

    def on_task_state(self, message):
        try:
            progress = json.loads(message.data)
            task_id = str(progress["task_id"])
            if task_id != self.active_task_id:
                return
            self.delivery_confirmed_task_id = task_id
            self.store.set_pending_progress(progress)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"忽略無效任務進度：{exc}")

    def flush_progress(self):
        if not self.configured:
            return
        progress = self.store.get_pending_progress()
        if not progress:
            return
        try:
            self.api.report_progress(str(progress["task_id"]), progress)
            self.store.set_pending_progress(None)
        except (ApiError, KeyError, TypeError) as exc:
            self.log_api_error(exc)

    def on_task_result(self, message):
        try:
            result = json.loads(message.data)
            task_id = str(result["task_id"])
            event_id = str(result["event_id"])
            status = str(result["status"])
            if status not in {"done", "failed", "cancelled", "released"}:
                raise ValueError(f"不支援的任務結果狀態：{status}")
            if self.active_task_id and task_id != self.active_task_id:
                self.get_logger().warning(
                    f"保存非目前任務結果：{task_id}（目前：{self.active_task_id}）"
                )
            self.store.enqueue_result(result)
            ack = String()
            ack.data = json.dumps({"event_id": event_id, "task_id": task_id})
            self.result_ack_publisher.publish(ack)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"忽略無效任務結果：{exc}")

    def flush_result_outbox(self):
        if not self.configured:
            return
        item = self.store.next_result()
        if not item:
            return
        try:
            if item["status"] == "released":
                self.api.release_task(item["task_id"], item["event_id"], item.get("note"))
            else:
                self.api.report_result(
                    item["task_id"], item["event_id"], item["status"], item.get("note")
                )
            self.store.mark_delivered(item["event_id"])
            if item["task_id"] == self.active_task_id:
                self.store.set_active_task(None)
                self.store.set_pending_progress(None)
                self.active_task = None
                self.active_task_id = None
                self.delivery_confirmed_task_id = None
                self.cancel_notified_task_id = None
            self.get_logger().info(f"雲端已確認任務結果 {item['task_id']}: {item['status']}")
        except ApiError as exc:
            self.store.mark_retry(item["event_id"], int(item["attempts"]))
            self.log_api_error(exc)

    def log_api_error(self, error):
        now = time.monotonic()
        if now - self.last_error_at >= 10.0:
            self.get_logger().error(str(error))
            self.last_error_at = now

    def destroy_node(self):
        self.store.close()
        super().destroy_node()


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
