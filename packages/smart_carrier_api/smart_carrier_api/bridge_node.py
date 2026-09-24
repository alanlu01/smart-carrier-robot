import json
import os
import time
import uuid
from concurrent.futures import Future, ThreadPoolExecutor

import rclpy
from power_monitor.power_status import payload_to_slots
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from smart_carrier_api.api_client import ApiError, SmartCarrierApi
from smart_carrier_api.bridge_store import BridgeStore, is_terminal_result_error
from smart_carrier_api.claim_queue import (
    ClaimProjectionError,
    project_claimed_slots,
    should_close_claim_batch,
    tasks_for_batch,
)

STATE_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
COMMAND_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class ApiBridgeNode(Node):
    """Durable, non-blocking bridge between cloud APIs and local ROS topics."""

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
        self.declare_parameter("cancel_republish_interval", 2.0)
        self.declare_parameter("reconcile_interval", 10.0)
        self.declare_parameter("max_claimed_tasks", 3)
        self.declare_parameter("batch_quiet_period", 1.0)
        self.declare_parameter("batch_max_wait", 5.0)
        self.declare_parameter("batch_claim_retry_interval", 0.25)

        api_url = str(self.get_parameter("api_url").value)
        robot_id = str(self.get_parameter("robot_id").value)
        token = str(self.get_parameter("robot_token").value)
        request_timeout = float(self.get_parameter("request_timeout").value)
        self.api = SmartCarrierApi(api_url, robot_id, token, timeout=request_timeout)
        self.store = BridgeStore()
        self.configured = bool(api_url and robot_id and token)
        self.max_claimed_tasks = int(self.get_parameter("max_claimed_tasks").value)
        if self.max_claimed_tasks < 1:
            raise ValueError("max_claimed_tasks 必須至少為 1")
        self.batch_quiet_period = float(self.get_parameter("batch_quiet_period").value)
        self.batch_max_wait = float(self.get_parameter("batch_max_wait").value)
        self.batch_claim_retry_interval = float(
            self.get_parameter("batch_claim_retry_interval").value
        )
        if self.batch_quiet_period < 0 or self.batch_max_wait <= 0:
            raise ValueError("批次收集時間參數必須為正值")
        if self.batch_claim_retry_interval <= 0:
            raise ValueError("批次 claim 重試間隔必須大於 0")

        self.claimed_tasks = self.store.get_claimed_tasks()
        self.active_batch = self.store.get_active_batch()
        self.last_batch_publish_at = 0.0
        self.collection_started_at: float | None = None
        self.collection_empty_since: float | None = None
        self.last_cancel_publish_at = {}
        self.last_error_at = 0.0
        self.slots = []
        self.power_healthy = False
        self.power_received = False
        self.heartbeat_confirmed = False

        # Only this worker performs HTTP. SQLite and ROS publishers stay on the
        # executor thread, so a slow network can never starve local callbacks.
        self.network = ThreadPoolExecutor(max_workers=1, thread_name_prefix="carrier-api")
        self.network_future: Future | None = None
        self.network_action: tuple[str, dict] | None = None
        self.network_online: bool | None = None
        self.reconciliation_supported = True
        self.reconciled = False
        now = time.monotonic()
        self.next_claim_at = now
        self.next_heartbeat_at = now
        self.next_reconcile_at = now
        self.next_progress_at = now
        self.next_task_poll_at: dict[str, float] = {}

        self.order_publisher = self.create_publisher(String, "order", COMMAND_QOS)
        self.cancel_publisher = self.create_publisher(
            String, "/smart_carrier/task_cancel", COMMAND_QOS
        )
        self.result_ack_publisher = self.create_publisher(
            String, "/smart_carrier/task_result_ack", STATE_QOS
        )
        self.create_subscription(String, "power_status", self.on_power_status, 10)
        self.create_subscription(
            String, "/smart_carrier/task_result", self.on_task_result, STATE_QOS
        )
        self.create_subscription(
            String, "/smart_carrier/task_state", self.on_task_state, STATE_QOS
        )
        self.create_timer(0.1, self.network_cycle)

        if not self.configured:
            self.get_logger().error(
                "API bridge 尚未完整設定；請載入 SMART_CARRIER_API_URL、"
                "SMART_CARRIER_ROBOT_ID、SMART_CARRIER_ROBOT_TOKEN"
            )
        else:
            self.get_logger().info(f"API bridge 已啟動：robot={robot_id}, api={api_url}")
            if self.claimed_tasks:
                self.get_logger().warning(
                    f"已復原 {len(self.claimed_tasks)} 筆尚未完成任務，等待雲端對帳"
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

    def task_ids(self):
        return {str(task.get("id")) for task in self.claimed_tasks if task.get("id")}

    def publish_claimed_orders(self):
        if not self.active_batch:
            return
        now = time.monotonic()
        interval = float(self.get_parameter("order_republish_interval").value)
        if now - self.last_batch_publish_at < interval:
            return
        tasks = tasks_for_batch(self.claimed_tasks, self.active_batch)
        tasks = [
            task
            for task in tasks
            if not self.store.has_pending_result(str(task.get("id")))
        ]
        if not tasks:
            self._set_active_batch(None)
            return
        message = String()
        message.data = json.dumps(
            {
                "batch_id": self.active_batch["id"],
                "orders": tasks,
                "created_at": self.active_batch.get("created_at"),
            },
            ensure_ascii=False,
        )
        self.order_publisher.publish(message)
        self.last_batch_publish_at = now

    def _set_active_batch(self, batch):
        self.active_batch = batch
        self.store.set_active_batch(batch)
        self.last_batch_publish_at = 0.0

    def _collectable_tasks(self):
        return [
            task
            for task in self.claimed_tasks
            if task.get("id")
            and not self.store.has_pending_result(str(task["id"]))
        ]

    def _ensure_collection(self, now):
        if self.active_batch or not self._collectable_tasks():
            return
        if self.collection_started_at is None:
            self.collection_started_at = now
            self.collection_empty_since = None

    def _maybe_close_collection(self, now):
        if self.active_batch:
            return False
        tasks = self._collectable_tasks()
        self._ensure_collection(now)
        if not should_close_claim_batch(
            len(tasks),
            now=now,
            started_at=self.collection_started_at,
            empty_since=self.collection_empty_since,
            max_tasks=self.max_claimed_tasks,
            quiet_period=self.batch_quiet_period,
            max_wait=self.batch_max_wait,
        ):
            return False
        batch = {
            "id": str(uuid.uuid4()),
            "task_ids": [str(task["id"]) for task in tasks],
            "created_at": time.time(),
        }
        self._set_active_batch(batch)
        if self.store.get_pending_claim() is not None:
            # A timed-out claim may have committed remotely.  Do not let its
            # stale slot snapshot leak into the next batch; reconciliation will
            # recover the idempotently claimed task before another claim begins.
            self.store.set_pending_claim(None)
            self.reconciled = False
            self.next_reconcile_at = now
        self.collection_started_at = None
        self.collection_empty_since = None
        self.publish_claimed_orders()
        self.get_logger().info(
            f"批次 {batch['id']} 已封存，共 {len(tasks)} 筆；交由車端一次排程"
        )
        return True

    def _complete_batch_task_locally(self, task_id):
        if not self.active_batch:
            return
        remaining = [
            item
            for item in self.active_batch.get("task_ids", [])
            if str(item) != str(task_id)
        ]
        if remaining:
            batch = dict(self.active_batch)
            batch["task_ids"] = remaining
            self._set_active_batch(batch)
        else:
            self._set_active_batch(None)

    def _submit(self, kind, callback, **context):
        self.network_action = (kind, context)
        self.network_future = self.network.submit(callback)

    def _try_submit_claim(self, now, poll_interval):
        if (
            self.active_batch
            or not self.reconciled
            or self.store.has_pending_results()
            or self.order_publisher.get_subscription_count() == 0
        ):
            return False
        pending_claim = self.store.get_pending_claim()
        if pending_claim is None:
            if not self.power_healthy or len(self.claimed_tasks) >= self.max_claimed_tasks:
                return False
            if now < self.next_claim_at:
                return False
            try:
                projected_slots = project_claimed_slots(self.slots, self.claimed_tasks)
            except ClaimProjectionError as exc:
                self.get_logger().error(f"多筆任務槽位預約失敗：{exc}")
                self.next_claim_at = now + poll_interval
                return False
            pending_claim = {
                "claim_request_id": str(uuid.uuid4()),
                "slots": projected_slots,
            }
            # Persist before POST so a lost response retries the same claim.
            self.store.set_pending_claim(pending_claim)

        self._submit(
            "claim",
            lambda claim=pending_claim: self.api.claim_task(
                claim["slots"], claim["claim_request_id"]
            ),
            claim=pending_claim,
        )
        return True

    def network_cycle(self):
        if not self.configured:
            return
        self.publish_claimed_orders()
        if self.network_future is not None:
            if not self.network_future.done():
                return
            future = self.network_future
            kind, context = self.network_action
            self.network_future = None
            self.network_action = None
            try:
                result = future.result()
            except ApiError as exc:
                self._handle_network_error(kind, context, exc)
            except Exception as exc:
                self._handle_network_error(kind, context, ApiError(str(exc)))
            else:
                try:
                    self._handle_network_success(kind, context, result)
                except (ApiError, KeyError, TypeError, ValueError) as exc:
                    self._handle_network_error(kind, context, ApiError(str(exc)))
            if self.network_future is not None:
                return

        now = time.monotonic()
        if self.reconciled:
            self._ensure_collection(now)
            self._maybe_close_collection(now)
        progress = self.store.next_progress()
        if progress and now >= self.next_progress_at:
            self._submit(
                "progress",
                lambda p=progress: self.api.report_progress(str(p["task_id"]), p),
                progress=progress,
            )
            return

        result = self.store.next_result()
        if result and not self.store.has_pending_progress(result["task_id"]):
            if result["status"] == "released":
                def callback():
                    return self.api.release_task(
                        result["task_id"], result["event_id"], result.get("note")
                    )
            else:
                def callback():
                    return self.api.report_result(
                        result["task_id"],
                        result["event_id"],
                        result["status"],
                        result.get("note"),
                    )
            self._submit("result", callback, result=result)
            return

        if self.reconciliation_supported and (
            not self.reconciled or now >= self.next_reconcile_at
        ):
            self._submit("reconcile", self.api.list_tasks)
            return

        poll_interval = float(self.get_parameter("poll_interval").value)
        if self.collection_started_at is not None and self._try_submit_claim(
            now, poll_interval
        ):
            return

        for task in self.claimed_tasks:
            task_id = str(task["id"])
            if now >= self.next_task_poll_at.get(task_id, 0.0):
                self.next_task_poll_at[task_id] = now + poll_interval
                self._submit(
                    "task_status",
                    lambda item_id=task_id: self.api.get_task(item_id),
                    task_id=task_id,
                )
                return

        if now >= self.next_heartbeat_at:
            interval = float(self.get_parameter("heartbeat_interval").value)
            self.next_heartbeat_at = now + interval
            if self.claimed_tasks:
                mode = "dispatching"
            elif not self.power_healthy:
                mode = "power_sensor_error"
            else:
                mode = "idle"
            payload = {"mode": mode, "slots": list(self.slots)}
            self._submit("heartbeat", lambda p=payload: self.api.heartbeat(p))
            return

        self._try_submit_claim(now, poll_interval)

    def _handle_network_success(self, kind, context, result):
        was_offline = self.network_online is False
        self.network_online = True
        if was_offline and kind != "reconcile" and self.reconciliation_supported:
            self.reconciled = False
            self.next_reconcile_at = time.monotonic()
            self.get_logger().info("API 連線已恢復，重新對帳雲端指派任務")

        if kind == "progress":
            self.store.mark_progress_delivered(context["progress"]["event_id"])
            self.next_progress_at = time.monotonic()
        elif kind == "result":
            self._ack_result(context["result"])
        elif kind == "reconcile":
            self._merge_remote_tasks(result)
            self.reconciled = True
            self.next_reconcile_at = time.monotonic() + float(
                self.get_parameter("reconcile_interval").value
            )
        elif kind == "task_status":
            self._handle_task_status(context["task_id"], result)
        elif kind == "heartbeat" and not self.heartbeat_confirmed:
            self.get_logger().info("雲端 heartbeat 已成功")
            self.heartbeat_confirmed = True
        elif kind == "claim":
            self._accept_claim(result)
            self.store.set_pending_claim(None)

    def _handle_network_error(self, kind, context, error):
        if kind == "reconcile" and error.status_code == 404:
            self.reconciliation_supported = False
            self.reconciled = True
            self.get_logger().warning(
                "雲端 API 尚未提供任務對帳端點，暫用舊版相容模式；請儘快部署 API migration"
            )
            return
        if kind == "progress" and error.status_code == 404:
            progress = context["progress"]
            self.store.mark_progress_delivered(progress["event_id"])
            self.get_logger().warning(
                f"丟棄已非執行中任務的舊進度：{progress['task_id']}"
            )
            return
        if kind == "task_status" and error.status_code == 404:
            # The backend no longer owns this local task. Ask the delivery node
            # to stop it instead of retaining an unresolvable queue entry.
            self.network_online = True
            self._handle_task_status(
                context["task_id"],
                {
                    "status": "cancelled",
                    "cancel_reason": "Backend no longer assigns this task to the robot",
                },
            )
            return
        if kind == "result":
            item = context["result"]
            if is_terminal_result_error(item["status"], error.status_code):
                # A result response may be lost after the backend commits it, or
                # an administrator may force-finalize the task while the robot is
                # offline. In both cases the task is no longer assigned and
                # retrying can never succeed. Settle the durable local copy and
                # ACK the delivery journal so new claims are not blocked forever.
                self.network_online = True
                self._ack_result(item, terminally_reconciled=True)
                self.get_logger().warning(
                    "雲端任務已非執行中，安全結清本機結果："
                    f"{item['task_id']} ({item['event_id']})"
                )
                return
        self.network_online = False
        self.heartbeat_confirmed = False
        if kind == "result":
            item = context["result"]
            self.store.mark_retry(item["event_id"], int(item["attempts"]))
        elif kind == "progress":
            self.next_progress_at = time.monotonic() + 1.0
        now = time.monotonic()
        if kind == "claim":
            self.next_claim_at = now + 1.0
        elif kind == "reconcile":
            self.next_reconcile_at = now + 2.0
        self.log_api_error(error)

    def _merge_remote_tasks(self, remote_tasks):
        if not isinstance(remote_tasks, list):
            raise ApiError("API task reconciliation returned a non-list response")
        added = 0
        known = self.task_ids()
        remote_ids = set()
        for task in remote_tasks:
            if not isinstance(task, dict) or not task.get("id"):
                raise ApiError("API task reconciliation returned an invalid task")
            task_id = str(task["id"])
            remote_ids.add(task_id)
            if task_id not in known:
                self.claimed_tasks.append(task)
                known.add(task_id)
                added += 1
        for task_id in sorted(known - remote_ids):
            # Successful reconciliation is authoritative: a locally retained
            # task missing from this active-task list has already become terminal
            # in the backend. Keep it until the delivery node acknowledges the
            # cancellation, but stop treating it as silently active.
            self._handle_task_status(
                task_id,
                {
                    "status": "cancelled",
                    "cancel_reason": "Task is no longer active in backend reconciliation",
                },
            )
        if added:
            self.store.set_claimed_tasks(self.claimed_tasks)
            self.get_logger().warning(
                f"雲端對帳復原 {added} 筆本機遺失任務；等待批次排程"
            )

    def _accept_claim(self, task):
        poll_interval = float(self.get_parameter("poll_interval").value)
        now = time.monotonic()
        if not task:
            if self._collectable_tasks():
                if self.collection_empty_since is None:
                    self.collection_empty_since = now
                self.next_claim_at = now + self.batch_claim_retry_interval
            else:
                self.next_claim_at = now + poll_interval
            return
        if not isinstance(task, dict) or not task.get("id"):
            raise ApiError("API claimed task without an id")
        task_id = str(task["id"])
        if task_id not in self.task_ids():
            self.claimed_tasks.append(task)
            self.store.set_claimed_tasks(self.claimed_tasks)
            if self.collection_started_at is None:
                self.collection_started_at = now
            self.collection_empty_since = None
            self.get_logger().info(
                f"已領取並保存雲端任務 {task_id} "
                f"(批次收集 {len(self._collectable_tasks())}/{self.max_claimed_tasks})"
            )
        self.next_claim_at = now

    def _handle_task_status(self, task_id, task):
        if not isinstance(task, dict):
            raise ApiError(f"API returned invalid task status for {task_id}")
        cancellation_requested = bool(task.get("cancel_requested_at"))
        if task.get("status") in {"cancelled", "done", "failed"}:
            cancellation_requested = True
        if not cancellation_requested:
            self.last_cancel_publish_at.pop(task_id, None)
            return
        now = time.monotonic()
        interval = float(self.get_parameter("cancel_republish_interval").value)
        if now - self.last_cancel_publish_at.get(task_id, 0.0) < interval:
            return
        first_notification = task_id not in self.last_cancel_publish_at
        message = String()
        message.data = json.dumps(
            {
                "task_id": task_id,
                "reason": task.get("cancel_reason")
                or "Task was terminally stopped by administrator",
            },
            ensure_ascii=False,
        )
        self.cancel_publisher.publish(message)
        self.last_cancel_publish_at[task_id] = now
        if first_notification:
            self.get_logger().warning(f"收到雲端取消要求：{task_id}")

    def on_task_state(self, message):
        try:
            progress = json.loads(message.data)
            task_id = str(progress["task_id"])
            if task_id not in self.task_ids():
                return
            self.store.enqueue_progress(progress)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"忽略無效任務進度：{exc}")

    def on_task_result(self, message):
        try:
            result = json.loads(message.data)
            task_id = str(result["task_id"])
            event_id = str(result["event_id"])
            status = str(result["status"])
            if status not in {"done", "failed", "cancelled", "released"}:
                raise ValueError(f"不支援的任務結果狀態：{status}")
            if task_id not in self.task_ids():
                self.get_logger().warning(f"保存非目前 claimed queue 任務結果：{task_id}")
            stored = self.store.enqueue_result(result)
            if stored:
                self.get_logger().info(
                    f"任務結果已保存至本機 outbox，等待雲端確認：{task_id} ({event_id})"
                )
            if stored or self.store.has_result_event(event_id) or self.store.is_result_settled(
                event_id
            ):
                # This ACK means the terminal result is durably owned by the
                # bridge, not that the cloud has already replied.  It lets the
                # delivery node continue the frozen route while the outbox
                # retries independently across network outages and restarts.
                self._complete_batch_task_locally(task_id)
                ack = String()
                ack.data = json.dumps({"event_id": event_id, "task_id": task_id})
                self.result_ack_publisher.publish(ack)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"忽略無效任務結果：{exc}")

    def _ack_result(self, item, *, terminally_reconciled=False):
        self.store.settle_result(item["event_id"], item["task_id"])
        self.claimed_tasks = self.store.get_claimed_tasks()
        ack = String()
        ack.data = json.dumps({"event_id": item["event_id"], "task_id": item["task_id"]})
        self.result_ack_publisher.publish(ack)
        self.last_cancel_publish_at.pop(item["task_id"], None)
        self.next_task_poll_at.pop(item["task_id"], None)
        if not terminally_reconciled:
            self.get_logger().info(f"雲端已確認任務結果 {item['task_id']}: {item['status']}")

    def log_api_error(self, error):
        now = time.monotonic()
        if now - self.last_error_at >= 10.0:
            self.get_logger().error(str(error))
            self.last_error_at = now

    def destroy_node(self):
        self.network.shutdown(wait=True, cancel_futures=True)
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
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            # ros2 launch may deliver a second SIGINT while cleanup is running.
            pass
        if rclpy.ok():
            rclpy.shutdown()
