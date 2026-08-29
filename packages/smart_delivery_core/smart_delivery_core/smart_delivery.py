import json
import math
import time
import uuid
from dataclasses import dataclass
from datetime import UTC, datetime

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from power_monitor.power_status import normalize_power_bank_status, payload_to_slots
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from smart_delivery_core.delivery_state import (
    DeliveryJournal,
    DeliveryStateMachine,
    SlotActionVerifier,
)

BORROW_DISTANCE_WEIGHT = 0.7
POWER_STATUS_TIMEOUT_SECONDS = 10.0
SLOT_CONFIRMATION_WARNING_SECONDS = 30.0
SLOT_CONFIRMATION_TIMEOUT_SECONDS = 60.0
SLOT_CONFIRMATION_SAMPLES = 6
OCCUPIED_SLOT_STATUSES = {"low", "ready", "full"}
STATE_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# --- 1. 預先定義的靠牆待機點 ---
STANDBY_POINTS = [
    {"name": "門口", "x": -0.45, "y": -0.35, "yaw": 0.0},
    # {"name": "走廊轉角", "x": 5.0, "y": 0.0, "yaw": -1.57}
]

# --- 2. 固定的配送座標清單 (查表法資料庫) ---
LOCATION_DB = {
    "門口": {"x": -0.45, "y": -0.35, "yaw": 0.0},
    # "走廊轉角": {"x": 5.0, "y": 0.0, "yaw": -1.57},
    "廚房": {"x": -1.5, "y": 5.0, "yaw": 0.0},
    "廁所": {"x": -4.0, "y": 1.5, "yaw": 3.14},
    "客廳TV": {"x": 0.4, "y": 0.0, "yaw": 0.0},
}

BORROWABLE_STATUS_PRIORITY = {
    "full": 0,
    "ready": 1,
}

NAVIGATION_ONLY_TASK_TYPES = {"delivery", "navigation", "callbot"}


@dataclass
class SlotConfirmationTracker:
    """Confirm a physical borrow/return action using fresh slot snapshots."""

    task_type: str
    slot_number: int
    confirm_samples: int = SLOT_CONFIRMATION_SAMPLES
    matching_samples: int = 0
    last_status: str = "unknown"

    def __post_init__(self):
        if self.task_type not in {"borrow", "return"}:
            raise ValueError("槽位確認只支援 borrow 或 return")
        if self.slot_number not in (1, 2, 3):
            raise ValueError("slot_number 必須介於 1 到 3")
        if self.confirm_samples < 1:
            raise ValueError("confirm_samples 必須大於 0")

    def update(self, slots):
        slot = next(
            (item for item in slots if int(item.get("slot", 0)) == self.slot_number),
            None,
        )
        if not slot or not slot.get("sensor_ok", False):
            self.last_status = "unknown"
            self.matching_samples = 0
            return False

        self.last_status = normalize_power_bank_status(slot.get("status"))
        if self.task_type == "borrow":
            matches = self.last_status == "empty"
        else:
            matches = self.last_status in OCCUPIED_SLOT_STATUSES
        self.matching_samples = self.matching_samples + 1 if matches else 0
        return self.matching_samples >= self.confirm_samples


def select_power_bank(
    power_bank_slots,
    required_charge=0,
    status_aliases=None,
    power_bank_id=None,
):
    if not 0 <= required_charge <= 100:
        raise ValueError("required_charge 必須介於 0 到 100")
    candidates = []
    for slot_index, power_bank in enumerate(power_bank_slots):
        if power_bank is None:
            continue
        bank_id = power_bank.get("bank_id") or power_bank.get("id")
        if power_bank_id is not None and bank_id != power_bank_id:
            continue
        status = normalize_power_bank_status(power_bank.get("status"), status_aliases)
        if status not in BORROWABLE_STATUS_PRIORITY:
            continue
        charge = power_bank.get("charge")
        if charge is None:
            if required_charge > 0 and status != "full":
                continue
            effective_charge = 100.0 if status == "full" else 101.0
        else:
            charge = float(charge)
            if charge < required_charge:
                continue
            effective_charge = charge

        candidates.append(
            (
                BORROWABLE_STATUS_PRIORITY[status],
                effective_charge,
                slot_index,
                power_bank,
            )
        )

    if not candidates:
        return None
    _, _, slot_index, power_bank = min(candidates, key=lambda item: item[:3])
    return slot_index, power_bank


def _prepare_slots(power_banks, slot_capacity, status_aliases=None):
    slots = []
    for power_bank in power_banks:
        if power_bank is None:
            slots.append(None)
            continue
        bank = dict(power_bank)
        bank["status"] = normalize_power_bank_status(bank.get("status"), status_aliases)
        bank.setdefault("id", bank.get("bank_id"))
        slots.append(None if bank["status"] == "empty" else bank)
    return slots + [None] * (slot_capacity - len(slots))


def _returned_power_bank(order, route_number, status_aliases=None):
    power_bank = dict(
        order.get(
            "power_bank",
            {
                "id": f"returned-{route_number}",
                "status": order.get("return_status", "low"),
                "charge": order.get("charge"),
            },
        )
    )
    power_bank["status"] = normalize_power_bank_status(power_bank.get("status"), status_aliases)
    return power_bank


def schedule_orders(
    pending_orders,
    current_pos,
    power_banks,
    slot_capacity=3,
    status_aliases=None,
    borrow_distance_weight=BORROW_DISTANCE_WEIGHT,
):
    orders = [dict(order) for order in pending_orders]
    slots = _prepare_slots(power_banks, slot_capacity, status_aliases)
    position = {"x": float(current_pos["x"]), "y": float(current_pos["y"])}
    optimized_route = []

    while orders:
        feasible_orders = []
        for order_index, order in enumerate(orders):
            order_type = order.get("type")
            distance = math.hypot(
                float(order["x"]) - position["x"], float(order["y"]) - position["y"]
            )

            if order_type == "borrow":
                selection = select_power_bank(
                    slots,
                    float(order.get("required_charge", 0)),
                    status_aliases,
                    order.get("power_bank_id"),
                )
                if selection is not None:
                    feasible_orders.append(
                        (
                            distance * borrow_distance_weight,
                            distance,
                            order_index,
                            "borrow",
                            selection,
                        )
                    )
            elif order_type == "return":
                try:
                    empty_slot = slots.index(None)
                except ValueError:
                    continue
                feasible_orders.append((distance, distance, order_index, "return", empty_slot))
            elif order_type in NAVIGATION_ONLY_TASK_TYPES:
                feasible_orders.append((distance, distance, order_index, order_type, None))

        if not feasible_orders:
            break

        priority_score, distance, order_index, order_type, resource = min(
            feasible_orders, key=lambda item: (item[0], item[1], item[2])
        )
        order = orders.pop(order_index)
        scheduled_order = dict(order)
        scheduled_order["distance"] = distance

        if order_type == "borrow":
            slot_index, power_bank = resource
            scheduled_order["selected_power_bank"] = dict(power_bank)
            scheduled_order["slot_number"] = slot_index + 1
            slots[slot_index] = None
        elif order_type == "return":
            slot_index = resource
            power_bank = _returned_power_bank(order, len(optimized_route) + 1, status_aliases)
            slots[slot_index] = power_bank
            scheduled_order["returned_power_bank"] = dict(power_bank)
            scheduled_order["slot_number"] = slot_index + 1

        optimized_route.append(scheduled_order)
        position = {"x": float(scheduled_order["x"]), "y": float(scheduled_order["y"])}

    return optimized_route, orders, slots


def order_payload_to_order(payload):
    data = json.loads(payload) if isinstance(payload, str) else payload
    if not isinstance(data, dict):
        raise ValueError("訂單必須是 JSON object")

    task_type = str(data.get("task_type") or data.get("type") or "borrow").lower()
    if task_type not in {"borrow", "return", *NAVIGATION_ONLY_TASK_TYPES}:
        raise ValueError(f"不支援的訂單類型：{task_type}")

    quantity = int(data.get("quantity") or 1)
    if quantity != 1:
        raise ValueError("目前機器人每筆任務只支援 1 顆行動電源")

    location = data.get("location") or {}
    if not isinstance(location, dict):
        raise ValueError("location 必須是 JSON object")
    location_code = data.get("location_code") or location.get("code")
    location_name = location.get("name") or data.get("name") or location_code
    if location.get("x") is not None and location.get("y") is not None:
        coordinates = {
            "x": float(location["x"]),
            "y": float(location["y"]),
            "yaw": float(location.get("yaw") or 0.0),
        }
    elif location_name in LOCATION_DB:
        coordinates = LOCATION_DB[location_name]
    else:
        raise ValueError(f"地點缺少地圖座標：{location_name}")

    required_charge = int(data.get("required_charge") or 0)
    if not 0 <= required_charge <= 100:
        raise ValueError("required_charge 必須介於 0 到 100")

    order = {
        "task_id": str(data["id"]) if data.get("id") is not None else None,
        "location_code": location_code,
        "name": location_name,
        "type": task_type,
        "power_bank_id": data.get("power_bank_id"),
        "required_charge": required_charge,
        "quantity": quantity,
        **coordinates,
    }
    if task_type == "return":
        order["power_bank"] = data.get("power_bank", {"id": "PB-RT", "status": "low", "charge": 20})
    return order


def publish_task_result(publisher, task_id, status, note, event_id=None):
    if not task_id:
        return None
    event_id = event_id or str(uuid.uuid4())
    message = String()
    message.data = json.dumps(
        {
            "event_id": event_id,
            "task_id": str(task_id),
            "status": status,
            "note": note,
            "created_at": datetime.now(UTC).isoformat(),
        },
        ensure_ascii=False,
    )
    publisher.publish(message)
    return json.loads(message.data)


def publish_task_state(publisher, task_id, state, *, event_id=None, **details):
    if not task_id:
        return
    payload = {
        "event_id": event_id or str(uuid.uuid4()),
        "task_id": str(task_id),
        "progress_state": state,
        "progress_updated_at": datetime.now(UTC).isoformat(),
        **details,
    }
    message = String()
    message.data = json.dumps(payload, ensure_ascii=False)
    publisher.publish(message)


def infeasible_order_note(order):
    if order.get("type") == "borrow":
        if order.get("power_bank_id"):
            return f"Selected power bank {order['power_bank_id']} is no longer available"
        return "No ready power bank meets the required charge"
    if order.get("type") == "return":
        return "No empty power slot is available"
    return "Task is not executable with the current robot state"


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def go_to_standby(navigator, current_pos, localization_is_ready=lambda: True):
    """尋找最近的待機點並前往避讓"""
    print("\n💤 進入待機模式，尋找最近的靠牆避讓點...")
    closest_standby = min(
        STANDBY_POINTS,
        key=lambda pt: math.hypot(pt["x"] - current_pos["x"], pt["y"] - current_pos["y"]),
    )

    print(f"➡️ 前往待機點：{closest_standby['name']}")
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = closest_standby["x"]
    goal_pose.pose.position.y = closest_standby["y"]
    goal_pose.pose.orientation = yaw_to_quaternion(closest_standby["yaw"])

    navigator.goToPose(goal_pose)
    while not navigator.isTaskComplete():
        time.sleep(0.1)  # 釋放 CPU
        rclpy.spin_once(navigator, timeout_sec=0.05)
        if not localization_is_ready():
            navigator.cancelTask()
            navigator.get_logger().warning("定位可信度不足，已取消前往待機點")
            while not navigator.isTaskComplete():
                rclpy.spin_once(navigator, timeout_sec=0.05)
            return None

    print(f"✅ 已靠牆停妥於 {closest_standby['name']}，等待新任務。")
    return {"x": closest_standby["x"], "y": closest_standby["y"]}


def main():
    rclpy.init()
    navigator = BasicNavigator()
    tf_buffer = Buffer()
    _tf_listener = TransformListener(tf_buffer, navigator)
    navigator.declare_parameter("slot_confirmation_warning_sec", SLOT_CONFIRMATION_WARNING_SECONDS)
    navigator.declare_parameter("slot_confirmation_timeout_sec", SLOT_CONFIRMATION_TIMEOUT_SECONDS)
    navigator.declare_parameter("slot_confirmation_samples", SLOT_CONFIRMATION_SAMPLES)
    warning_seconds = float(navigator.get_parameter("slot_confirmation_warning_sec").value)
    timeout_seconds = float(navigator.get_parameter("slot_confirmation_timeout_sec").value)
    confirmation_samples = int(navigator.get_parameter("slot_confirmation_samples").value)
    if not 0 < warning_seconds < timeout_seconds or confirmation_samples < 1:
        raise ValueError("槽位確認須符合 0 < warning < timeout，且樣本數大於 0")

    localization_ready = False
    localization_state = "UNINITIALIZED"
    last_localization_warning_at = 0.0

    def localization_ready_callback(message):
        nonlocal localization_ready
        localization_ready = bool(message.data)

    def localization_state_callback(message):
        nonlocal localization_state
        try:
            localization_state = json.loads(message.data).get("state", "UNKNOWN")
        except (AttributeError, TypeError, json.JSONDecodeError):
            localization_state = "UNKNOWN"

    navigator.create_subscription(
        Bool, "/localization/ready", localization_ready_callback, STATE_QOS
    )
    navigator.create_subscription(
        String, "/localization/state", localization_state_callback, STATE_QOS
    )

    def refresh_current_position(fallback):
        try:
            transform = tf_buffer.lookup_transform(
                "map", "base_footprint", Time(), timeout=Duration(seconds=0.2)
            )
        except TransformException as exc:
            navigator.get_logger().warning(f"無法取得即時 map 位姿，暫用上次位置：{exc}")
            return fallback
        return {
            "x": float(transform.transform.translation.x),
            "y": float(transform.transform.translation.y),
        }

    print("⏳ 等待 Nav2 系統上線...")
    navigator.waitUntilNav2Active()
    print("✅ Nav2 準備就緒！")

    print("⏳ 等待定位管理器確認位置...")
    while rclpy.ok() and not localization_ready:
        rclpy.spin_once(navigator, timeout_sec=0.2)
        now = time.monotonic()
        if now - last_localization_warning_at >= 10.0:
            navigator.get_logger().warning(
                f"定位尚未就緒（{localization_state}），不會送出導航目標"
            )
            last_localization_warning_at = now
    if not rclpy.ok():
        return
    print("✅ 定位可信度檢查通過！")

    latest_power_banks = None
    latest_power_status_at = 0.0
    latest_power_sequence = 0
    last_power_error_at = 0.0
    pending_orders = []
    active_task_id = None
    cancellation_requests = {}
    journal = DeliveryJournal()
    recovered = journal.load()
    pending_result = recovered.get("pending_result")
    active_task = recovered.get("active_task")
    last_result_publish_at = 0.0

    task_result_publisher = navigator.create_publisher(
        String, "/smart_carrier/task_result", STATE_QOS
    )
    task_state_publisher = navigator.create_publisher(
        String, "/smart_carrier/task_state", STATE_QOS
    )
    motion_inhibit_publisher = navigator.create_publisher(
        Bool, "/localization/motion_inhibited", STATE_QOS
    )

    def set_motion_inhibited(inhibited):
        message = Bool()
        message.data = bool(inhibited)
        motion_inhibit_publisher.publish(message)

    set_motion_inhibited(False)

    def persist(active=None, state=None, result=None):
        data = {}
        if active is not None:
            data["active_task"] = active
        if state is not None:
            data["state"] = state
        if result is not None:
            data["pending_result"] = result
        if data:
            journal.save(data)
        else:
            journal.clear()

    def emit_result(result):
        nonlocal last_result_publish_at
        message = String()
        message.data = json.dumps(result, ensure_ascii=False)
        task_result_publisher.publish(message)
        last_result_publish_at = time.monotonic()

    def queue_result(task, fsm, status, note):
        nonlocal pending_result
        if fsm.state != "result_pending":
            fsm.transition("result_pending")
        publish_task_state(
            task_state_publisher,
            task.get("task_id"),
            "result_pending",
            progress_message="操作已完成，正在同步結果，請勿重複操作",
        )
        pending_result = {
            "event_id": str(uuid.uuid4()),
            "task_id": str(task["task_id"]),
            "status": status,
            "note": note,
            "created_at": datetime.now(UTC).isoformat(),
        }
        persist(task, fsm.state, pending_result)
        emit_result(pending_result)

    def progress(task, fsm, state, **details):
        fsm.transition(state)
        persist(task, fsm.state, pending_result)
        publish_task_state(task_state_publisher, task.get("task_id"), state, **details)

    def result_ack_callback(message):
        nonlocal active_task, pending_result
        try:
            ack = json.loads(message.data)
            if pending_result and str(ack.get("event_id")) == pending_result["event_id"]:
                publish_task_state(
                    task_state_publisher,
                    pending_result["task_id"],
                    "result_acked",
                )
                pending_result = None
                active_task = None
                journal.clear()
        except (TypeError, json.JSONDecodeError):
            navigator.get_logger().warning("忽略無效 task_result_ack")

    navigator.create_subscription(
        String, "/smart_carrier/task_result_ack", result_ack_callback, STATE_QOS
    )

    if active_task and not pending_result:
        task_id = active_task.get("task_id") or active_task.get("id")
        pending_result = {
            "event_id": str(uuid.uuid4()),
            "task_id": str(task_id),
            "status": "failed",
            "note": "Robot restarted during an active task; manual recovery required",
            "created_at": datetime.now(UTC).isoformat(),
        }
        persist(active_task, "recovery_required", pending_result)
        publish_task_state(
            task_state_publisher,
            task_id,
            "recovery_required",
            progress_message="機器人重啟後偵測到未完成任務，已停止移動並等待結果同步",
        )
        navigator.get_logger().error(
            f"偵測到重啟前未完成任務 {task_id}；不自動續航，改回報復原失敗"
        )

    def power_status_callback(msg):
        nonlocal latest_power_banks, latest_power_status_at
        nonlocal latest_power_sequence, last_power_error_at
        try:
            latest_power_banks = payload_to_slots(msg.data, require_healthy=True)
            latest_power_status_at = time.monotonic()
            latest_power_sequence += 1
        except ValueError as exc:
            now = time.monotonic()
            if now - last_power_error_at >= 10.0:
                navigator.get_logger().error(str(exc))
                last_power_error_at = now

    navigator.create_subscription(String, "power_status", power_status_callback, 10)

    def order_callback(msg):
        task_id = None
        try:
            raw = json.loads(msg.data)
            task_id = raw.get("id")
            order = order_payload_to_order(raw)
            if pending_result:
                navigator.get_logger().warning(f"結果尚待本機橋接確認，暫不接收任務 {task_id}")
                return
            if any(item.get("task_id") == str(task_id) for item in pending_orders):
                return
            pending_orders.append(order)
            publish_task_state(
                task_state_publisher,
                task_id,
                "queued_on_robot",
                progress_message=f"Queued on robot ({len(pending_orders)} waiting)",
            )
            print(f"\n📥 收到雲端任務：{order['name']} ({order['type']})")
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            navigator.get_logger().error(f"訂單解析失敗：{exc}")
            if task_id:
                invalid_task = {"task_id": str(task_id)}
                queue_result(
                    invalid_task,
                    DeliveryStateMachine("task_accepted"),
                    "failed",
                    str(exc),
                )

    navigator.create_subscription(String, "order", order_callback, STATE_QOS)

    def task_cancel_callback(msg):
        nonlocal active_task_id
        try:
            cancellation = json.loads(msg.data)
            task_id = str(cancellation["task_id"])
            reason = str(cancellation.get("reason") or "Cancelled by administrator")
            cancellation_requests[task_id] = reason
            if active_task_id == task_id:
                navigator.get_logger().warning(f"正在停止管理員取消的任務：{task_id}")
                navigator.cancelTask()
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            navigator.get_logger().error(f"無效的取消任務訊息：{exc}")

    navigator.create_subscription(
        String, "/smart_carrier/task_cancel", task_cancel_callback, STATE_QOS
    )

    print("⏳ 等待 INA3221 電流資料...")
    deadline = time.monotonic() + POWER_STATUS_TIMEOUT_SECONDS
    while latest_power_banks is None and time.monotonic() < deadline:
        rclpy.spin_once(navigator, timeout_sec=0.2)
    if latest_power_banks is None:
        navigator.get_logger().error("超時未收到健康的 power_status")
        rclpy.shutdown()
        return

    current_pos = {"x": 0.0, "y": 0.0}
    is_standby = False
    last_stale_warning_at = 0.0
    print("\n🚀 智慧動態派車系統已啟動")

    while rclpy.ok():
        rclpy.spin_once(navigator, timeout_sec=0.5)

        if not localization_ready:
            now = time.monotonic()
            if now - last_localization_warning_at >= 10.0:
                navigator.get_logger().warning(
                    f"定位暫停（{localization_state}），保留訂單並等待恢復"
                )
                last_localization_warning_at = now
            continue

        if pending_result:
            if time.monotonic() - last_result_publish_at >= 2.0:
                emit_result(pending_result)
            if not is_standby:
                standby_pos = go_to_standby(navigator, current_pos, lambda: localization_ready)
                if standby_pos is not None:
                    current_pos = standby_pos
                    is_standby = True
            continue

        if not pending_orders:
            if not is_standby:
                print("\n🏁 無待處理任務，返回待機點")
                standby_pos = go_to_standby(navigator, current_pos, lambda: localization_ready)
                if standby_pos is not None:
                    current_pos = standby_pos
                    is_standby = True
            continue

        cancelled_order = next(
            (order for order in pending_orders if order.get("task_id") in cancellation_requests),
            None,
        )
        if cancelled_order:
            pending_orders.remove(cancelled_order)
            active_task = cancelled_order
            fsm = DeliveryStateMachine("task_accepted")
            persist(active_task, fsm.state, None)
            queue_result(
                cancelled_order,
                fsm,
                "cancelled",
                cancellation_requests.pop(cancelled_order["task_id"]),
            )
            continue

        now = time.monotonic()
        if now - latest_power_status_at > POWER_STATUS_TIMEOUT_SECONDS:
            if now - last_stale_warning_at >= 10.0:
                navigator.get_logger().error("power_status 已逾時，暫停新任務")
                last_stale_warning_at = now
            continue

        current_pos = refresh_current_position(current_pos)
        optimized_route, _, _ = schedule_orders(
            pending_orders, current_pos, latest_power_banks, slot_capacity=3
        )
        if not optimized_route:
            target = pending_orders.pop(0)
            active_task = target
            fsm = DeliveryStateMachine("task_accepted")
            persist(active_task, fsm.state, None)
            progress(target, fsm, "precheck", progress_message="Checking slot availability")
            queue_result(target, fsm, "released", infeasible_order_note(target))
            continue

        target = optimized_route[0]
        active_task = target
        fsm = DeliveryStateMachine("task_accepted")
        persist(active_task, fsm.state, None)
        is_standby = False
        progress(target, fsm, "precheck", progress_message="Task precheck passed")
        progress(
            target,
            fsm,
            "navigating",
            progress_message=f"Navigating to {target['name']}",
            expected_slot=target.get("slot_number"),
        )

        print(f"\n➡️ 前往 {target['name']}（{target['type']}，距離 {target['distance']:.2f}m）")
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(target["x"])
        goal_pose.pose.position.y = float(target["y"])
        goal_pose.pose.orientation = yaw_to_quaternion(target["yaw"])
        navigator.goToPose(goal_pose)
        active_task_id = target.get("task_id")
        cancel_sent = False
        localization_aborted = False
        while not navigator.isTaskComplete():
            time.sleep(0.1)
            rclpy.spin_once(navigator, timeout_sec=0.05)
            if active_task_id in cancellation_requests and not cancel_sent:
                navigator.cancelTask()
                cancel_sent = True
            elif not localization_ready and not cancel_sent:
                navigator.get_logger().error(
                    f"定位進入 {localization_state}，取消導航並保留任務等待重送"
                )
                navigator.cancelTask()
                cancel_sent = True
                localization_aborted = True

        nav_result = navigator.getResult()
        keep_pending = False
        if localization_aborted:
            fsm.transition("waiting_localization")
            persist(target, fsm.state, pending_result)
            publish_task_state(
                task_state_publisher,
                target.get("task_id"),
                "waiting_localization",
                progress_message="Localization recovery in progress; task retained",
            )
            keep_pending = True
        elif active_task_id in cancellation_requests:
            queue_result(
                target,
                fsm,
                "cancelled",
                cancellation_requests.pop(active_task_id),
            )
        elif nav_result != TaskResult.SUCCEEDED:
            note = (
                "Nav2 goal cancelled" if nav_result == TaskResult.CANCELED else "Nav2 goal failed"
            )
            queue_result(target, fsm, "failed", note)
        else:
            current_pos = {"x": float(target["x"]), "y": float(target["y"])}
            progress(
                target,
                fsm,
                "arrived",
                progress_message=f"Arrived at {target['name']}",
                expected_slot=target.get("slot_number"),
            )
            if target["type"] not in {"borrow", "return"}:
                queue_result(target, fsm, "done", "Nav2 goal reached")
            else:
                slot_number = int(target["slot_number"])
                try:
                    verifier = SlotActionVerifier(
                        target["type"],
                        slot_number,
                        latest_power_banks,
                        confirm_samples=confirmation_samples,
                    )
                except ValueError as exc:
                    queue_result(target, fsm, "failed", str(exc))
                else:
                    action_deadline = datetime.fromtimestamp(
                        time.time() + timeout_seconds, UTC
                    ).isoformat()
                    progress(
                        target,
                        fsm,
                        "waiting_action",
                        progress_message=(
                            f"請在 {slot_number} 號槽"
                            f"{'取走' if target['type'] == 'borrow' else '放入'}行動電源"
                        ),
                        expected_slot=slot_number,
                        action_deadline=action_deadline,
                    )
                    set_motion_inhibited(True)
                    started = time.monotonic()
                    warned = False
                    last_sequence = latest_power_sequence
                    last_verification = None
                    confirmed = False
                    action_cancelled = False
                    while rclpy.ok() and time.monotonic() - started < timeout_seconds:
                        rclpy.spin_once(navigator, timeout_sec=0.2)
                        if active_task_id in cancellation_requests:
                            action_cancelled = True
                            break
                        elapsed = time.monotonic() - started
                        if not warned and elapsed >= warning_seconds:
                            warned = True
                            publish_task_state(
                                task_state_publisher,
                                target.get("task_id"),
                                fsm.state,
                                progress_message="尚未完成指定槽位操作；30 秒後任務將失敗",
                                expected_slot=slot_number,
                                action_deadline=action_deadline,
                                warning=True,
                            )
                        if latest_power_sequence == last_sequence:
                            continue
                        last_sequence = latest_power_sequence
                        verification = verifier.update(latest_power_banks)
                        signature = (
                            verification.state,
                            verification.changed_slot,
                            verification.changed_slots,
                            verification.message,
                        )
                        if signature != last_verification:
                            state = (
                                verification.state
                                if verification.state
                                in {"wrong_slot", "waiting_action", "verifying_action"}
                                else "waiting_action"
                            )
                            progress(
                                target,
                                fsm,
                                state,
                                progress_message=verification.message,
                                expected_slot=slot_number,
                                changed_slot=verification.changed_slot,
                                changed_slots=list(verification.changed_slots),
                                action_deadline=action_deadline,
                                warning=warned,
                            )
                            last_verification = signature
                        if verification.confirmed:
                            confirmed = True
                            break
                    set_motion_inhibited(False)
                    if action_cancelled:
                        queue_result(
                            target,
                            fsm,
                            "cancelled",
                            cancellation_requests.pop(active_task_id),
                        )
                    elif confirmed:
                        queue_result(
                            target,
                            fsm,
                            "done",
                            f"Nav2 goal reached and slot {slot_number} action confirmed",
                        )
                    else:
                        queue_result(
                            target,
                            fsm,
                            "failed",
                            f"Timed out after {timeout_seconds:.0f}s waiting "
                            f"for slot {slot_number}",
                        )

        if not keep_pending:
            target_key = target.get("task_id") or (target["name"], target["type"])
            pending_orders[:] = [
                order
                for order in pending_orders
                if (order.get("task_id") or (order["name"], order["type"])) != target_key
            ]
        active_task_id = None

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
