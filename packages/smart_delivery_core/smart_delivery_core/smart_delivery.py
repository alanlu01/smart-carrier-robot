import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from power_monitor.power_status import normalize_power_bank_status, payload_to_slots
from std_msgs.msg import String

BORROW_DISTANCE_WEIGHT = 0.7
POWER_STATUS_TIMEOUT_SECONDS = 10.0

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

def select_power_bank(
    power_bank_slots,
    required_charge=0,
    status_aliases=None,
    power_bank_id=None,
):
    if not 0 <= required_charge <= 100: raise ValueError("required_charge 必須介於 0 到 100")
    candidates = []
    for slot_index, power_bank in enumerate(power_bank_slots):
        if power_bank is None: continue
        bank_id = power_bank.get("bank_id") or power_bank.get("id")
        if power_bank_id is not None and bank_id != power_bank_id:
            continue
        status = normalize_power_bank_status(power_bank.get("status"), status_aliases)
        if status not in BORROWABLE_STATUS_PRIORITY: continue
        charge = power_bank.get("charge")
        if charge is not None:
            charge = float(charge)
            if charge < required_charge: continue

        best_fit_charge = charge if charge is not None else 101
        candidates.append((BORROWABLE_STATUS_PRIORITY[status], best_fit_charge, slot_index, power_bank))

    if not candidates: return None
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
    power_bank = dict(order.get("power_bank", {"id": f"returned-{route_number}", "status": order.get("return_status", "low"), "charge": order.get("charge")}))
    power_bank["status"] = normalize_power_bank_status(power_bank.get("status"), status_aliases)
    return power_bank

def schedule_orders(pending_orders, current_pos, power_banks, slot_capacity=3, status_aliases=None, borrow_distance_weight=BORROW_DISTANCE_WEIGHT):
    orders = [dict(order) for order in pending_orders]
    slots = _prepare_slots(power_banks, slot_capacity, status_aliases)
    position = {"x": float(current_pos["x"]), "y": float(current_pos["y"])}
    optimized_route = []

    while orders:
        feasible_orders = []
        for order_index, order in enumerate(orders):
            order_type = order.get("type")
            distance = math.hypot(float(order["x"]) - position["x"], float(order["y"]) - position["y"])

            if order_type == "borrow":
                selection = select_power_bank(
                    slots,
                    float(order.get("required_charge", 0)),
                    status_aliases,
                    order.get("power_bank_id"),
                )
                if selection is not None:
                    feasible_orders.append((distance * borrow_distance_weight, distance, order_index, "borrow", selection))
            elif order_type == "return":
                try: empty_slot = slots.index(None)
                except ValueError: continue
                feasible_orders.append((distance, distance, order_index, "return", empty_slot))
            elif order_type in NAVIGATION_ONLY_TASK_TYPES:
                feasible_orders.append((distance, distance, order_index, order_type, None))

        if not feasible_orders: break

        priority_score, distance, order_index, order_type, resource = min(feasible_orders, key=lambda item: (item[0], item[1], item[2]))
        order = orders.pop(order_index)
        scheduled_order = dict(order)
        scheduled_order["distance"] = distance

        if order_type == "borrow":
            slot_index, power_bank = resource
            scheduled_order["selected_power_bank"] = dict(power_bank)
            slots[slot_index] = None
        elif order_type == "return":
            slot_index = resource
            power_bank = _returned_power_bank(order, len(optimized_route) + 1, status_aliases)
            slots[slot_index] = power_bank
            scheduled_order["returned_power_bank"] = dict(power_bank)

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
        order["power_bank"] = data.get(
            "power_bank", {"id": "PB-RT", "status": "low", "charge": 20}
        )
    return order


def publish_task_result(publisher, task_id, status, note):
    if not task_id:
        return
    message = String()
    message.data = json.dumps(
        {"task_id": str(task_id), "status": status, "note": note}, ensure_ascii=False
    )
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

def go_to_standby(navigator, current_pos):
    """尋找最近的待機點並前往避讓"""
    print("\n💤 進入待機模式，尋找最近的靠牆避讓點...")
    closest_standby = min(
        STANDBY_POINTS, 
        key=lambda pt: math.hypot(pt["x"] - current_pos["x"], pt["y"] - current_pos["y"])
    )
    
    print(f"➡️ 前往待機點：{closest_standby['name']}")
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = closest_standby['x']
    goal_pose.pose.position.y = closest_standby['y']
    goal_pose.pose.orientation = yaw_to_quaternion(closest_standby['yaw'])

    navigator.goToPose(goal_pose)
    while not navigator.isTaskComplete():
        time.sleep(0.1) # 釋放 CPU
        rclpy.spin_once(navigator, timeout_sec=0.05)
        
    print(f"✅ 已靠牆停妥於 {closest_standby['name']}，等待新任務。")
    return {"x": closest_standby['x'], "y": closest_standby['y']}

def main():
    rclpy.init()
    navigator = BasicNavigator()

    print("⏳ 等待 Nav2 系統上線...")
    navigator.waitUntilNav2Active()
    print("✅ Nav2 準備就緒！")

    latest_power_banks = None
    latest_power_status_at = 0.0
    last_power_error_at = 0.0
    task_result_publisher = navigator.create_publisher(
        String, "/smart_carrier/task_result", 10
    )

    def power_status_callback(msg):
        nonlocal latest_power_banks, latest_power_status_at, last_power_error_at
        try:
            latest_power_banks = payload_to_slots(msg.data, require_healthy=True)
            latest_power_status_at = time.monotonic()
        except ValueError as exc:
            now = time.monotonic()
            if now - last_power_error_at >= 10.0:
                navigator.get_logger().error(str(exc))
                last_power_error_at = now

    navigator.create_subscription(String, "power_status", power_status_callback, 10)

    print("⏳ 等待 INA3221 三路電流資料...")
    deadline = time.monotonic() + POWER_STATUS_TIMEOUT_SECONDS
    while latest_power_banks is None and time.monotonic() < deadline:
        rclpy.spin_once(navigator, timeout_sec=0.2)

    if latest_power_banks is None:
        print("❌ 超時未收到 power_status，請確認電源監控節點是否運行。")
        rclpy.shutdown()
        return

    # --- 3. 動態訂單佇列與雲端訂單監聽 ---
    pending_orders = []
    active_task_id = None
    cancellation_requested_task_ids = set()

    def order_callback(msg):
        nonlocal pending_orders
        task_id = None
        try:
            new_order = json.loads(msg.data)
            task_id = new_order.get("id")
            if task_id and str(task_id) in cancellation_requested_task_ids:
                publish_task_result(
                    task_result_publisher,
                    task_id,
                    "cancelled",
                    "Cancelled by administrator before navigation started",
                )
                return
            order_data = order_payload_to_order(new_order)
            if task_id and any(order.get("task_id") == str(task_id) for order in pending_orders):
                navigator.get_logger().warning(f"忽略重複任務：{task_id}")
                return
            pending_orders.append(order_data)
            print(
                f"\n📥 收到雲端新訂單並加入排程："
                f"{order_data['name']} ({order_data['type']})"
            )
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            print(f"\n❌ 訂單解析失敗: {exc}")
            publish_task_result(task_result_publisher, task_id, "failed", str(exc))

    def task_cancel_callback(msg):
        nonlocal pending_orders, active_task_id
        try:
            cancellation = json.loads(msg.data)
            task_id = str(cancellation["task_id"])
            reason = str(cancellation.get("reason") or "Cancelled by administrator")
            cancellation_requested_task_ids.add(task_id)
            if active_task_id == task_id:
                navigator.get_logger().warning(f"正在停止管理員取消的任務：{task_id}")
                return
            for index, order in enumerate(pending_orders):
                if order.get("task_id") == task_id:
                    pending_orders.pop(index)
                    publish_task_result(task_result_publisher, task_id, "cancelled", reason)
                    navigator.get_logger().warning(f"已取消尚未開始導航的任務：{task_id}")
                    return
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            navigator.get_logger().error(f"無效的取消任務訊息：{exc}")

    # 訂閱 cloud_bridge 發出的訂單話題
    navigator.create_subscription(String, "order", order_callback, 10)
    navigator.create_subscription(
        String, "/smart_carrier/task_cancel", task_cancel_callback, 10
    )

    current_pos = {"x": 0.0, "y": 0.0}
    is_standby = False  # 用來記錄目前是否已經在待機點
    last_stale_warning_at = 0.0
    
    print("\n🚀 智慧動態派車系統已啟動！等待接收訂單...")

    # --- 4. 無窮迴圈：讓程式成為常駐服務 ---
    while rclpy.ok():
        # 處理回呼函式 (讓 power_status 和 order 能夠隨時更新)
        rclpy.spin_once(navigator, timeout_sec=0.5)

        if pending_orders:
            now = time.monotonic()
            if now - latest_power_status_at > POWER_STATUS_TIMEOUT_SECONDS:
                if now - last_stale_warning_at >= 10.0:
                    navigator.get_logger().error("power_status 已逾時，暫停派送新任務")
                    last_stale_warning_at = now
                time.sleep(1.0)
                continue

            optimized_route, _, _ = schedule_orders(
                pending_orders, current_pos, latest_power_banks, slot_capacity=3
            )

            if not optimized_route:
                blocked_order = pending_orders.pop(0)
                note = infeasible_order_note(blocked_order)
                navigator.get_logger().warning(
                    f"任務 {blocked_order.get('task_id')} 執行條件已改變，釋放回雲端：{note}"
                )
                publish_task_result(
                    task_result_publisher,
                    blocked_order.get("task_id"),
                    "released",
                    note,
                )
                continue

            # 如果有可執行的訂單，解除待機狀態並準備出發
            is_standby = False
            target = optimized_route[0]

            action = {
                "borrow": "借用",
                "return": "歸還",
                "delivery": "配送",
                "navigation": "導航",
                "callbot": "呼叫機器人",
            }[target["type"]]
            bank = target.get("selected_power_bank") or target.get("returned_power_bank")
            bank_text = f" {bank['id']}" if bank else ""
            print(
                f"\n➡️ 動態決策前往：{target['name']}"
                f"（{action}{bank_text}，預計距離: {target['distance']:.2f}m）"
            )

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = float(target['x'])
            goal_pose.pose.position.y = float(target['y'])
            goal_pose.pose.orientation = yaw_to_quaternion(target['yaw'])

            navigator.goToPose(goal_pose)
            active_task_id = target.get("task_id")
            cancel_sent = False

            # 在導航過程中依然要更新話題，確保能邊走邊接單
            while not navigator.isTaskComplete():
                time.sleep(0.1)
                rclpy.spin_once(navigator, timeout_sec=0.05)
                if active_task_id in cancellation_requested_task_ids and not cancel_sent:
                    navigator.cancelTask()
                    cancel_sent = True

            result = navigator.getResult()
            if active_task_id in cancellation_requested_task_ids:
                print("⚠️ 任務已由管理員取消。")
                result_status = "cancelled"
                result_note = "Cancelled by administrator; navigation stopped"
            elif result == TaskResult.SUCCEEDED:
                if target["type"] == "borrow":
                    msg_txt = "您的行動電源已送達！"
                elif target["type"] == "return":
                    msg_txt = "請將行動電源放入空槽！"
                else:
                    msg_txt = "機器人已抵達指定位置！"
                print(f"🎉 成功抵達 {target['name']}！(語音：{msg_txt})")
                current_pos = {"x": float(target['x']), "y": float(target['y'])}
                result_status = "done"
                result_note = "Nav2 goal reached"
            elif result == TaskResult.CANCELED:
                print("⚠️ 任務取消！")
                result_status = "failed"
                result_note = "Nav2 goal cancelled unexpectedly"
            else:
                print("❌ 導航失敗！")
                result_status = "failed"
                result_note = "Nav2 goal failed"

            publish_task_result(
                task_result_publisher,
                target.get("task_id"),
                result_status,
                result_note,
            )
            if active_task_id:
                cancellation_requested_task_ids.discard(active_task_id)
            active_task_id = None

            # 任務執行完畢，將這筆訂單移除
            target_key = target.get("task_id") or (target["name"], target["type"])
            for index, order in enumerate(pending_orders):
                order_key = order.get("task_id") or (order["name"], order["type"])
                if order_key == target_key:
                    pending_orders.pop(index)
                    break

        else:
            # 當佇列空了，且還沒回到待機點時，執行待機動作
            if not is_standby:
                print("\n🏁 目前無待處理訂單！準備靠牆待機。")
                current_pos = go_to_standby(navigator, current_pos)
                is_standby = True

    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
