import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from power_monitor.power_status import payload_to_slots
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from smart_delivery_core.service_locations import LOCATION_DB, STANDBY_POINTS

BORROW_DISTANCE_WEIGHT = 0.7
POWER_STATUS_TIMEOUT_SECONDS = 10.0
LEASE_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# 行動電源狀態介面的預設數字值
POWER_BANK_STATUS_CODES = {
    0: "empty",  
    1: "low",  
    2: "ready",  
    3: "full",  
}

DEFAULT_STATUS_ALIASES = {
    "EMPTY": "empty",
    "LOW": "low",
    "READY": "ready",
    "FULL": "full",
    "AVAILABLE": "ready",
    "CHARGING": "low",
    "ALMOST_FULL": "ready",
    "NOT_INSERTED": "empty",
    "空": "empty",
    "低電量": "low",
    "就緒": "ready",
    "全滿": "full",
}

BORROWABLE_STATUS_PRIORITY = {
    "full": 0,
    "ready": 1,
}

def power_status_payload_to_banks(payload):
    try:
        slots = payload_to_slots(payload, require_healthy=True)
        return [
            {
                **slot,
                "id": slot.get("bank_id") or f"PB-{slot['slot']:02d}",
            }
            for slot in slots
        ]
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise ValueError(f"無效的 power_status 數據：{exc}") from exc

def normalize_power_bank_status(status, status_aliases=None):
    if status_aliases and status in status_aliases:
        status = status_aliases[status]
    if isinstance(status, int) and not isinstance(status, bool):
        try: return POWER_BANK_STATUS_CODES[status]
        except KeyError as exc: raise ValueError(f"未知的行動電源狀態碼：{status}") from exc

    aliases = dict(DEFAULT_STATUS_ALIASES)
    if status_aliases: aliases.update(status_aliases)
    key = str(status).strip()
    normalized = aliases.get(key, aliases.get(key.upper(), key.lower()))
    if normalized not in POWER_BANK_STATUS_CODES.values():
        raise ValueError(f"未知的行動電源狀態：{status}")
    return normalized

def select_power_bank(power_bank_slots, required_charge=0, status_aliases=None):
    if not 0 <= required_charge <= 100: raise ValueError("required_charge 必須介於 0 到 100")
    candidates = []
    for slot_index, power_bank in enumerate(power_bank_slots):
        if power_bank is None: continue
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
                selection = select_power_bank(slots, float(order.get("required_charge", 0)), status_aliases)
                if selection is not None:
                    feasible_orders.append((distance * borrow_distance_weight, distance, order_index, "borrow", selection))
            elif order_type == "return":
                try: empty_slot = slots.index(None)
                except ValueError: continue
                feasible_orders.append((distance, distance, order_index, "return", empty_slot))

        if not feasible_orders: break

        priority_score, distance, order_index, order_type, resource = min(feasible_orders, key=lambda item: (item[0], item[1], item[2]))
        order = orders.pop(order_index)
        scheduled_order = dict(order)
        scheduled_order["distance"] = distance

        if order_type == "borrow":
            slot_index, power_bank = resource
            scheduled_order["selected_power_bank"] = dict(power_bank)
            slots[slot_index] = None
        else:
            slot_index = resource
            power_bank = _returned_power_bank(order, len(optimized_route) + 1, status_aliases)
            slots[slot_index] = power_bank
            scheduled_order["returned_power_bank"] = dict(power_bank)

        optimized_route.append(scheduled_order)
        position = {"x": float(scheduled_order["x"]), "y": float(scheduled_order["y"])}

    return optimized_route, orders, slots

def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

def go_to_standby(navigator, current_pos, set_navigation_active=lambda _active: None):
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

    set_navigation_active(True)
    try:
        navigator.goToPose(goal_pose)
        while not navigator.isTaskComplete():
            time.sleep(0.1) # 釋放 CPU
            rclpy.spin_once(navigator, timeout_sec=0.05)
        result = navigator.getResult()
    finally:
        set_navigation_active(False)

    if result != TaskResult.SUCCEEDED:
        navigator.get_logger().warning(
            f"前往待機點 {closest_standby['name']} 未成功（result={result}）；"
            "保留實際位置並稍後重試"
        )
        return None

    print(f"✅ 已靠牆停妥於 {closest_standby['name']}，等待新任務。")
    return {"x": closest_standby['x'], "y": closest_standby['y']}

def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigation_lease_publisher = navigator.create_publisher(
        Bool, "/smart_carrier/delivery_navigation_active", LEASE_QOS
    )
    delivery_navigation_active = False

    def publish_navigation_lease():
        if not delivery_navigation_active:
            return
        message = Bool()
        message.data = True
        navigation_lease_publisher.publish(message)

    def set_navigation_active(active):
        nonlocal delivery_navigation_active
        delivery_navigation_active = bool(active)
        message = Bool()
        message.data = delivery_navigation_active
        navigation_lease_publisher.publish(message)

    navigator.create_timer(0.2, publish_navigation_lease)

    print("⏳ 等待 Nav2 系統上線...")
    navigator.waitUntilNav2Active()
    print("✅ Nav2 準備就緒！")

    latest_power_banks = None

    def power_status_callback(msg):
        nonlocal latest_power_banks
        try: latest_power_banks = power_status_payload_to_banks(msg.data)
        except ValueError as exc: navigator.get_logger().error(str(exc))

    navigator.create_subscription(String, "power_status", power_status_callback, 10)

    print("⏳ 等待 INA3221 三路電流資料...")
    deadline = time.monotonic() + POWER_STATUS_TIMEOUT_SECONDS
    while latest_power_banks is None and time.monotonic() < deadline:
        rclpy.spin_once(navigator, timeout_sec=0.2)

    if latest_power_banks is None:
        print("❌ 超時未收到 power_status，請確認電源監控節點是否運行。")
        rclpy.shutdown()
        return

    pending_orders = [
        # {
        #     "name": "右樓梯口", "type": "return",
        #     "power_bank": {"id": "PB-03", "status": "low", "charge": 20},
        #     "x": 7.7, "y": -7.2, "yaw": 0.0,
        # },
        {
            "name": "左樓梯口", "type": "return",
            "power_bank": {"id": "PB-04", "status": "low", "charge": 20},
            **LOCATION_DB["左樓梯口"],
        },
        {
            "name": "電機工程學系", "type": "borrow", "required_charge": 90,
            **LOCATION_DB["電機工程學系"],
        },
    ]

    current_pos = {"x": 0.0, "y": 0.0}
    print("\n🚀 開始執行智慧動態派車系統！")

    # --- 修改亮點 1：動態派車迴圈 (Dynamic Dispatch Loop) ---
    while pending_orders:
        # 確保在每一輪排程前，都更新到最新的感測器資料
        rclpy.spin_once(navigator, timeout_sec=0.1)

        optimized_route, deferred_orders, _ = schedule_orders(
            pending_orders, current_pos, latest_power_banks, slot_capacity=3
        )

        if not optimized_route:
            print("⚠️ 目前無可執行的訂單 (可能因槽位或庫存不足)。")
            break # 跳出迴圈，準備進入待機

        # 只取最新算出的「第一站」去執行
        target = optimized_route[0]
        
        action = "借用" if target["type"] == "borrow" else "歸還"
        bank = target.get("selected_power_bank") or target.get("returned_power_bank")
        print(f"\n➡️ 動態決策前往：{target['name']}（{action} {bank['id']}，預計距離: {target['distance']:.2f}m）")

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(target['x'])
        goal_pose.pose.position.y = float(target['y'])
        goal_pose.pose.orientation = yaw_to_quaternion(target['yaw'])

        set_navigation_active(True)
        navigator.goToPose(goal_pose)

        # --- 修改亮點 2：加入 time.sleep(0.1) 拯救樹莓派 CPU ---
        while not navigator.isTaskComplete():
            time.sleep(0.1)
            # 在導航過程中持續更新 Topic 資料
            rclpy.spin_once(navigator, timeout_sec=0.05)

        set_navigation_active(False)
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            msg = "您的行動電源已送達！" if target["type"] == "borrow" else "請將行動電源放入空槽！"
            print(f"🎉 成功抵達 {target['name']}！(語音：{msg})")
            
            # 抵達後更新目前位置，作為下一輪排程的起點
            current_pos = {"x": float(target['x']), "y": float(target['y'])}
        elif result == TaskResult.CANCELED:
            print(f"⚠️ 任務取消！")
        elif result == TaskResult.FAILED:
            print(f"❌ 導航失敗！")

        # 任務執行完畢，將這筆訂單從未完成清單中移除
        pending_orders = [o for o in pending_orders if o['name'] != target['name']]

    # 迴圈結束：可能是所有訂單送完，也可能是全部變成 deferred_orders
    if pending_orders:
        names = "、".join(o["name"] for o in pending_orders)
        print(f"\n🏁 還有未完成的暫緩訂單等待庫存更新：{names}")
    else:
        print("\n🏁 所有訂單配送完畢！")

    # --- 呼叫靠牆避讓待機函式 ---
    current_pos = go_to_standby(navigator, current_pos, set_navigation_active)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
