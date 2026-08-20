import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import String

import json
import math
import time

BORROW_DISTANCE_WEIGHT = 0.7
POWER_STATUS_TIMEOUT_SECONDS = 10.0
EMPTY_CURRENT_A = 0.0005
READY_CURRENT_MIN_A = 0.05
LOW_CURRENT_MIN_A = 0.4

# --- 1. 預先定義的靠牆待機點 ---
STANDBY_POINTS = [
    {"name": "門口", "x": -0.45, "y": -0.35, "yaw": 0.0},
    {"name": "走廊轉角", "x": 5.0, "y": 0.0, "yaw": -1.57}
]

# --- 2. 固定的配送座標清單 (查表法資料庫) ---
LOCATION_DB = {
    "門口": {"x": -0.45, "y": -0.35, "yaw": 0.0},
    "走廊轉角": {"x": 5.0, "y": 0.0, "yaw": -1.57},
    "廚房": {"x": -1.5, "y": 5.0, "yaw": 0.0},
    "廁所": {"x": -4.0, "y": 1.5, "yaw": 3.14},
    "客廳TV": {"x": 0.4, "y": 0.0, "yaw": 0.0},
}

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

def classify_current_status(current_a):
    abs_current = abs(float(current_a))
    if abs_current <= EMPTY_CURRENT_A: return "empty"
    if abs_current >= LOW_CURRENT_MIN_A: return "low"
    if abs_current >= READY_CURRENT_MIN_A: return "ready"
    return "full"

def estimate_charge_from_current(current_a, status):
    status = normalize_power_bank_status(status)
    current_a = abs(float(current_a))

    if status == "empty": return 0
    if status == "full": return 100
    if status == "low": return min(79, max(1, round(80 * (1.0 - min(current_a, 1.0)))))

    ready_ratio = (0.4 - min(max(current_a, 0.05), 0.4)) / 0.35
    return round(80 + ready_ratio * 19)

def power_status_payload_to_banks(payload):
    try:
        power_data = json.loads(payload) if isinstance(payload, str) else payload
        power_banks = []
        for channel_number in range(1, 4):
            channel = power_data[f"ch{channel_number}"]
            current_a = float(channel["current"])
            status = classify_current_status(current_a)
            power_banks.append({
                "id": f"PB-{channel_number:02d}",
                "status": status,
                "charge": estimate_charge_from_current(current_a, status),
                "current": current_a,
            })
        return power_banks
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

    # --- 3. 動態訂單佇列與雲端訂單監聽 ---
    pending_orders = []

    def order_callback(msg):
        nonlocal pending_orders
        try:
            new_order = json.loads(msg.data)
            loc_name = new_order.get("name")
            
            # 查表比對座標
            if loc_name in LOCATION_DB:
                order_data = {
                    "name": loc_name,
                    "type": new_order.get("type", "borrow"),
                    "required_charge": new_order.get("required_charge", 0),
                    "x": LOCATION_DB[loc_name]["x"],
                    "y": LOCATION_DB[loc_name]["y"],
                    "yaw": LOCATION_DB[loc_name]["yaw"]
                }
                
                # 如果是歸還訂單，補上虛擬的行動電源狀態資訊，以符合演算法需求
                if order_data["type"] == "return":
                    order_data["power_bank"] = new_order.get("power_bank", {"id": "PB-RT", "status": "low", "charge": 20})
                
                pending_orders.append(order_data)
                print(f"\n📥 收到雲端新訂單並加入排程：{loc_name} ({order_data['type']})")
            else:
                print(f"\n❌ 收到未知地點的訂單，查無座標：{loc_name}")
        except Exception as e:
            print(f"\n❌ 訂單解析失敗: {e}")

    # 訂閱 cloud_bridge 發出的訂單話題
    navigator.create_subscription(String, "order", order_callback, 10)

    current_pos = {"x": 0.0, "y": 0.0}
    is_standby = False  # 用來記錄目前是否已經在待機點
    
    print("\n🚀 智慧動態派車系統已啟動！等待接收訂單...")

    # --- 4. 無窮迴圈：讓程式成為常駐服務 ---
    while rclpy.ok():
        # 處理回呼函式 (讓 power_status 和 order 能夠隨時更新)
        rclpy.spin_once(navigator, timeout_sec=0.5)

        if pending_orders:
            optimized_route, deferred_orders, _ = schedule_orders(
                pending_orders, current_pos, latest_power_banks, slot_capacity=3
            )

            if not optimized_route:
                if not is_standby:
                    print("\n⚠️ 訂單因槽位或庫存不足暫緩。前往待機點充電等待...")
                    current_pos = go_to_standby(navigator, current_pos)
                    is_standby = True
                
                # 已經在待機點了，就休眠一下等待電量更新，避免狂刷螢幕
                time.sleep(1.0)
                continue 

            # 如果有可執行的訂單，解除待機狀態並準備出發
            is_standby = False
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

            navigator.goToPose(goal_pose)

            # 在導航過程中依然要更新話題，確保能邊走邊接單
            while not navigator.isTaskComplete():
                time.sleep(0.1)
                rclpy.spin_once(navigator, timeout_sec=0.05)

            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                msg_txt = "您的行動電源已送達！" if target["type"] == "borrow" else "請將行動電源放入空槽！"
                print(f"🎉 成功抵達 {target['name']}！(語音：{msg_txt})")
                current_pos = {"x": float(target['x']), "y": float(target['y'])}
            elif result == TaskResult.CANCELED:
                print(f"⚠️ 任務取消！")
            elif result == TaskResult.FAILED:
                print(f"❌ 導航失敗！")

            # 任務執行完畢，將這筆訂單移除
            pending_orders = [o for o in pending_orders if o['name'] != target['name']]

        else:
            # 當佇列空了，且還沒回到待機點時，執行待機動作
            if not is_standby:
                print("\n🏁 目前無待處理訂單！準備靠牆待機。")
                current_pos = go_to_standby(navigator, current_pos)
                is_standby = True

    rclpy.shutdown()

if __name__ == '__main__':
    main()
