#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import sys

class CmdVelToSerialNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_serial_node')
        
        # --- 1. 序列埠 (Serial) 設定 ---
        # 🌟 這裡換成我們剛剛綁定的專屬 udev 名稱
        self.serial_port_name = '/dev/stm32' 
        self.baud_rate = 115200 
        
        try:
            self.serial_port = serial.Serial(self.serial_port_name, self.baud_rate, timeout=1)
            self.get_logger().info(f"✅ 成功連接到 STM32 序列埠: {self.serial_port_name}")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ 無法開啟序列埠 {self.serial_port_name}，錯誤: {e}")
            sys.exit(1)

        # --- 2. 訂閱者 (Subscriber) 設定 ---
        self.subscription = self.create_subscription(
            Twist,
            '/chassis_cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info("🎧 正在監聽 /cmd_vel 三軸速度指令 (Vx, Vy, W)...")

    def cmd_vel_callback(self, msg):
        # 提取出 線速度 (vx, vy) 與 角速度 (w)
        vx = msg.linear.x
        vy = msg.linear.y  # 👈 麥克納姆輪專屬的橫向平移速度
        w = msg.angular.z
        
        # --- 3. 封包格式化 (加入 vy) ---
        # 格式: $CMD,vx,vy,w#\n
        command_str = f"$CMD,{vx:.2f},{vy:.2f},{w:.2f}#\n"
        
        # --- 4. 發送至 STM32 ---
        try:
            self.serial_port.write(command_str.encode('utf-8'))
            # 🌟 將 info 改為 debug，避免遙控時狂刷終端機導致系統卡頓
            self.get_logger().debug(f"📤 發送指令: {command_str.strip()}")
        except Exception as e:
            self.get_logger().error(f"⚠️ 發送失敗: {e}")

    def destroy_node(self):
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("🔒 序列埠已安全關閉。")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
