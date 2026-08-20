import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import smbus2
import collections
import json


def decode_shunt_current(raw_val, current_lsb=0.0004):
    """解析 INA3221 16-bit 二補數分流電流寄存器。"""
    if not 0 <= raw_val <= 0xFFFF:
        raise ValueError("raw_val 必須是 16-bit 無號整數")

    signed_raw = raw_val - 0x10000 if raw_val & 0x8000 else raw_val
    shunt_steps = signed_raw >> 3
    return shunt_steps * current_lsb


class INA3221Node(Node):
    def __init__(self):
        super().__init__('ina3221_node')
        
        # 建立 Publisher
        self.publisher_ = self.create_publisher(String, 'power_status', 10)
        
        # 設定 Timer，每 0.5 秒執行一次
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # INA3221 硬體設定
        self.I2C_BUS = 1
        self.I2C_ADDR = 0x40
        self.CURRENT_LSB = 0.0004
        self.bus = smbus2.SMBus(self.I2C_BUS)
        
        # 移動平均濾波器設定 (取最近 6 筆資料，約 3 秒的平均)
        window_size = 6
        self.ch1_history = collections.deque(maxlen=window_size)
        self.ch2_history = collections.deque(maxlen=window_size)
        self.ch3_history = collections.deque(maxlen=window_size)
        
        self.get_logger().info("INA3221 電源監控節點已啟動 (5V/1A 專用版)！")

    def read_raw_current(self, reg_addr):
        try:
            data = self.bus.read_i2c_block_data(self.I2C_ADDR, reg_addr, 2)
            raw_val = (data[0] << 8) | data[1]
            return decode_shunt_current(raw_val, self.CURRENT_LSB)
        except Exception as e:
            self.get_logger().error(f"I2C 讀取錯誤: {e}")
            return 0.0

    def timer_callback(self):
        # 1. 讀取原始數據並加入移動平均佇列
        self.ch1_history.append(self.read_raw_current(0x01))
        self.ch2_history.append(self.read_raw_current(0x03))
        self.ch3_history.append(self.read_raw_current(0x05))
        
        # 2. 計算平均值
        avg_ch1 = sum(self.ch1_history) / len(self.ch1_history) if self.ch1_history else 0.0
        avg_ch2 = sum(self.ch2_history) / len(self.ch2_history) if self.ch2_history else 0.0
        avg_ch3 = sum(self.ch3_history) / len(self.ch3_history) if self.ch3_history else 0.0

        # 3. INA 節點只發布原始電流，分類與排程由 smart_delivery.py 負責。
        self.get_logger().info(
            f"\n"
            f"=== INA3221 三路電流即時監控 ===\n"
            f"[通道 1] 電流: {avg_ch1:5.3f} A\n"
            f"[通道 2] 電流: {avg_ch2:5.3f} A\n"
            f"[通道 3] 電流: {avg_ch3:5.3f} A\n"
            f"=============================="
        )

        # 4. 只打包三路原始電流，避免感測節點與業務分類重複。
        power_data = {
            "ch1": {"current": round(avg_ch1, 3)},
            "ch2": {"current": round(avg_ch2, 3)},
            "ch3": {"current": round(avg_ch3, 3)}
        }
        msg = String()
        msg.data = json.dumps(power_data)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = INA3221Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
