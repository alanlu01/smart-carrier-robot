import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import smbus2
import collections
import json

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
            is_negative = (raw_val & 0x8000) != 0
            adc_val = (raw_val >> 3) & 0x0FFF
            
            current = adc_val * self.CURRENT_LSB
            return -current if is_negative else current
        except Exception as e:
            self.get_logger().error(f"I2C 讀取錯誤: {e}")
            return 0.0

    def classify_status(self, current_a):
        # 根據 5V/1A 的充電特性重新定義門檻
        abs_c = abs(current_a)
        if abs_c >= 0.4: 
            return "charging"
        elif 0.05 <= abs_c < 0.4: 
            return "almost_full"
        else: 
            return "not_inserted"

    def status_to_zh(self, status):
        # 將英文狀態轉換為中文，方便終端機顯示
        mapping = {
            "charging": "充電中",
            "almost_full": "幾乎滿電",
            "not_inserted": "未插入行充"
        }
        return mapping.get(status, "未知狀態")

    def timer_callback(self):
        # 1. 讀取原始數據並加入移動平均佇列
        self.ch1_history.append(self.read_raw_current(0x01))
        self.ch2_history.append(self.read_raw_current(0x03))
        self.ch3_history.append(self.read_raw_current(0x05))
        
        # 2. 計算平均值
        avg_ch1 = sum(self.ch1_history) / len(self.ch1_history) if self.ch1_history else 0.0
        avg_ch2 = sum(self.ch2_history) / len(self.ch2_history) if self.ch2_history else 0.0
        avg_ch3 = sum(self.ch3_history) / len(self.ch3_history) if self.ch3_history else 0.0

        # 3. 取得分類狀態
        stat1 = self.classify_status(avg_ch1)
        stat2 = self.classify_status(avg_ch2)
        stat3 = self.classify_status(avg_ch3)

        # 4. 直接印出排版好的資訊到終端機
        self.get_logger().info(
            f"\n"
            f"=== 行充租借站狀態即時監控 ===\n"
            f"[行充 1] 電流: {avg_ch1:5.3f} A | 狀態: {self.status_to_zh(stat1)}\n"
            f"[行充 2] 電流: {avg_ch2:5.3f} A | 狀態: {self.status_to_zh(stat2)}\n"
            f"[行充 3] 電流: {avg_ch3:5.3f} A | 狀態: {self.status_to_zh(stat3)}\n"
            f"=============================="
        )

        # 5. 將數據打包發布 (維持英文狀態標籤，方便核心系統寫 if/else 判斷)
        power_data = {
            "ch1": {"current": round(avg_ch1, 3), "status": stat1},
            "ch2": {"current": round(avg_ch2, 3), "status": stat2},
            "ch3": {"current": round(avg_ch3, 3), "status": stat3}
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
