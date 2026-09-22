import time

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String


class SerialToROS(Node):
    """Publish STM32 feedback without logging every high-rate frame."""

    def __init__(self):
        super().__init__("serial_to_ros")
        self.declare_parameter("serial_port", "/dev/stm32")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("summary_log_period_sec", 5.0)
        self.declare_parameter("log_raw_frames", False)
        serial_port = str(self.get_parameter("serial_port").value)
        baud_rate = int(self.get_parameter("baud_rate").value)
        self.summary_log_period = max(
            1.0, float(self.get_parameter("summary_log_period_sec").value)
        )
        self.log_raw_frames = bool(self.get_parameter("log_raw_frames").value)

        self.publisher_ = self.create_publisher(String, "stm32_data", 10)
        self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
        self.timer = self.create_timer(0.05, self.read_serial)
        self.frames_since_summary = 0
        self.latest_frame = ""
        self.last_summary_at = time.monotonic()
        self.get_logger().info(
            f"STM32 serial feedback ready: port={serial_port}, baud={baud_rate}"
        )

    def read_serial(self):
        try:
            line = self.ser.readline().decode(errors="ignore").strip()
            if not line:
                return
            msg = String()
            msg.data = line
            self.publisher_.publish(msg)
            self.frames_since_summary += 1
            self.latest_frame = line
            if self.log_raw_frames:
                self.get_logger().debug(f"Publish: {line}")
            now = time.monotonic()
            if now - self.last_summary_at >= self.summary_log_period:
                elapsed = max(0.001, now - self.last_summary_at)
                self.get_logger().info(
                    "STM32 feedback: "
                    f"{self.frames_since_summary / elapsed:.1f} Hz, "
                    f"latest={self.latest_frame}"
                )
                self.frames_since_summary = 0
                self.last_summary_at = now
        except (OSError, UnicodeError, serial.SerialException) as exc:
            self.get_logger().error(f"STM32 serial read failed: {exc}")

    def destroy_node(self):
        if self.ser.is_open:
            self.ser.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialToROS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
