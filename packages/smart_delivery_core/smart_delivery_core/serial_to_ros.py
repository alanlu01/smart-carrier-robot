"""Read bounded serial chunks and recover hotplug without an error storm."""
import json
import time
import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String
from smart_delivery_core.serial_recovery import ReconnectingSerial, FeedbackReadiness


class SerialToROS(Node):
    def __init__(self):
        super().__init__('serial_to_ros')
        self.declare_parameter('serial_port', '/dev/stm32')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('summary_log_period_sec', 5.)
        self.declare_parameter('log_raw_frames', False)
        self.link = ReconnectingSerial(str(self.get_parameter('serial_port').value),
                                     int(self.get_parameter('baud_rate').value), self.get_logger())
        self.summary_period = max(1.,float(self.get_parameter('summary_log_period_sec').value))
        self.log_raw_frames = bool(self.get_parameter('log_raw_frames').value)
        self.publisher_ = self.create_publisher(String, 'stm32_data', 10)
        self.health_pub = self.create_publisher(String, '/chassis/serial_rx_health', 1)
        self.session = ''
        self.pending = bytearray()
        self.feedback = FeedbackReadiness(time.monotonic())
        self.frames = 0
        self.complete_ros_ns = 0
        self.last_summary = time.monotonic()
        self.timer = self.create_timer(.05, self.read_serial)
        self.health_timer = self.create_timer(.2, self.publish_health)

    def read_serial(self):
        now = time.monotonic()
        if not self.link.poll(now):
            self.pending.clear()
            return
        if self.session != self.link.session:
            self.session = self.link.session
            self.pending.clear()
            self.feedback = FeedbackReadiness(now)
            self.complete_ros_ns = 0
            self.frames, self.last_summary = 0,now
        try:
            completed = False
            self.pending.extend(self.link.port.read(min(4096, max(1,self.link.port.in_waiting))))
            if len(self.pending)>8192:
                raise OSError('serial feedback frame buffer overflow')
            for _ in range(32):
                if b'\n' not in self.pending:
                    break
                line, _, rest = self.pending.partition(b'\n')
                self.pending = bytearray(rest)
                text = line.decode(errors='ignore').strip()
                if not text:
                    continue
                before = self.feedback.samples.complete_sample_count
                self.feedback.observe(text,now)
                if self.feedback.samples.complete_sample_count > before:
                    self.complete_ros_ns = self.get_clock().now().nanoseconds
                    completed = True
                if self.log_raw_frames:
                    self.get_logger().debug('STM32: '+text)
                self.publisher_.publish(String(data=text))
                self.frames += 1
            if completed:
                self.publish_health()  # Do not hold a newly completed 2 Hz group until the next heartbeat.
            if now-self.last_summary >= self.summary_period:
                self.get_logger().info(f'📡 STM32 回授：{self.frames/(now-self.last_summary):.1f} Hz')
                self.frames, self.last_summary = 0, now
        except (OSError, serial.SerialException) as exc:
            self.link.fail(exc, now)
            self.pending.clear()

    def publish_health(self):
        now = time.monotonic()
        data = self.link.snapshot(now)
        data.update(self.feedback.snapshot(now))
        data['healthy'] = data['connected'] and data['healthy'] and self.session==self.link.session
        data['sample_ros_stamp_ns'] = self.get_clock().now().nanoseconds
        data['complete_ros_stamp_ns'] = self.complete_ros_ns
        self.health_pub.publish(String(data=json.dumps(data)))

    def destroy_node(self):
        self.link.close()
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


if __name__=='__main__':main()
