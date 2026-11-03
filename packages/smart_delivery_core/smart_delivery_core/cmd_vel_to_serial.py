"""Reconnect with zero speed and fresh stopped-wheel evidence before motion."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from smart_delivery_core.serial_recovery import ReconnectingSerial, fresh_feedback

ZERO = b'$CMD,0.00,0.00,0.00#\n'


class CmdVelToSerialNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_serial_node')
        self.declare_parameter('serial_port','/dev/stm32')
        self.declare_parameter('baud_rate',115200)
        self.link = ReconnectingSerial(str(self.get_parameter('serial_port').value),
                                     int(self.get_parameter('baud_rate').value),self.get_logger())
        self.session = self.rx_session = ''
        self.feedback, self.feedback_at = {}, 0.
        self.armed = False
        self.last_command = -float('inf')
        self.last_zero = -float('inf')
        self.opened_ros_ns = 0
        self.rejected_commands = 0
        self.accepted_commands = 0
        self.arming_count = 0
        self.health_pub = self.create_publisher(String,'/chassis/serial_tx_health',1)
        self.subscription = self.create_subscription(Twist,'/chassis_cmd_vel',self.cmd_vel_callback,1)
        self.create_subscription(String,'/chassis/serial_rx_health',self.feedback_callback,1)
        self.timer = self.create_timer(.05,self.poll)
        self.create_timer(.2,self.publish_health)

    def feedback_callback(self, message):
        try:
            payload = json.loads(message.data)
            if not isinstance(payload,dict):
                raise ValueError('feedback must be an object')
            self.feedback, self.feedback_at = payload,time.monotonic()
        except (ValueError,TypeError):
            self.feedback = {}

    def feedback_ready(self, now):
        return fresh_feedback(self.feedback,self.get_clock().now().nanoseconds,now-self.feedback_at)

    def zero(self, now):
        if self.link.port is not None and now-self.last_zero >= .15:
            self.link.write(ZERO,now)
            self.last_zero = now

    def poll(self):
        now = time.monotonic()
        if not self.link.poll(now):
            self.armed = False
            return
        if self.session != self.link.session:
            self.session = self.link.session
            self.armed = False
            self.opened_ros_ns = self.get_clock().now().nanoseconds
            self.last_zero = -float('inf')
            self.zero(now)
            if self.link.port is None:
                return
        ready = self.feedback_ready(now)
        if not ready or self.rx_session != self.feedback.get('session'):
            if self.armed:
                self.last_zero = -float('inf')
            self.armed = False
            self.rx_session = self.feedback.get('session','')
        if (not self.armed and ready and self.feedback.get('stop_proven') is True
                and self.feedback['sample_ros_stamp_ns'] > self.opened_ros_ns):
            if self.feedback['complete_ros_stamp_ns'] <= self.opened_ros_ns:
                self.zero(now)
                return
            self.armed = True
            self.arming_count += 1
            self.get_logger().info('✅ STM32 新鮮回授及停車確認完成')
        if not self.armed or now-self.last_command > .25:
            self.zero(now)

    def cmd_vel_callback(self, message, info=None):
        now = time.monotonic()
        values = (message.linear.x,message.linear.y,message.angular.z)
        source = getattr(info,'source_timestamp',0) if info is not None else 0
        source_ok = info is None or source > 0 and -.1 <= (self.get_clock().now().nanoseconds-source)/1e9 <= .25
        if (not self.armed or self.rx_session != self.feedback.get('session')
                or not self.feedback_ready(now) or not source_ok
                or not all(math.isfinite(v) for v in values)):
            self.rejected_commands += 1
            self.zero(now)
            return
        payload = f'$CMD,{values[0]:.2f},{values[1]:.2f},{values[2]:.2f}#\n'.encode()
        if self.link.write(payload,now):
            self.last_command = now
            self.accepted_commands += 1
        else:
            self.armed = False

    def publish_health(self):
        now = time.monotonic()
        data = self.link.snapshot(now)
        data.update(sample_ros_stamp_ns=self.get_clock().now().nanoseconds,
                    armed=self.armed and self.feedback_ready(now),rx_session=self.rx_session,
                    rejected_commands=self.rejected_commands,
                    accepted_commands=self.accepted_commands,
                    arming_count=self.arming_count,
                    state='ready' if self.armed and self.feedback_ready(now) else 'waiting_for_safe_feedback')
        self.health_pub.publish(String(data=json.dumps(data)))

    def destroy_node(self):
        self.link.write(ZERO,time.monotonic())
        self.link.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
