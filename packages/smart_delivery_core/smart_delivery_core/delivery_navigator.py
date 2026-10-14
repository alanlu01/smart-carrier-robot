"""Bound lifecycle discovery/RPC without resetting Nav2, tasks or initial pose."""
import json
import time
import rclpy
from lifecycle_msgs.srv import GetState
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String


class Nav2StartupUnavailable(RuntimeError):
    """An unavailable Nav2 must not trap an uninterruptible startup forever."""


class DeliveryNavigator(BasicNavigator):
    """Keep navigation unchanged; replace only lifecycle startup confirmation."""
    STARTUP_TIMEOUT = 30.0
    RPC_TIMEOUT = 3.0
    RETRY_DELAY = 1.0

    def __init__(self):
        super().__init__()
        self.startup_pub = self.create_publisher(String, '/delivery/startup_state',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))

    def _waitForNodeToActivate(self, node_name):
        started = time.monotonic()
        client = future = None
        retry_at = request_at = started
        attempts = 0
        last_status_at = -float('inf')
        previous = None
        phase, error = 'waiting_service', ''
        service = f'{node_name}/get_state'
        try:
            while rclpy.ok() and time.monotonic()-started < self.STARTUP_TIMEOUT:
                now = time.monotonic()
                if client is None and now >= retry_at:
                    try:
                        client = self.create_client(GetState, service)
                        request_at = now
                        phase = 'waiting_service'
                    except Exception as exc:
                        error, phase = str(exc), 'retrying'
                if client is not None:
                    try:
                        if future is None and client.service_is_ready():
                            future = client.call_async(GetState.Request())
                            attempts += 1
                            request_at = now
                            phase = 'waiting_response'
                        if future is not None and future.done():
                            response = future.result()
                            if response is None:
                                raise RuntimeError('empty lifecycle response')
                            if response.current_state.id == 3 and response.current_state.label == 'active':
                                phase = 'ready'
                            else:
                                error = f'lifecycle state={response.current_state.label}'
                                phase = 'waiting_activation'
                        elif now-request_at >= self.RPC_TIMEOUT:
                            error = phase+'_timeout'
                            phase = 'retrying'
                    except Exception as exc:
                        error, phase = str(exc), 'retrying'
                key = (phase, error)
                if key != previous or now-last_status_at >= 5.0:
                    payload = {'state': phase, 'service': service, 'attempts': attempts,
                               'elapsed_sec': round(now-started, 2),
                               'timeout_sec': self.STARTUP_TIMEOUT, 'reason': error}
                    message = String()
                    message.data = json.dumps(payload)
                    self.startup_pub.publish(message)
                    self.get_logger().info('Delivery啟動｜'+message.data)
                    last_status_at, previous = now, key
                if phase == 'ready':
                    return
                if phase in ('retrying', 'waiting_activation'):
                    if future is not None:
                        client.remove_pending_request(future)
                        future.cancel()
                        future = None
                    if client is not None:
                        self.destroy_client(client)
                    client = None
                    retry_at = now+self.RETRY_DELAY
                    phase = 'retry_backoff'
                rclpy.spin_once(self, timeout_sec=.1)
            reason = 'ROS context stopped' if not rclpy.ok() else (error or 'service unavailable')
            message = String()
            message.data = json.dumps({'state': 'failed', 'service': service,
                'attempts': attempts, 'reason': reason, 'timeout_sec': self.STARTUP_TIMEOUT})
            if rclpy.ok():
                self.startup_pub.publish(message)
            raise Nav2StartupUnavailable(
                f'{service} 啟動確認失敗：{reason}；保留訂單，請確認 start_nav 後重開 start_delivery')
        finally:
            if client is not None:
                if future is not None:
                    client.remove_pending_request(future)
                    future.cancel()
                self.destroy_client(client)
