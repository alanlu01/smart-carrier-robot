"""Bound lifecycle discovery/RPC without resetting Nav2, tasks or initial pose."""
import json
import time
import rclpy
from lifecycle_msgs.srv import GetState
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from smart_delivery_core.delivery_console import startup_status
from ament_index_python.packages import get_package_share_directory
from smart_delivery_core.navigation_policy import navigation_policy
from smart_delivery_core.delivery_inputs import shared_inputs, message_json
from std_srvs.srv import Trigger


class Nav2StartupUnavailable(RuntimeError):
    """An unavailable Nav2 must not trap an uninterruptible startup forever."""


class DeliveryNavigator(BasicNavigator):
    """Bound startup while retaining BasicNavigator's executor-driven actions."""
    STARTUP_TIMEOUT = 30.0
    RPC_TIMEOUT = 3.0
    RETRY_DELAY = 1.0

    def __init__(self):
        super().__init__()
        self.policy_share = get_package_share_directory('smart_delivery_core')
        self.policy_pub = self.create_publisher(String, '/delivery/navigation_policy',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.startup_pub = self.create_publisher(String, '/delivery/startup_state',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.action_health=None
        self.action_health_at=-float('inf')
        self.action_request=None
        self.action_request_at=0.
        self.action_request_warned=False
        self.action_requested_key=None
        self.action_recovery_client=self.create_client(Trigger,'/navigation/recover_actions')
        shared_inputs(self).subscribe(String,'/navigation/pipeline_health',self._action_health,1)
        self.create_timer(.5,self._poll_action_recovery)

    def _action_health(self, message):
        try:
            payload=message_json(message)
            age=(self.get_clock().now().nanoseconds-payload['sample_ros_stamp_ns'])/1e9
            if -.1<=age<=3. and isinstance(payload.get('action'),dict):
                self.action_health,self.action_health_at=payload['action'],time.monotonic()
        except (KeyError,ValueError,TypeError,AttributeError):pass

    def navigation_actions_ready(self):
        # Initial discovery still uses the existing Nav2/localization startup
        # gates. Once observed, missing heartbeats never clear an action fault.
        return (self.action_health is None or (time.monotonic()-self.action_health_at<=3.
                and self.action_health.get('state')=='healthy'))

    def _poll_action_recovery(self):
        if self.action_request is not None:
            if not self.action_request.done():
                if time.monotonic()-self.action_request_at>3. and not self.action_request_warned:
                    self.action_request_warned=True
                    self.get_logger().warning('⚠️ 導航恢復請求未回應，暫留原地；不重複重置')
                return
            try:self.action_request.result()
            except Exception:pass
            self.action_request=None
        status=self.action_health or {}
        key=(status.get('session'),status.get('fault_sequence'))
        if (status.get('state')=='fault' and key!=self.action_requested_key
                and time.monotonic()-self.action_health_at<=3.
                and self.action_recovery_client.service_is_ready()):
            self.action_requested_key=key
            self.action_request=self.action_recovery_client.call_async(Trigger.Request())
            self.action_request_at=time.monotonic()
            self.action_request_warned=False
            self.get_logger().warning('⏸️ 導航回應異常，停妥後嘗試恢復；保留任務')

    def configure_navigation_goal(self, pose, behavior_tree=''):
        selected = navigation_policy(pose, behavior_tree, self.policy_share)
        if selected is not None:
            behavior_tree, policy = selected
            self.policy_pub.publish(String(data=policy))
        return behavior_tree

    def _waitForNodeToActivate(self, node_name):
        started = time.monotonic()
        client = future = None
        retry_at = request_at = started
        attempts = 0
        last_status_at = -float('inf')
        previous = None
        phase, error = 'waiting_service', ''
        service = f'{node_name}/get_state'
        providers = None
        graph_at = -float('inf')
        last_console_at = -float('inf')
        previous_console = None
        try:
            while rclpy.ok() and time.monotonic()-started < self.STARTUP_TIMEOUT:
                now = time.monotonic()
                if now-graph_at >= 2.:
                    graph_at = now
                    try:
                        namespace, _, leaf = ('/'+node_name.strip('/')).rpartition('/')
                        providers = sum(name == leaf and ns == (namespace or '/')
                                        for name, ns in self.get_node_names_and_namespaces())
                    except (AttributeError, RuntimeError):
                        providers = None
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
                            if providers is not None and providers > 1:
                                raise RuntimeError(f'duplicate Nav2 nodes: {node_name}, count={providers}')
                            if response.current_state.id == 3 and response.current_state.label == 'active':
                                error = ''
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
                               'timeout_sec': self.STARTUP_TIMEOUT, 'reason': error,
                               'matching_node_count': providers,
                               'hint': ('Nav2 尚未 active；請檢查 start_nav，勿重複啟動'
                                        if phase == 'waiting_activation' else
                                        '若有重複 Nav2／同名服務，請先正常結束舊程序'
                                        if providers and providers > 1 else '')}
                    message = String()
                    message.data = json.dumps(payload)
                    self.startup_pub.publish(message)
                    text = startup_status(phase,error,bool(providers and providers>1))
                    if text != previous_console or now-last_console_at >= 5.:
                        self.get_logger().info(text)
                        previous_console,last_console_at = text,now
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
                self.get_logger().info(startup_status('failed',reason))
            raise Nav2StartupUnavailable(
                f'{service} 啟動確認失敗：{reason}；保留訂單，請確認 start_nav 後重開 start_delivery')
        finally:
            if client is not None:
                if future is not None:
                    client.remove_pending_request(future)
                    future.cancel()
                self.destroy_client(client)
