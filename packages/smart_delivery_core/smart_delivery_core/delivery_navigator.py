"""Bound lifecycle discovery/RPC without resetting Nav2, tasks or initial pose."""
import json
import time
import rclpy
from lifecycle_msgs.srv import GetState
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from smart_delivery_core.delivery_console import startup_status
from smart_delivery_core.navigation_wait import NavigationWait, NavigationOwnershipUncertain
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class Nav2StartupUnavailable(RuntimeError):
    """An unavailable Nav2 must not trap an uninterruptible startup forever."""


class DeliveryNavigator(BasicNavigator):
    """Bound startup and poll owned actions without nesting executor waits."""
    STARTUP_TIMEOUT = 30.0
    RPC_TIMEOUT = 3.0
    RETRY_DELAY = 1.0

    def goToPose(self, pose, behavior_tree=''):
        previous = getattr(self,'_delivery_wait',None)
        if previous is not None and previous.phase not in ('finished','rejected'):
            raise RuntimeError('Previous navigation action has not drained')
        if getattr(self,'result_future',None) is not None and not self.result_future.done():
            raise NavigationOwnershipUncertain('Auxiliary navigation action has not drained')
        self._delivery_wait = NavigationWait(time.monotonic())
        self._delivery_pose, self._delivery_tree = pose, behavior_tree
        self.goal_handle = self.result_future = None
        self.status = GoalStatus.STATUS_UNKNOWN
        self.pending_goal_request = None
        return True

    def isTaskComplete(self):
        wait = getattr(self,'_delivery_wait',None)
        if wait is not None and wait.phase not in ('finished','rejected'):
            now = time.monotonic()
            if wait.request is None and self.nav_to_pose_client.server_is_ready():
                goal = NavigateToPose.Goal()
                goal.pose, goal.behavior_tree = self._delivery_pose, self._delivery_tree
                wait.request = self.nav_to_pose_client.send_goal_async(goal,self._feedbackCallback)
                self.pending_goal_request = wait.request
                wait.phase = 'waiting_acceptance'
            done = wait.poll(now)
            self.goal_handle, self.result_future = wait.handle, wait.result
            if done:
                self.pending_goal_request = None
                self.status = (GoalStatus.STATUS_ABORTED if wait.phase == 'rejected'
                               else wait.result.result().status)
            return done
        # Auxiliary motions own their request/deadline; only inspect their result.
        result = getattr(self,'result_future',None)
        if result is None:return True
        if not result.done():return False
        self.status = result.result().status
        return True

    def cancelTask(self):
        wait = getattr(self,'_delivery_wait',None)
        if wait is not None and wait.phase not in ('finished','rejected'):
            if wait.request is None:
                wait.phase = 'finished'
                self.status = GoalStatus.STATUS_CANCELED
                return
            wait.cancel_once(time.monotonic())
        elif getattr(self,'goal_handle',None) is not None and self.result_future is not None:
            self.goal_handle.cancel_goal_async()

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
