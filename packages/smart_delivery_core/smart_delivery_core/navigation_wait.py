"""Cooperative action phases. Unknown ownership never becomes a safe completion."""
import time


class NavigationOwnershipUncertain(RuntimeError):
    pass


class NavigationWait:
    def __init__(self, now, timeout=180., ack_timeout=5.):
        self.started, self.timeout, self.ack_timeout = now, timeout, ack_timeout
        self.request = self.handle = self.result = self.cancel = None
        self.cancel_at = None
        self.phase = 'waiting_server'

    def cancel_once(self, now):
        if self.cancel_at is None:self.cancel_at = now
        if self.handle is not None and self.cancel is None:
            self.cancel = self.handle.cancel_goal_async()
        self.phase = 'cancelling'

    def poll(self, now):
        if self.request is not None and self.handle is None and self.request.done():
            self.handle = self.request.result()
            if not self.handle.accepted:
                self.phase = 'rejected';return True
            self.result = self.handle.get_result_async()
            self.phase = 'running'
            if self.cancel_at is not None:self.cancel_once(now)
        if self.result is not None and self.result.done():
            self.phase = 'finished';return True
        if self.handle is None and now-self.started >= self.ack_timeout:
            raise NavigationOwnershipUncertain('Navigation goal acknowledgement timed out; retain task and stop')
        if self.cancel_at is not None and now-self.cancel_at >= self.ack_timeout:
            raise NavigationOwnershipUncertain('Navigation cancellation/result timed out; stop is not confirmed')
        if now-self.started >= self.timeout:self.cancel_once(now)
        return False
