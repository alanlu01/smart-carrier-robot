"""Unwind on user stop while keeping the ROS context alive for cancellation."""
import signal


class DeliveryStop(BaseException):
    """A requested normal stop, not an execution failure."""


class DeliverySignals:
    def __enter__(self):
        self.requested = False
        self.previous = {sig:signal.getsignal(sig) for sig in (signal.SIGINT,signal.SIGTERM)}
        for sig in self.previous:
            signal.signal(sig,self._request)
        return self

    def _request(self, _signum, _frame):
        # No ROS or I/O in the handler. Repeated Ctrl+C cannot interrupt cleanup.
        if not self.requested:
            self.requested = True
            raise DeliveryStop()

    def __exit__(self, *_):
        for sig,handler in self.previous.items():
            signal.signal(sig,handler)
