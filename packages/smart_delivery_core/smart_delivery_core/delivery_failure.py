"""Bounded exception cleanup; a cleanup result never authorizes more motion."""

import time


def stop_after_failure(runtime, *, timeout=5., spin_once=None):
    """Stop lease renewal, cancel accepted/pending goals, verify a fresh stop."""
    uses_ros = spin_once is None
    if uses_ros:
        import rclpy
        spin_once = lambda: rclpy.spin_once(runtime.navigator, timeout_sec=.05)
    runtime.observation_collecting = False
    # Keep the independent guard armed: publishing False would release it.
    runtime.suspend_lease()
    runtime.cancel_requested = True
    if uses_ros and not rclpy.ok():
        return dict(stopped=False, action_drained=False, reason='ros_context_unavailable', errors=[])
    navigator = runtime.navigator
    deadline = time.monotonic() + timeout
    last_cancel, stopped_since = -float('inf'), None
    request = runtime.goal_request
    handle = None if request is not None else getattr(navigator, 'goal_handle', None)
    result = None if request is not None else getattr(navigator, 'result_future', None)
    cancel = None
    errors = []
    while time.monotonic() < deadline:
        if uses_ros and not rclpy.ok():
            return dict(stopped=False, action_drained=False, reason='ros_context_unavailable', errors=errors)
        now = time.monotonic()
        try:
            if request is not None and request.done():
                handle = request.result()
                if handle.accepted:
                    result = handle.get_result_async()
                else:
                    handle = None
                request = None
            if (handle is not None and (result is None or not result.done())
                    and now-last_cancel >= 1. and (cancel is None or cancel.done())):
                cancel = handle.cancel_goal_async()
                last_cancel = now
            drained = request is None and (handle is None or (result is not None and result.done()))
            if drained and runtime._stationary():
                stopped_since = now if stopped_since is None else stopped_since
                if now-stopped_since >= 1.:
                    return dict(stopped=True, action_drained=True, errors=errors)
            else:
                stopped_since = None
            spin_once()
        except Exception as exc:
            # A failing callback must not defeat bounded cancellation retries.
            if len(errors) < 4:
                errors.append(f'{type(exc).__name__}: {exc}')
            stopped_since = None
            time.sleep(.02)
    return dict(stopped=False, action_drained=False, errors=errors)
