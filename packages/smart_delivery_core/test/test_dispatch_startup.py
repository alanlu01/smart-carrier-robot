"""Startup parking gate must not become a lifetime cloud-ACK gate."""

import pytest

from smart_delivery_core.dispatch_startup import StartupDispatchGate


@pytest.mark.parametrize('state', [None, 'waiting', 'tasks'])
def test_startup_waits_for_actual_work_or_confirmed_empty(state):
    gate = StartupDispatchGate()
    gate.observe(has_work=False, sync_state=state)
    assert gate.waiting


def test_received_initial_batch_bypasses_parking_and_latches_gate():
    gate = StartupDispatchGate()
    gate.observe(has_work=True, sync_state='tasks')
    assert not gate.waiting
    # Completion, cancellation, pending cloud result or network failure must not
    # reopen startup reconciliation after the local batch has been handled.
    for state in ('waiting', 'tasks', None, 'empty'):
        gate.observe(has_work=False, sync_state=state)
        assert not gate.waiting


def test_confirmed_empty_allows_parking_after_later_offline_or_cloud_ack_wait():
    gate = StartupDispatchGate()
    gate.observe(has_work=False, sync_state='empty')
    assert not gate.waiting
    gate.observe(has_work=False, sync_state='waiting')
    assert not gate.waiting


def test_new_delivery_process_must_reconcile_again():
    previous = StartupDispatchGate()
    previous.observe(has_work=False, sync_state='empty')
    assert not previous.waiting
    assert StartupDispatchGate().waiting


def test_gate_is_observed_before_initial_work_can_be_cancelled_or_acked():
    """Check the integration point in the real delivery loop."""
    import ast
    from pathlib import Path

    source = Path(__file__).parents[1] / 'smart_delivery_core' / 'smart_delivery.py'
    main = next(item for item in ast.parse(source.read_text()).body
                if isinstance(item, ast.FunctionDef) and item.name == 'main')
    loop = next(item for item in main.body if isinstance(item, ast.While)
                and ast.unparse(item.test) == 'rclpy.ok()')
    assert ast.unparse(loop.body[1]).startswith('startup_dispatch.observe(')
    observation = ast.unparse(loop.body[1])
    assert 'bool(pending_orders) and pending_result is None' in observation
    idle = next(item for item in loop.body if isinstance(item, ast.If)
                and ast.unparse(item.test) == 'not pending_orders')
    assert any(isinstance(item, ast.If)
               and ast.unparse(item.test) == 'startup_dispatch.waiting' for item in idle.body)


def test_old_pending_result_alone_does_not_skip_fresh_startup_reconciliation():
    """A restart with only an old result must still discover queued orders."""
    gate = StartupDispatchGate()
    pending_orders = [{'task_id': 'old-completed-task'}]
    pending_result = {'task_id': 'old-completed-task', 'status': 'done'}
    gate.observe(has_work=bool(pending_orders) and pending_result is None,
                 sync_state='waiting')
    assert gate.waiting


@pytest.mark.parametrize('confirmed,parked,expected_goals,expected_requests', [
    (False, False, 0, 1), (True, False, 1, 0), (True, True, 0, 0),
])
def test_actual_idle_branch_waits_only_at_startup(
    confirmed, parked, expected_goals, expected_requests,
):
    """Run the production idle branch with offline/waiting API and fake motion."""
    import ast
    import math
    from pathlib import Path
    from types import SimpleNamespace

    from smart_delivery_core.standby_retry import StandbyRetryPolicy

    source = Path(__file__).parents[1] / 'smart_delivery_core' / 'smart_delivery.py'
    main = next(item for item in ast.parse(source.read_text()).body
                if isinstance(item, ast.FunctionDef) and item.name == 'main')
    loop = next(item for item in main.body if isinstance(item, ast.While)
                and ast.unparse(item.test) == 'rclpy.ok()')
    idle = next(item for item in loop.body if isinstance(item, ast.If)
                and ast.unparse(item.test) == 'not pending_orders')
    wrapper = ast.For(
        target=ast.Name(id='_iteration', ctx=ast.Store()),
        iter=ast.List(elts=[ast.Constant(value=0)], ctx=ast.Load()),
        body=[idle], orelse=[],
    )
    goals, requests, warnings = [], [], []

    def park(*args):
        goals.append(args[-1])
        return {'x': 2.4, 'y': 5.0}

    namespace = dict(
        pending_orders=[], startup_dispatch=StartupDispatchGate(confirmed),
        dispatch_state={'state': 'waiting', 'online': False},
        time=SimpleNamespace(monotonic=lambda: 100.0), math=math, is_standby=parked,
        dispatch_last_request_at=0.0, dispatch_started_at=0.0,
        dispatch_last_warning_at=0.0, dispatch_request_id='fresh-start',
        dispatch_sync_publisher=SimpleNamespace(publish=requests.append),
        navigator=SimpleNamespace(get_logger=lambda: SimpleNamespace(warning=warnings.append)),
        String=SimpleNamespace, json=__import__('json'),
        current_pos={'x': 0.0, 'y': 0.0}, refresh_current_position=lambda pos: pos,
        STANDBY_POINTS=[{'name': 'CYCU EE', 'x': 2.4, 'y': 5.0, 'yaw': 0.0}],
        standby_retry=StandbyRetryPolicy(), localization_ready=True,
        go_to_standby=park, set_navigation_active=lambda _: None,
        report_standby=lambda *args: None,
    )
    module = ast.fix_missing_locations(ast.Module(body=[wrapper], type_ignores=[]))
    exec(compile(module, str(source), 'exec'), namespace)
    assert len(goals) == expected_goals
    assert len(requests) == expected_requests
    if confirmed:
        assert not warnings
        assert namespace['is_standby']
