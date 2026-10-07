"""Keep physical outcomes and transient navigation failures distinct."""

import pytest

from smart_delivery_core.navigation_retry import (
    result_sync_message, retry_data_fresh, schedule_tf_retry, transient_tf_failure,
)


@pytest.mark.parametrize('code', [102, 202])
def test_only_actual_jazzy_tf_codes_are_retryable(code):
    assert transient_tf_failure(code, '')


@pytest.mark.parametrize('code', [103, 105, 106, 203, 208, 107])
def test_invalid_path_no_control_and_obstacles_are_not_tf_failures(code):
    assert not transient_tf_failure(code, '')


def test_explicit_transform_error_text_is_retryable():
    assert transient_tf_failure(0, 'Unable to transform goal pose into costmap frame')
    assert not transient_tf_failure(0, 'Goal failed')
    assert not transient_tf_failure(106, 'No valid control; earlier transform error')


def test_retry_budget_is_retained_in_durable_task_and_bounded_at_two():
    task = {'task_id': 't'}
    assert schedule_tf_retry(task, 10)
    assert task['_tf_retry_count'] == 1
    assert task['_tf_retry_wait_started'] == 10
    restarted_task = dict(task)
    assert schedule_tf_retry(restarted_task, 30)
    assert restarted_task['_tf_retry_count'] == 2
    assert not schedule_tf_retry(dict(restarted_task), 50)


def test_failed_outcome_never_says_operation_completed():
    assert '操作已完成' not in result_sync_message('failed')
    assert '任務失敗' in result_sync_message('failed')
    assert '操作已完成' in result_sync_message('done')


def test_recent_common_transform_need_not_be_at_exact_now():
    sensors = {'scan': (99.95, 9_950_000_000), 'odom': (99.98, 9_980_000_000)}
    assert retry_data_fresh(sensors, 100.0, 10_000_000_000, 9_980_000_000)


@pytest.mark.parametrize('name', ['scan', 'odom'])
@pytest.mark.parametrize('receipt,stamp', [
    (99.6, 9_980_000_000), (99.98, 9_600_000_000),
    (99.98, 10_200_000_000), (99.98, 0), (float('nan'), 9_980_000_000),
])
def test_retry_rejects_stale_future_invalid_or_delayed_sensor(name, receipt, stamp):
    sensors = {'scan': (99.98, 9_980_000_000), 'odom': (99.98, 9_980_000_000)}
    sensors[name] = (receipt, stamp)
    assert not retry_data_fresh(sensors, 100.0, 10_000_000_000, 9_980_000_000)


@pytest.mark.parametrize('stamp', [0, 9_600_000_000, 10_200_000_000])
def test_latest_transform_still_requires_a_fresh_valid_timestamp(stamp):
    sensors = {'scan': (99.98, 9_980_000_000), 'odom': (99.98, 9_980_000_000)}
    assert not retry_data_fresh(sensors, 100.0, 10_000_000_000, stamp)


def test_actual_delivery_retry_uses_latest_common_tf_without_starting_nodes():
    """Execute the actual nested callback against tf2's in-memory Buffer."""
    import ast
    from pathlib import Path
    from types import SimpleNamespace

    from geometry_msgs.msg import TransformStamped
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException

    source = Path(__file__).parents[1] / 'smart_delivery_core' / 'smart_delivery.py'
    main = next(item for item in ast.parse(source.read_text()).body
                if isinstance(item, ast.FunctionDef) and item.name == 'main')
    callback = next(item for item in main.body
                    if isinstance(item, ast.FunctionDef) and item.name == 'retry_data_ready')
    buffer = Buffer()

    def set_transform(parent, child, nanos):
        transform = TransformStamped()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.header.stamp = Time(nanoseconds=nanos).to_msg()
        transform.transform.rotation.w = 1.0
        buffer.set_transform(transform, 'test')

    set_transform('map', 'odom', 9_900_000_000)
    set_transform('map', 'odom', 10_100_000_000)
    set_transform('odom', 'base_footprint', 9_900_000_000)
    set_transform('odom', 'base_footprint', 9_980_000_000)
    ros_now = Time(nanoseconds=10_000_000_000)
    assert not buffer.can_transform('map', 'base_footprint', ros_now)
    namespace = dict(
        time=SimpleNamespace(monotonic=lambda: 100.0),
        navigator=SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: ros_now)),
        tf_buffer=buffer, Time=Time, TransformException=TransformException,
        retry_sensor_times={'scan': (99.95, 9_950_000_000),
                            'odom': (99.98, 9_980_000_000)},
        retry_data_fresh=retry_data_fresh,
    )
    exec(compile(ast.Module(body=[callback], type_ignores=[]), str(source), 'exec'), namespace)
    assert namespace['retry_data_ready']()
    # A connected but stale chain, missing chain, and delayed data stay blocked.
    namespace['time'].monotonic = lambda: 101.0
    assert not namespace['retry_data_ready']()
    namespace['time'].monotonic = lambda: 100.0
    namespace['navigator'].get_clock = lambda: SimpleNamespace(
        now=lambda: Time(nanoseconds=11_000_000_000))
    assert not namespace['retry_data_ready']()
    buffer.clear()
    assert not namespace['retry_data_ready']()
