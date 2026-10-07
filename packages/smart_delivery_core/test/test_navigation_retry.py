"""Keep physical outcomes and transient navigation failures distinct."""

import pytest

from smart_delivery_core.navigation_retry import (
    result_sync_message, schedule_tf_retry, transient_tf_failure,
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
