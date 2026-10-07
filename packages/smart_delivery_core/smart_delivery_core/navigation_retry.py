"""Bounded retry rules; a blocked path is not a transient TF failure."""


def transient_tf_failure(error_code, error_msg):
    """Recognize explicit transform failures, not arbitrary cancellation."""
    # Verified against the installed Jazzy action constants.
    text = str(error_msg or '').lower()
    code = int(error_code or 0)
    if code != 0:
        return code in {102, 202}
    return any(token in text for token in (
        'extrapolation', 'transform error', 'tf error', 'tf_error',
        'unable to transform', 'failed to transform', 'could not transform',
        'cannot transform', 'transform timeout',
    ))


def schedule_tf_retry(task, now):
    """Persist the budget in the task, so process restarts cannot reset it."""
    retries = int(task.get('_tf_retry_count', 0))
    if retries >= 2:
        return False
    task['_tf_retry_count'] = retries + 1
    task['_tf_retry_wait_started'] = float(now)
    task['_resume_state'] = 'waiting_localization'
    return True


def result_sync_message(status):
    """Describe the outcome without confusing local receipt with cloud success."""
    label = {'done': '操作已完成', 'failed': '任務失敗',
             'cancelled': '任務已取消', 'released': '任務已釋回佇列'}.get(status, '任務已結束')
    return label + '，結果正在同步雲端，請勿重複操作'
