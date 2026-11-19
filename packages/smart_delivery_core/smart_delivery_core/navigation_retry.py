"""Bounded retry rules; a blocked path is not a transient TF failure."""

import math


def retry_data_fresh(sensor_times, monotonic_now, ros_now_ns, transform_stamp_ns):
    """Require fresh receipt/source times and a fresh latest-common TF."""
    stamps = [transform_stamp_ns]
    for name in ('scan', 'odom'):
        receipt, stamp = sensor_times.get(name, (0.0, 0))
        receipt_age = monotonic_now - receipt
        if not math.isfinite(receipt_age) or not 0.0 <= receipt_age <= 0.30:
            return False
        stamps.append(stamp)
    return all(
        stamp > 0 and -0.1 <= (ros_now_ns - stamp) / 1e9 <= 0.30
        for stamp in stamps
    )


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


def transient_navigation_failure(error_code, error_msg):
    """Explicit middleware/lifecycle failures only, never blocked-path failures."""
    text = str(error_msg or '').lower()
    return any(token in text for token in (
        'timed out while waiting for action server', 'failed to get result for',
        'action server is not available', 'server is inactive',
        'server is down after not receiving a heartbeat',
    ))


def transient_navigation_log(message):
    """An expected BT halt while we cancel is not evidence of a DDS outage."""
    text=str(message or '').lower()
    return 'in node halt' not in text and transient_navigation_failure(0,text)


def navigation_failure_is_transient(code, message, *, data_unavailable=False,
                                    failure_kind='', recent_tf=False, recent_transport=False):
    """Explicit physical/policy failures take precedence over nearby cancel logs."""
    physical = failure_kind in {'translation_blocked','clearance_timeout','stage_deadline','route_no_progress',
        'alignment_no_progress','alignment_turn_blocked','alignment_displacement',
        'translation_budget_exhausted'} or any(x in str(message) for x in (
            '通道持續受阻','平移通道未通過','平移偏離','平移車頭方向','準備點偏離'))
    if physical and not data_unavailable:
        return False
    return bool(data_unavailable or transient_tf_failure(code,message)
                or (code==0 and recent_tf) or transient_navigation_failure(code,message)
                or recent_transport)


def result_sync_message(status):
    """Describe the outcome without confusing local receipt with cloud success."""
    label = {'done': '操作已完成', 'failed': '任務失敗',
             'cancelled': '任務已取消', 'released': '任務已釋回佇列'}.get(status, '任務已結束')
    return label + '，結果正在同步雲端，請勿重複操作'
