"""Human-readable console only. Machine state/reason codes remain unchanged."""


def brief(value, limit=64):
    """Keep one compact line even for externally configured names/errors."""
    text = ' '.join(str(value or '').split())
    return text if len(text) <= limit else text[:limit-1]+'…'


def localization_label(state):
    return {
        'UNINITIALIZED':'尚未初始化','WAITING_FOR_SENSORS':'等待感測資料',
        'SEEDING':'設定初始位置','VERIFYING':'驗證定位','LOCALIZED':'定位就緒',
        'SUSPECT':'確認定位品質','RECOVERING_LOCAL':'原地更新定位',
        'RECOVERING_GLOBAL':'全域重新定位','MANUAL_REQUIRED':'需要人工定位',
        'STABILIZING_BEFORE_RESUME':'等待定位穩定',
    }.get(state, brief(state))


def reason_label(reason):
    """Translate common control codes without hiding the original in ROS state."""
    reason = str(reason or '')
    exact = {
        'near_obstacle':'附近有障礙','recent_near_obstacle':'等待附近障礙離開',
        'unknown_scan_coverage':'雷達視野不足','unknown_coverage':'雷達視野不足',
        'laser_tf_pending':'等待雷達座標更新','laser_tf_wait_timeout':'雷達座標更新逾時',
        'laser_tf_unavailable':'雷達座標不可用','scan_stale_or_absent':'雷達資料過期',
        'odom_tf_stale':'里程計座標過期','people_data_pause':'等待新鮮感測資料',
        'localization_suspect':'等待定位恢復','observation_wait_unsafe':'安全條件尚未恢復',
        'people_localization_wait_timeout':'等待定位／感測恢復逾時',
        'selected_standby_safety_pause':'等待定位／感測恢復',
        'selected_standby_recovery_timeout':'等待停靠恢復逾時',
        'insufficient_rotation_coverage':'旋轉覆蓋不足','nav2_failed':'導航未完成',
        'goal_rejected':'導航目標被拒絕','waiting_fresh_safety_data':'等待新鮮定位資料',
        'vehicle_not_stopped':'等待停妥','continuous_safe_duration_insufficient':'等待連續安全視野',
        'insufficient_independent_scans':'等待更多雷達資料',
        'stopped_duration_insufficient':'等待停妥','observation_position_not_verified':'分析點位置尚未確認',
        'observation_or_navigation_timeout':'導航／分析逾時','spin_resume_budget_exhausted':'續轉額度已用盡',
        'localization_recovered':'定位／感測已恢復',
        'localization_recovered_recollecting':'定位修正，重新收集人流',
    }
    if reason in exact:
        return exact[reason]
    if reason.endswith('unknown_coverage'):
        return '雷達視野不足'
    if reason == 'motion_disabled_moving_or_sensor_gap':
        return '感測／停穩條件不足'
    if reason.startswith('transform_or_geometry_error'):
        return '雷達座標／幾何資料異常'
    if reason.startswith('observation_preempted'):
        return '新任務／電池優先接管'
    if reason.startswith('spin_ready_timeout:'):
        return '等待安全旋轉逾時'
    if reason.startswith('people_localization_escalated:'):
        return '定位升級，結束本輪分析'
    if reason.startswith('selected_standby_'):
        return '選定待機點尚未停妥'
    if reason.startswith('people_localization_'):
        return '定位證據不足，結束本輪分析'
    return brief(reason or '等待安全條件')


def people_status(state, reason, point, degrees, attempts=0, blocker=''):
    name = brief((point or {}).get('name','—'))
    labels = {
        'going_observation':'➡️ 前往人流分析點',
        'waiting_observation_stop':('⏳ 等待停妥與安全視野' if blocker in ('','not_waiting','ready')
                                    else '⏳ 等待旋轉：'+reason_label(blocker)),
        'observing':f'🔄 人流分析中（{max(0., degrees):.0f}°／360°）',
        'waiting_observation_retry':'⏸️ 人流自旋暫停：'+reason_label(blocker or reason),
        'waiting_people_localization':'⏸️ 人流暫停：'+reason_label(reason),
        'people_destination_selected':f'✅ 分析完成，選擇：{name}',
        'going_people_standby':f'➡️ 前往待機點：{name}'+(f'（第 {attempts} 次）' if attempts>1 else ''),
        'waiting_people_parking_recovery':f'⏸️ 等待定位／感測恢復，保留目標：{name}',
        'people_arrival_check':f'⏳ 確認停妥：{name}',
        'people_parking_retry_wait':f'⏳ 準備補停：{name}',
        'people_parking_blocked':f'⚠️ 停靠未完成：{name}，等待新任務',
    }
    if state in labels:
        return labels[state]
    if reason == 'parked_at_selected_standby':
        return f'💤 已停妥：{name}'
    if reason == 'insufficient_directional_demand':
        return '🔎 人流證據不足，維持一般待機'
    if reason.startswith('observation_preempted'):
        return '📥 新任務／電池接管，人流流程已結束'
    return '💤 人流待機／冷卻' if not reason else '⚠️ 人流流程結束：'+reason_label(reason)


def safety_status(failed, state, motion):
    labels = {'map_matches':'地圖不符','delivery_ready':'等待定位',
              'motion_allowed':'暫時禁止移動','battery_healthy':'電池資料異常',
              'odom_fresh':'里程計更新中','map_pose_valid':'定位座標更新中'}
    detail = '、'.join(labels.get(key,key) for key in failed[:2])
    if not motion and state in ('idle','people_parking_blocked'):
        return '⏳ 待機計時暫緩：'+detail
    if not motion:
        return '⏳ 人流流程暫緩：'+detail
    return '⏸️ 人流動作安全暫停：'+detail


def startup_status(state, reason='', duplicates=False):
    if state == 'ready':
        return '✅ 導航準備就緒'
    if duplicates:
        return '⚠️ 發現重複導航節點，請先結束舊程序'
    if state == 'waiting_activation':
        return '⏳ 導航尚未啟用，請檢查 start_nav'
    if state == 'failed':
        return '❌ 導航啟動逾時，訂單已保留'
    if 'response' in reason:
        return '⏳ 導航回覆逾時，正在重試'
    return '⏳ 等待導航上線'
