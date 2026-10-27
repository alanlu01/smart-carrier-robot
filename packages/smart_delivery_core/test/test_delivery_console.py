"""Console presentation cannot erase the machine-readable diagnosis."""

import json
from types import SimpleNamespace
import pytest

from smart_delivery_core.delivery_console import people_status,safety_status,startup_status,reason_label
from test_optional_idle_runtime import runtime


@pytest.mark.parametrize('state',['going_observation','waiting_observation_stop','observing',
    'waiting_observation_retry','waiting_people_localization','people_destination_selected',
    'going_people_standby','waiting_people_parking_recovery','people_arrival_check',
    'people_parking_retry_wait','people_parking_blocked','idle'])
def test_people_lines_are_compact_single_lines_not_json(state):
    line=people_status(state,'people_data_pause',{'name':'座位'},180.,1)
    assert len(line)<80 and '\n' not in line and '{' not in line
    assert not line.startswith(('state','人流｜'))


def test_idle_qualification_is_not_presented_as_a_failed_spin():
    assert safety_status(['odom_fresh'],'idle',False)=='⏳ 待機計時暫緩：里程計更新中'
    assert '安全暫停' in safety_status(['map_pose_valid'],'observing',True)
    assert people_status('idle','parked_at_selected_standby',{'name':'座位'},360.)=='💤 已停妥：座位'
    assert reason_label('scan_filtered_unknown_coverage')=='雷達視野不足'


def test_startup_presentation_keeps_inactive_distinct_from_duplicates():
    assert '尚未啟用' in startup_status('waiting_activation')
    assert '重複' in startup_status('retrying',duplicates=True)
    assert startup_status('ready').startswith('✅')


def test_structured_failure_survives_short_console_and_rate_limit(runtime):
    m,n,t=runtime
    lines=[];n.get_logger=lambda:SimpleNamespace(info=lines.append,warning=lines.append)
    m._pose=lambda *_:None
    assert not m._safe()
    m._heartbeat()
    assert lines[0].startswith('⏳ 待機計時暫緩') and '{' not in lines[0]
    assert m.last_unsafe_report['failed_checks']==['map_pose_valid']
    assert m.last_unsafe_report['odom_source_age_sec']==0.
    assert not m._safe() and len(lines)==2  # one safe failure + one idle transition


def test_cancel_pending_console_does_not_claim_spin_is_still_analysing(runtime):
    m,n,_=runtime
    m.state,m.motion,m.cancel_requested='observing',True,True
    m.reason='people_data_pause';lines=[]
    n.get_logger=lambda:SimpleNamespace(info=lines.append)
    m._log_people_state()
    assert lines[0].startswith('🛑') and '分析中' not in lines[0]
