"""Auditable aggregation of extracted recordings; no robot access."""
import collections
import sys
import csv
import datetime as dt
import json
import re
from pathlib import Path
import numpy as np

HERE = Path(__file__).parent / 'inputs'
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[1] / 'reproduced_quantitative'
OUT.mkdir(parents=True, exist_ok=True)
TZ = dt.timezone(dt.timedelta(hours=8))


def load(name):
    return [json.loads(s) for s in (HERE/name).read_text(encoding='utf-8-sig').splitlines() if s.strip()]


def iso(t):
    return dt.datetime.fromtimestamp(t, TZ).isoformat(timespec='milliseconds') if t is not None else None


def event_time(x, field):
    val = x['data'].get(field)
    try:
        return dt.datetime.fromisoformat(val.replace('Z', '+00:00')).timestamp()
    except (TypeError, ValueError, AttributeError):
        return x['t']


def stats(vals):
    vals = [x for x in vals if x is not None and np.isfinite(x)]
    if not vals:
        return {'n': 0, 'median': None, 'p95': None, 'max': None, 'mean': None, 'min': None}
    return {'n': len(vals), 'median': float(np.median(vals)), 'p95': float(np.percentile(vals, 95)),
            'max': max(vals), 'min': min(vals), 'mean': float(np.mean(vals))}


runs = load('extracted.jsonl')
extras = {x['run']: x for x in load('extras.jsonl')}
texts = {x['run']: x for x in load('text_events.jsonl')}
power_check = {x['run']: x for x in load('power_coverage.jsonl')}
all_tasks = collections.defaultdict(lambda: collections.defaultdict(list))
sessions, topics, episodes, batches, parks, power = [], [], [], [], [], []
historical = []
for idx, r in enumerate(runs, 1):
    sid = f'S{idx:02}'
    r['source_id'] = sid
    correction = {(x['topic'], x['t']): x['data'] for x in texts[r['run']]['rows']}
    # September 22/23 used a single-order /order schema rather than {orders:[...]}.
    legacy_seen = set()
    for x in texts[r['run']]['rows']:
        s = x['data']
        if x['topic']=='/order' and s.get('id') and s.get('task_type') and 'orders' not in s:
            tid = str(s['id'])
            if tid in legacy_seen:
                continue
            legacy_seen.add(tid)
            task = r['tasks'].setdefault(tid, {'states':[],'results':[],'acks':[],'claims':[],'cancels':[]})
            task['claims'].append({'t':x['t'],'batch_id':tid,'data':s})
    for e in r['localization_events']:
        e['data'] = correction.get(('/localization/state', e['t']), e['data'])
    # Rebuild ROS evidence from escaped-Unicode transport, not a lossy console decode.
    r['rosout_evidence'] = [dict(t=x['t'], **x['data']) for x in texts[r['run']]['rows'] if x['topic']=='/rosout']
    for tid, task in r['tasks'].items():
        for key, topic in [('states','task_state'),('results','task_result'),('acks','task_result_ack'),('cancels','task_cancel')]:
            for row in task[key]:
                row['data'] = correction.get(('/smart_carrier/'+topic, row['t']), row['data'])
                row['source_id'] = sid
                row['run'] = r['run']
                all_tasks[tid][key].append(row)
        for row in task['claims']:
            b = correction.get(('/order', row['t']))
            if b:
                row['data'] = next((o for o in b.get('orders', []) if str(o.get('id'))==tid), row['data'])
            row['source_id'] = sid
            row['run'] = r['run']
            all_tasks[tid]['claims'].append(row)
    # State episodes are continuous non-LOCALIZED intervals, not warning-message counts.
    current = None
    prev = None
    eps = []
    for e in r['localization_events']:
        state = e['data'].get('state')
        if state == 'LOCALIZED':
            if current:
                current['end'] = e['t']
                current['duration_s'] = e['t']-current['start']
                current['outcome'] = '曾要求人工後回復' if current['manual_required'] else '程式自動回復'
                eps.append(current)
                current = None
        else:
            if current is None:
                current = {'source_id': sid, 'run': r['run'], 'start': e['t'], 'end': None, 'duration_s': None,
                           'first_state': state, 'reason': e['data'].get('reason'), 'startup_or_left_censored': prev is None,
                           'local_recovery': False, 'global_recovery': False, 'manual_required': False,
                           'outcome': '錄製結束前未觀察到回復', 'map_score_at_start': e['data'].get('map_match_score'),
                           'xy_std_m': e['data'].get('xy_std'), 'yaw_std_rad': e['data'].get('yaw_std'),
                           'physical_ground_truth': '未量測'}
            current['local_recovery'] |= state == 'RECOVERING_LOCAL'
            current['global_recovery'] |= state == 'RECOVERING_GLOBAL'
            current['manual_required'] |= state == 'MANUAL_REQUIRED'
        prev = state
    if current:
        eps.append(current)
    for k, e in enumerate(eps, 1):
        e['episode_id'] = f'{sid}-L{k:02}'
        e['start_local'] = iso(e['start'])
        e['end_local'] = iso(e['end'])
        e['operation_note'] = '人工操作情境未完整標註'
        hm = dt.datetime.fromtimestamp(e['start'], TZ).strftime('%H:%M')
        if r['run'].startswith('20260929_152029') and hm in ('15:47', '15:48'):
            e['operation_note'] = '使用者確認此時段曾搬動或轉動車輛'
        elif r['run'].startswith('20260929_203819') and hm in ('20:57','20:58'):
            e['operation_note'] = '使用者確認往右搬約1m、車頭微轉；曾出現錯誤定位解'
            e['physical_ground_truth'] = '使用者觀察錯位，未作測量'
        elif r['run'].startswith('20261001') and hm == '14:03':
            e['operation_note'] = '使用者確認網路訊號差，未動硬體；同步資料管線停頓'
        elif r['run'].startswith('20261001') and hm in ('14:13', '14:14'):
            e['operation_note'] = '使用者確認搬車'
        elif r['run'].startswith('20261003') and hm in ('12:39', '12:42'):
            e['operation_note'] = '是否刻意搬車尚未確認，不當成自主重定位成功率樣本'
    episodes.extend(eps)
    for n, t in r['timing'].items():
        if t['count']<2:
            continue
        topics.append({'source_id':sid,'run':r['run'],'topic':n,'samples':t['count'],'hz':t['hz'],
                       'max_gap_s':t['gap_s']['max'],'p95_gap_s':t['gap_s']['p95'],'gaps_gt_1s':t['gaps_gt_1s'],
                       'age_median_s':t['age_s']['median'] if t['age_s'] else None,
                       'age_p95_s':t['age_s']['p95'] if t['age_s'] else None,
                       'large_gaps':t['large_gaps']})
    unique_results = {}
    for tid, task in r['tasks'].items():
        for ev in task['results']:
            t = event_time(ev, 'created_at')
            key = (tid, ev['data'].get('event_id'))
            if r['start'] <= t <= r['end']:
                unique_results[key] = ev
            else:
                historical.append({'source_id':sid,'task_id':tid,'event_id':key[1],'event_time_local':iso(t),'received_local':iso(ev['t']),'status':ev['data'].get('status'),'reason':'事件時間在此錄製視窗之外'})
    status = collections.Counter(ev['data'].get('status') for ev in unique_results.values())
    warns = extras[r['run']]['recording_warnings']
    loss = sorted(set(int(m.group(1)) for w in warns if (m:=re.search(r'lost on the transport layer:\s*(\d+)',w['line']))))
    # Recorder reports cumulative lost count; take the final maximum, never sum repeated reports.
    sess = {'source_id':sid,'run':r['run'],'start':r['start'],'end':r['end'],'start_local':iso(r['start']),'end_local':iso(r['end']),
            'duration_s':r['duration_s'],'messages':r['message_count'],'bytes':r['bytes'],'storage':r['storage'],
            'commit':r['commit'],'exit_code':r['exit_code'],'lost_reported':max(loss) if loss else None,
            'initialpose_qos_warning':any('incompatible QoS' in w['line'] for w in warns),
            'done_events':status['done'],'cancelled_events':status['cancelled'],'failed_events':status['failed'],
            'recorded_task_ids':len(r['tasks']),'historical_result_messages':sum(x['source_id']==sid for x in historical),
            'localization_episodes':len(eps),'postlocalized_episodes':sum(not e['startup_or_left_censored'] for e in eps),
            'local_recovery_episodes':sum(e['local_recovery'] for e in eps),'global_recovery_episodes':sum(e['global_recovery'] for e in eps),
            'manual_required_episodes':sum(e['manual_required'] for e in eps),
            'vision_fps_median':r['vision_fps']['median'] if r['vision_fps'] else None,
            'vision_message_hz':r['timing'].get('/vision/semantic_info',{}).get('hz'),
            'cpu_mean_percent':r['cpu_busy_percent']['mean'] if r['cpu_busy_percent'] else None,
            'cpu_p95_percent':r['cpu_busy_percent']['p95'] if r['cpu_busy_percent'] else None,
            'cpu_samples':len(extras[r['run']]['cpu_samples']), 'metadata_sha256':extras[r['run']]['metadata_sha256'], 'source_uri':r['uri']}
    sessions.append(sess)
    for bid,b in r['batches'].items():
        data = correction.get(('/order', b['t']), b['data'])
        packed = data.get('orders', [])
        if not packed and data.get('id') and data.get('task_type'):
            packed = [data]
        ids = [str(o.get('id')) for o in packed]
        nav = []
        for tid in ids:
            vals = [event_time(x,'progress_updated_at') for x in r['tasks'].get(tid,{}).get('states',[]) if x['data'].get('progress_state')=='navigating']
            if vals:
                nav.append((min(vals),tid))
        # Distinguish backlog/latched batches from newly observed schedules.
        batches.append({'source_id':sid,'run':r['run'],'batch_id':bid,'first_received':iso(b['t']),'order_count':len(ids),
                        'claimed_ids':ids,'execution_ids':[tid for _,tid in sorted(nav)],
                        'execution_times':[iso(t) for t,_ in sorted(nav)],'pose_at_first_receive':b['pose'],
                        'note':'接收時位姿非必然為排程起點，不據此計算最優性或0.7增益'})
    for row in r['parked']:
        row['data'] = correction.get(('/delivery/standby_state', row['t']), row['data'])
        parks.append({'source_id':sid,'run':r['run'],'time_local':iso(row['t']),'point':row['data'].get('name'),
                      'estimated_distance_m':row['estimated_distance_m'],'estimated_yaw_error_deg':row['estimated_yaw_error_deg'],
                      'amcl_age_s':row['t']-row['pose'][3],'note':'AMCL估計值，不是實體停靠誤差'})
    for ch in ('ch1','ch2','ch3'):
        p = r['power'][ch]
        note = '僅量化sensor_ok旗標；空槽、滿電、SOC需實體真值核對'
        if r['run'].startswith('20260930'):
            note += '；尾端斷電為使用者操作'
        power.append({'source_id':sid,'run':r['run'],'channel':ch,'samples':p['samples'],'invalid_samples':p['invalid_count'],
                      'known_sensor_flags':power_check[r['run']]['coverage'][ch]['known_sensor_flag'],
                      'saturated_samples':p['saturated_count'],'current_max_a':p['current_a']['max'] if p['current_a'] else None,'note':note})

# Deduplicate task events across recordings. The denominator includes only a current
# state or terminal result whose embedded timestamp lies inside a recorded interval.
orders = []
for tid, data in all_tasks.items():
    states = {}
    for x in data['states']:
        states[x['data'].get('event_id', json.dumps(x['data'],sort_keys=True))] = x
    states = sorted(states.values(), key=lambda x:event_time(x,'progress_updated_at'))
    results = {}
    for x in data['results']:
        results.setdefault(x['data'].get('event_id'),x)
    results = sorted(results.values(), key=lambda x:event_time(x,'created_at'))
    def covered(t):
        return any(r['start']<=t<=r['end'] for r in runs)
    valid_states = [x for x in states if covered(event_time(x,'progress_updated_at'))]
    valid_results = [x for x in results if covered(event_time(x,'created_at'))]
    if not valid_states and not valid_results:
        continue
    claim = min(data['claims'],key=lambda x:x['t']) if data['claims'] else None
    info = claim['data'] if claim else {}
    terminal = valid_results[-1] if valid_results else None
    done_t = event_time(terminal,'created_at') if terminal else None
    sources = sorted(set(x['source_id'] for arr in data.values() for x in arr))
    def first(st):
        return min((event_time(x,'progress_updated_at') for x in valid_states if x['data'].get('progress_state')==st),default=None)
    nav = first('navigating')
    arrived = first('arrived')
    # Use the final contiguous verification attempt before completion, not an earlier aborted attempt.
    last_ver = None
    for x in valid_states:
        t = event_time(x,'progress_updated_at')
        if done_t is None or t>done_t:
            continue
        st = x['data'].get('progress_state')
        if st == 'verifying_action':
            if last_ver is None:
                last_ver = t
        elif st in ('wrong_slot','failed','queued_on_robot','navigating','paused_localization'):
            last_ver = None
        elif st == 'waiting_action':
            # The final success notification uses waiting_action with no changed slots.
            if x['data'].get('changed_slots'):
                last_ver = None
    if terminal and terminal['data'].get('status')!='done':
        last_ver = None
    cloud = []
    for ex in extras.values():
        cloud.extend(x for x in ex['cloud_ack_logs'] if x['task_id']==tid and terminal and x['status']==terminal['data'].get('status') and x['t']>=done_t)
    cloud_t = min((x['t'] for x in cloud),default=None)
    event_id = terminal['data'].get('event_id') if terminal else None
    ack_t = min((x['t'] for x in data['acks'] if x['data'].get('event_id')==event_id and done_t is not None and x['t']>=done_t),default=None)
    row = {'task_id':tid,'source_ids':','.join(sources),'type':info.get('task_type'), 'location_code':info.get('location_code'),
           'created_local':iso(event_time(claim,'created_at')) if claim and info.get('created_at') else None,
           'first_navigating_local':iso(nav),'arrived_local':iso(arrived),
           'result_local':iso(done_t),'result_status':terminal['data'].get('status') if terminal else '未觀察到終態',
           'result_source_id':terminal['source_id'] if terminal else None,
           'confirm_start_local':iso(last_ver),'confirm_to_result_s':done_t-last_ver if done_t is not None and last_ver is not None else None,
           'cloud_ack_local':iso(cloud_t),'cloud_ack_delay_s':cloud_t-done_t if cloud_t is not None else None,
           'first_ack_local':iso(ack_t),'first_ack_delay_s':ack_t-done_t if ack_t is not None else None,
           'navigation_elapsed_s':arrived-nav if nav is not None and arrived is not None and arrived>=nav else None,
           'wrong_slot_events':0,'paused_localization_events':0,'historical_result_events':len(results)-len(valid_results),
           'note':'含人工協助與程序重啟；非自主配送成功率'}
    last_st = None
    for x in valid_states:
        st = x['data'].get('progress_state')
        if st!=last_st:
            if st=='wrong_slot': row['wrong_slot_events'] += 1
            if st=='paused_localization': row['paused_localization_events'] += 1
        last_st=st
    if len(set(x['data'].get('status') for x in valid_results))>1:
        row['note'] += '；多種終態事件需人工核對'
    if row['confirm_to_result_s'] is not None and row['confirm_to_result_s']<0:
        raise ValueError(('negative confirmation',tid))
    orders.append(row)
orders.sort(key=lambda x:x['result_local'] or x['first_navigating_local'] or '')
for s in sessions:
    rows = [o for o in orders if o['result_source_id']==s['source_id']]
    s['unique_done'] = sum(o['result_status']=='done' for o in rows)
    s['unique_cancelled'] = sum(o['result_status']=='cancelled' for o in rows)
    s['unique_failed'] = sum(o['result_status']=='failed' for o in rows)
    s['done_cloud_confirmed'] = sum(o['result_status']=='done' and o['cloud_ack_local'] is not None for o in rows)
    s['confirm_stats'] = stats([o['confirm_to_result_s'] for o in rows if o['result_status']=='done'])
    s['cloud_stats'] = stats([o['cloud_ack_delay_s'] for o in rows if o['result_status']=='done'])

status = collections.Counter(o['result_status'] for o in orders)
recent = [o for o in orders if o['result_source_id'] in ('S07','S08','S09')]
summary = {'sessions':len(sessions),'duration_h':sum(s['duration_s'] for s in sessions)/3600,'messages':sum(s['messages'] for s in sessions),
           'size_gib':sum(s['bytes'] for s in sessions)/1024**3,'observed_orders':len(orders),'statuses':dict(status),
           'done_cloud_confirmed':sum(o['result_status']=='done' and o['cloud_ack_local'] is not None for o in orders),
           'confirm_stats':stats([o['confirm_to_result_s'] for o in orders if o['result_status']=='done']),
           'cloud_stats':stats([o['cloud_ack_delay_s'] for o in orders if o['result_status']=='done']),
           'recent_orders':len(recent),'recent_statuses':dict(collections.Counter(o['result_status'] for o in recent)),
           'recent_confirm_stats':stats([o['confirm_to_result_s'] for o in recent if o['result_status']=='done']),
           'recent_cloud_stats':stats([o['cloud_ack_delay_s'] for o in recent if o['result_status']=='done']),
           'episodes':len(episodes),'manual_required_episodes':sum(e['manual_required'] for e in episodes),
           'postlocalized_episodes':sum(not e['startup_or_left_censored'] for e in episodes),
           'historical_result_messages':len(historical),'recorder_lost_reported':sum(s['lost_reported'] or 0 for s in sessions),
           'multi_order_batches':sum(b['order_count']>1 for b in {b['batch_id']:b for b in batches}.values()),
           'batches':len({b['batch_id'] for b in batches}),
           'initialpose_qos_warning_sessions':sum(s['initialpose_qos_warning'] for s in sessions)}
summary['sensor_flag_samples'] = sum(p['known_sensor_flags'] for p in power)
summary['sensor_flag_invalid_samples'] = sum(p['invalid_samples'] for p in power)
summary['near_current_limit_samples'] = sum(p['saturated_samples'] for p in power)
summary['closed_localization_episodes'] = sum(e['end'] is not None for e in episodes)
summary['localization_recovery_stats'] = stats([e['duration_s'] for e in episodes if not e['startup_or_left_censored']])
summary['short_pause_stats'] = stats([e['duration_s'] for e in episodes if not e['startup_or_left_censored'] and not e['local_recovery'] and not e['global_recovery']])
payload = {'summary':summary,'sessions':sessions,'orders':orders,'localization_episodes':episodes,'topic_metrics':topics,'power':power,'batches':batches,'parked':parks,'excluded_historical_results':historical,
           'semantic': [{'source_id':r['source_id'],'run':r['run'],'duration_s':r['semantic_duration_s'],'commands':r['semantic_command_counts'],
                         'vision_fps':r['vision_fps'],'detection_counts':r['detections'],'half_error':r['semantic_half_error']} for r in runs]}
(OUT/'analysis_data.json').write_text(json.dumps(payload,ensure_ascii=False,indent=2),encoding='utf-8')
for key in ('sessions','orders','localization_episodes','topic_metrics','power','batches','parked','excluded_historical_results'):
    rows = payload[key]
    if not rows:
        continue
    with (OUT/(key+'.csv')).open('w',encoding='utf-8-sig',newline='') as f:
        wr=csv.DictWriter(f,fieldnames=list(rows[0]))
        wr.writeheader()
        for row in rows:
            wr.writerow({k:json.dumps(v,ensure_ascii=False) if isinstance(v,(dict,list)) else v for k,v in row.items()})
print(json.dumps(summary,ensure_ascii=False,indent=2))
for s in sessions:
    print(s['source_id'],s['run'],'done',s['unique_done'],'cancel',s['unique_cancelled'],'cloud',s['done_cloud_confirmed'],
          'historical',s['historical_result_messages'],'episodes',s['localization_episodes'],'manual',s['manual_required_episodes'],
          'confirm',s['confirm_stats']['median'],'cloudmed',s['cloud_stats']['median'])
