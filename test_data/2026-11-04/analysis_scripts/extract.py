"""Streaming read-only ROS analysis. JSON output only; never writes on the Pi."""
import collections
import json
import math
import re
import sqlite3
import tarfile
from pathlib import Path

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPICS = {'/scan', '/scan_filtered', '/stm32_data', '/odom', '/amcl_pose', '/tf',
          '/power_status', '/vehicle_battery_status', '/order', '/localization/state',
          '/localization/ready', '/localization/motion_inhibited', '/smart_carrier/task_state',
          '/smart_carrier/task_result', '/smart_carrier/task_result_ack', '/smart_carrier/task_cancel',
          '/delivery/standby_state', '/chassis/feedback_health', '/vision/pipeline_health',
          '/semantic/pipeline_health', '/semantic/speed_multiplier', '/vision/semantic_info',
          '/cmd_vel', '/cmd_vel_nav', '/cmd_vel_smoothed', '/chassis_cmd_vel', '/rosout',
          '/initialpose', '/navigate_to_pose/_action/status', '/spin/_action/status'}


def stats(vals):
    a = np.asarray(vals, dtype=float)
    a = a[np.isfinite(a)]
    if not len(a):
        return None
    return {k: float(v) for k, v in [('min', a.min()), ('median', np.median(a)), ('p95', np.percentile(a, 95)), ('max', a.max()), ('mean', a.mean())]} | {'n': len(a)}


def read_bag(path, storage):
    if storage == 'sqlite3':
        # Explicitly prohibit writes, including SQLite sidecar creation.
        for f in sorted(path.glob('*.db3')):
            db = sqlite3.connect(f'file:{f}?mode=ro', uri=True)
            ts = {i: (n, ty) for i, n, ty in db.execute('SELECT id,name,type FROM topics')}
            ids = [i for i, (n, _) in ts.items() if n in TOPICS]
            query = 'SELECT topic_id,data,timestamp FROM messages WHERE topic_id IN (' + ','.join(map(str, ids)) + ') ORDER BY timestamp'
            for tid, data, ns in db.execute(query):
                yield ts[tid][0], ts[tid][1], data, ns
            db.close()
    else:
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=storage), rosbag2_py.ConverterOptions('', ''))
        ty = {x.name: x.type for x in reader.get_all_topics_and_types()}
        reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(TOPICS & ty.keys())))
        while reader.has_next():
            name, data, ns = reader.read_next()
            yield name, ty[name], data, ns


def analyze(meta):
    root = meta.parent.parent
    md = yaml.safe_load(meta.read_text())['rosbag2_bagfile_information']
    start = md['starting_time']['nanoseconds_since_epoch']/1e9
    end = start + md['duration']['nanoseconds']/1e9
    out = {'run': root.name, 'uri': str(meta.parent), 'start': start, 'end': end,
           'storage': md['storage_identifier'], 'duration_s': end-start, 'message_count': md['message_count'],
           'bytes': sum(f.stat().st_size for f in meta.parent.iterdir() if f.suffix in ('.db3', '.mcap')),
           'exit_code': (root/'rosbag_exit_code.txt').read_text().strip() if (root/'rosbag_exit_code.txt').exists() else None,
           'commit': (root/'git_commit.txt').read_text().strip() if (root/'git_commit.txt').exists() else None,
           'topics': [{'name': x['topic_metadata']['name'], 'type': x['topic_metadata']['type'], 'count': x['message_count']} for x in md['topics_with_message_count']]}
    classes = {}
    times = collections.defaultdict(list)
    ages = collections.defaultdict(list)
    tasks = collections.defaultdict(lambda: {'states': [], 'results': [], 'acks': [], 'claims': [], 'cancels': []})
    loc_events, loc_rows, initial, parked, batches, roslogs = [], [], [], [], {}, []
    health_fps, health_bad, pow_rows, mult_rows = [], [], [], []
    detections = collections.Counter()
    pose = odom = cmd = None
    mult = 1.0
    mult_time = None
    mult_duration = collections.Counter()
    commands = collections.Counter()
    match_half = []
    last_sig = {}
    decode_errors = collections.Counter()
    for name, ty, data, ns in read_bag(meta.parent, out['storage']):
        t = ns/1e9
        times[name].append(t)
        try:
            if ty not in classes:
                classes[ty] = get_message(ty)
            m = deserialize_message(data, classes[ty])
        except Exception as e:
            decode_errors[str(e)] += 1
            continue
        if hasattr(m, 'header'):
            stamp = m.header.stamp.sec + m.header.stamp.nanosec/1e9
            if stamp > 0:
                ages[name].append(t-stamp)
        if name == '/tf':
            for tr in m.transforms:
                if tr.header.frame_id == 'odom' and tr.child_frame_id in ('base_footprint', 'base_link'):
                    ages['odom_tf'].append(t-(tr.header.stamp.sec+tr.header.stamp.nanosec/1e9))
        elif name in ('/odom', '/amcl_pose', '/initialpose'):
            p = m.pose.pose.position
            q = m.pose.pose.orientation
            v = [p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)), t]
            if name == '/odom':
                odom = v
            elif name == '/amcl_pose':
                pose = v
            else:
                initial.append({'t': t, 'pose': v})
        elif name == '/cmd_vel':
            cmd = [t, m.linear.x, m.linear.y, m.angular.z]
        elif name == '/chassis_cmd_vel':
            # Pair only close-in-time nonzero input; do not claim this proves physical braking.
            if cmd and 0 <= t-cmd[0] <= .10 and math.hypot(cmd[1], cmd[2]) > .02:
                if mult == .5:
                    commands['half_nonzero_input'] += 1
                    err = max(abs(m.linear.x-cmd[1]*.5), abs(m.linear.y-cmd[2]*.5), abs(m.angular.z-cmd[3]*.5))
                    if err < .001:
                        commands['half_matching_output'] += 1
                    match_half.append(err)
                elif mult == 0:
                    commands['zero_nonzero_input'] += 1
                    if max(abs(m.linear.x), abs(m.linear.y), abs(m.angular.z)) < .001:
                        commands['zero_matching_output'] += 1
        elif name == '/semantic/speed_multiplier':
            if mult_time is not None:
                mult_duration[str(mult)] += max(0, min(t-mult_time, .5))
                mult_duration['unobserved_gap_s'] += max(0, t-mult_time-.5)
            mult = float(m.data)
            mult_time = t
            mult_rows.append((t, mult))
        elif name == '/rosout':
            # Compact evidence only; retain result ACK messages and operational warnings.
            if m.level >= 30 or any(x in m.msg.lower() for x in ('result','acked','acknowledg','回報','結果','雲端','已停妥')):
                roslogs.append({'t': t, 'level': int(m.level), 'node': m.name, 'message': m.msg})
        elif hasattr(m, 'data') and isinstance(m.data, str):
            try:
                s = json.loads(m.data)
            except (ValueError, TypeError):
                continue
            if name == '/localization/state':
                loc_rows.append((t, s.get('state')))
                sig = (s.get('state'), s.get('reason'), s.get('recovery_step'))
                if sig != last_sig.get(name):
                    loc_events.append({'t': t, 'data': s, 'pose': pose, 'odom': odom})
                    last_sig[name] = sig
            elif name == '/order':
                bid = str(s.get('batch_id', s.get('id', 'unknown')))
                if bid not in batches:
                    batches[bid] = {'t': t, 'data': s, 'pose': pose}
                    for o in s.get('orders', []):
                        tid = str(o.get('id', o.get('task_id')))
                        tasks[tid]['claims'].append({'t': t, 'batch_id': bid, 'data': o})
            elif name.startswith('/smart_carrier/task_'):
                tid = str(s.get('task_id', s.get('id')))
                if name.endswith('task_state'):
                    sig = json.dumps(s, sort_keys=True, ensure_ascii=False)
                    if last_sig.get((name, tid)) != sig:
                        tasks[tid]['states'].append({'t': t, 'data': s})
                        last_sig[name, tid] = sig
                elif name.endswith('task_result'):
                    tasks[tid]['results'].append({'t': t, 'data': s})
                elif name.endswith('task_result_ack'):
                    tasks[tid]['acks'].append({'t': t, 'data': s})
                elif name.endswith('task_cancel'):
                    tasks[tid]['cancels'].append({'t': t, 'data': s})
            elif name == '/delivery/standby_state':
                if s.get('state') == 'arrived' and pose:
                    parked.append({'t': t, 'data': s, 'pose': pose,
                                   'estimated_distance_m': math.hypot(pose[0]-s['x'], pose[1]-s['y']),
                                   'estimated_yaw_error_deg': math.degrees(math.atan2(math.sin(pose[2]-s['yaw']), math.cos(pose[2]-s['yaw'])))})
            elif name == '/power_status':
                pow_rows.append((t, s))
            elif name == '/vision/semantic_info':
                detections.update(d.get('class', '?') for d in s.get('detections', []))
            elif name == '/vision/pipeline_health':
                for k in ('inference_fps', 'fps', 'processing_fps'):
                    if isinstance(s.get(k), (int, float)):
                        health_fps.append(s[k])
                        break
            elif name == '/chassis/feedback_health':
                if s.get('state') not in ('healthy', None) or (s.get('feedback_age_sec') or 0) > .8:
                    health_bad.append({'t': t, 'data': s})
    timing = {}
    for n, ts in times.items():
        gaps = np.diff(ts)
        timing[n] = {'count': len(ts), 'first': ts[0], 'last': ts[-1], 'hz': (len(ts)-1)/(ts[-1]-ts[0]) if len(ts)>1 and ts[-1]>ts[0] else None,
                     'gap_s': stats(gaps), 'gaps_gt_05s': int(np.sum(gaps>.5)), 'gaps_gt_1s': int(np.sum(gaps>1)),
                     'age_s': stats(ages[n]) if n in ages else None,
                     'large_gaps': [{'t': ts[i], 'seconds': float(gaps[i])} for i in np.where(gaps>1)[0]]}
    out['timing'] = timing
    out['odom_tf_age_s'] = stats(ages.get('odom_tf', []))
    out['tasks'] = dict(tasks)
    out['batches'] = batches
    out['localization_events'] = loc_events
    state_duration = collections.Counter()
    for (ta, sa), (tb, _) in zip(loc_rows, loc_rows[1:]):
        state_duration[sa] += tb-ta
    if loc_rows:
        state_duration[loc_rows[-1][1]] += max(0, end-loc_rows[-1][0])
    out['localization_state_duration_s'] = dict(state_duration)
    out['localization_observed_s'] = end-loc_rows[0][0] if loc_rows else None
    out['initialpose'] = initial
    out['parked'] = parked
    out['vision_fps'] = stats(health_fps)
    out['detections'] = dict(detections)
    out['semantic_duration_s'] = dict(mult_duration)
    out['semantic_command_counts'] = dict(commands)
    out['semantic_half_error'] = stats(match_half)
    out['feedback_unhealthy'] = health_bad
    power = {}
    for k in ('ch1', 'ch2', 'ch3'):
        vals = [(t, s.get(k, {})) for t, s in pow_rows]
        bad = [(t, x) for t, x in vals if x.get('sensor_ok') is False]
        saturated = [(t, x) for t, x in vals if isinstance(x.get('current'), (float,int)) and x['current'] >= 1.637]
        power[k] = {'samples': len(vals), 'invalid_count': len(bad), 'invalid_events': [{'t': t, 'data': s} for t, s in bad[:8]],
                    'saturated_count': len(saturated), 'current_a': stats([x['current'] for _,x in vals if isinstance(x.get('current'),(float,int))]),
                    'status_counts': dict(collections.Counter(x.get('status') for _,x in vals))}
    vb = [(t, s.get('vehicle_battery', {})) for t,s in pow_rows]
    voltage = [(t,x['voltage']) for t,x in vb if isinstance(x.get('voltage'),(float,int))]
    power['vehicle'] = {'voltage_v': stats([v for _,v in voltage]), 'low_voltage_events': [{'t':t,'v':v} for t,v in voltage if v<9.5][:10],
                        'invalid_count': sum(s.get('sensor_ok') is False for _,s in vb), 'status_counts': dict(collections.Counter(s.get('status') for _,s in vb))}
    out['power'] = power
    out['rosout_evidence'] = roslogs
    # Read archived text logs for final cloud ACK evidence and shutdown classification.
    text_evidence = []
    archive = root/'ros_logs.tar.gz'
    if archive.exists():
        with tarfile.open(archive, 'r:gz') as tar:
            for member in tar:
                if not member.isfile() or not member.name.endswith('.log') or member.size > 40000000:
                    continue
                txt = tar.extractfile(member).read().decode('utf-8', errors='replace')
                for line in txt.splitlines():
                    match = re.search(r'\[(\d{10}\.\d+)\].*?\[([^]]+)\]: (.*)', line)
                    if match:
                        stamp, node, msg = match.groups()
                        stamp = float(stamp)
                        if start<=stamp<=end and any(x in msg.lower() for x in ('result', 'acked', 'acknowledg', '結果', '雲端', '成功停', '已停妥')):
                            text_evidence.append({'t':stamp, 'node':node, 'message':msg, 'file':member.name})
    out['text_result_evidence'] = text_evidence
    cpu = []
    if (root/'system_top.log').exists():
        for line in (root/'system_top.log').read_text(errors='replace').splitlines():
            if '%Cpu(s)' in line:
                idle = re.search(r'([\d.,]+)\s*id', line)
                if idle:
                    cpu.append(100-float(idle.group(1).replace(',', '.')))
    out['cpu_busy_percent'] = stats(cpu)
    loss = []
    for f in [root/'rosbag_console.log', root/'rosbag.log']:
        if f.exists():
            for line in f.read_text(errors='replace').splitlines():
                if any(x in line.lower() for x in ('lost','qos','incompatib')):
                    loss.append(line)
    # Old recorder log often contained in archived ROS logs.
    if archive.exists():
        with tarfile.open(archive, 'r:gz') as tar:
            for member in tar:
                if member.isfile() and 'ros2_' in member.name and member.size<5000000:
                    for line in tar.extractfile(member).read().decode(errors='replace').splitlines():
                        if any(x in line.lower() for x in ('lost', 'qos', 'incompatib')):
                            loss.append(line)
    out['recording_warnings'] = loss
    out['decode_errors'] = dict(decode_errors)
    return out


for meta in sorted(Path('/home/robot/test_logs').glob('20*/rosbag/metadata.yaml')):
    if meta.parent.parent.name[:8] >= '20260922':
        print(json.dumps(analyze(meta), ensure_ascii=True, allow_nan=False), flush=True)
