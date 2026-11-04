"""Read-only source checks and ASCII-only transport for archived log evidence."""
import hashlib
import json
import re
import tarfile
from pathlib import Path
import yaml

uuid = r'[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}'
patterns = re.compile(uuid + r':\s*(done|failed|cancelled|released)\b')
stamp_pat = re.compile(r'\[(\d{10}\.\d+)\].*?\[([^]]+)\]: (.*)')
for meta in sorted(Path('/home/robot/test_logs').glob('20*/rosbag/metadata.yaml')):
    root = meta.parent.parent
    if root.name[:8] < '20260922':
        continue
    d = yaml.safe_load(meta.read_text())['rosbag2_bagfile_information']
    start = d['starting_time']['nanoseconds_since_epoch']/1e9
    end = start+d['duration']['nanoseconds']/1e9
    evidence, warnings, cpu = [], [], []
    def process(txt, name):
        for line in txt.splitlines():
            match = stamp_pat.search(line)
            if match:
                t, node, msg = match.groups()
                t = float(t)
                if start-1200 <= t <= end+120:
                    ack = patterns.search(msg)
                    if ack:
                        evidence.append({'t': t, 'task_id': ack.group(0).split(':')[0], 'status': ack.group(1), 'node': node, 'message': msg, 'file': name})
            if 'rosbag2' in line.lower() and any(w in line.lower() for w in ('lost', 'qos', 'incompatib')):
                warnings.append({'file': name, 'line': line})
    archive = root/'ros_logs.tar.gz'
    if archive.exists():
        with tarfile.open(archive, 'r:gz') as tar:
            for member in tar:
                if member.isfile() and member.name.endswith('.log') and member.size<40000000:
                    process(tar.extractfile(member).read().decode('utf-8', errors='replace'), member.name)
    else:
        for f in Path('/home/robot/.ros/log').glob('*.log'):
            if start-3600 <= f.stat().st_mtime <= end+12000:
                process(f.read_text(errors='replace'), str(f))
    if (root/'rosbag_console.log').exists():
        process((root/'rosbag_console.log').read_text(errors='replace'), 'rosbag_console.log')
    if (root/'system_top.log').exists():
        for line in (root/'system_top.log').read_text(errors='replace').splitlines():
            if '%Cpu(s)' in line:
                m = re.search(r'([\d.,]+)\s*id', line)
                if m:
                    cpu.append(100-float(m.group(1).replace(',', '.')))
    print(json.dumps({'run': root.name, 'metadata_sha256': hashlib.sha256(meta.read_bytes()).hexdigest(), 'cloud_ack_logs': evidence,
                     'recording_warnings': warnings, 'cpu_samples': cpu}, ensure_ascii=True), flush=True)
