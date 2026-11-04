"""Read-only inventory of robot recordings from September 22, 2026."""
import datetime
import json
from pathlib import Path
import yaml

root = Path('/home/robot/test_logs')
for run in sorted(root.iterdir()):
    if not run.is_dir() or not (run.name.startswith(('202609', '202610')) or run.name.startswith('validation_202609')):
        continue
    if run.name[:8].isdigit() and run.name[:8] < '20260922':
        continue
    bags = []
    for meta in run.rglob('metadata.yaml'):
        d = yaml.safe_load(meta.read_text())['rosbag2_bagfile_information']
        topics = [{'name': x['topic_metadata']['name'], 'type': x['topic_metadata']['type'], 'count': x['message_count']} for x in d['topics_with_message_count']]
        bags.append({'uri': str(meta.parent), 'storage': d['storage_identifier'], 'start': d['starting_time']['nanoseconds_since_epoch']/1e9,
                     'duration': d['duration']['nanoseconds']/1e9, 'messages': d['message_count'], 'topics': topics,
                     'files': [{'name': f.name, 'bytes': f.stat().st_size} for f in meta.parent.iterdir() if f.suffix in ('.mcap', '.db3')]})
    small = []
    for f in run.iterdir():
        if f.is_file() and f.stat().st_size < 300000 and f.suffix in ('.txt', '.log', '.json', '.yaml'):
            small.append({'name': f.name, 'bytes': f.stat().st_size, 'preview': f.read_text(errors='replace')[:700]})
    loose = [{'path': str(f), 'bytes': f.stat().st_size} for f in run.rglob('*') if f.is_file() and f.suffix in ('.mcap', '.db3')]
    print(json.dumps({'run': run.name, 'bags': bags, 'files': small, 'all_storage_files': loose}, ensure_ascii=False), flush=True)
