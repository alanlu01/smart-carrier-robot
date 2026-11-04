import collections
import json
from pathlib import Path

rows = []
for line in Path('tmp/quantitative_20261003/extracted.jsonl').read_text(encoding='utf-8-sig').splitlines():
    try:
        rows.append(json.loads(line))
    except json.JSONDecodeError:
        pass
for r in rows:
    print(r['run'], 'tasks', len(r['tasks']), 'states', len(r['localization_events']),
          'results', collections.Counter(x['data'].get('status') for v in r['tasks'].values() for x in v['results']),
          'cpu', r['cpu_busy_percent'], 'errors', r['decode_errors'])
if rows:
    r = rows[-1]
    for tid, v in list(r['tasks'].items())[:2]:
        print('TASK', tid, json.dumps(v, ensure_ascii=False)[:3300])
    print('EVIDENCE', json.dumps(r['text_result_evidence'][-7:], ensure_ascii=False))
    print('WARN', r['recording_warnings'])
    print('LOC', json.dumps(r['localization_events'][:2], ensure_ascii=False)[:4300])
