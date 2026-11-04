"""Independent read-only verification of saved XLSX and source aggregations."""
import sys
import collections
import json
import re
import zipfile
from pathlib import Path
import openpyxl

root=Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).resolve().parents[1]/'quantitative_20260922_20261003'
d=json.loads((root/'analysis_data.json').read_text(encoding='utf-8'))
book=root/'FlowCharge_rosbag量化結果_20260922_20261003.xlsx'
w=openpyxl.load_workbook(book,data_only=True)
f=openpyxl.load_workbook(book,data_only=False)
assert len(w.sheetnames)==8
assert w['量化摘要']['B10'].value==72
assert w['量化摘要']['B11'].value==69
assert w['量化摘要']['B13'].value==1
assert w['量化摘要']['B20'].value==6
assert len({o['task_id'] for o in d['orders']})==72
assert len({o['task_id'][:8] for o in d['orders']})==72
assert sum(s['unique_done'] for s in d['sessions'])==69
assert sum(s['done_cloud_confirmed'] for s in d['sessions'])==69
assert sum(s['messages'] for s in d['sessions'])==2967080
assert all(p['samples']==p['known_sensor_flags'] for p in d['power'])
errors=[]
for s in w:
    for row in s:
        for c in row:
            if c.data_type=='e': errors.append((s.title,c.coordinate,c.value))
assert not errors,errors
for idx,s in enumerate(d['sessions'],7):
    assert w['測試批次'].cell(idx,8).value==s['unique_done']
    assert w['測試批次'].cell(idx,9).value==s['unique_cancelled']
    assert w['測試批次'].cell(idx,10).value==s['done_cloud_confirmed']
for idx,o in enumerate(d['orders'],7):
    for c,k in ((10,'confirm_to_result_s'),(12,'cloud_ack_delay_s')):
        v=w['訂單明細'].cell(idx,c).value
        if o[k] is None:
            assert v=='n.a.'
        else:
            assert abs(v-o[k])<.002,(idx,k,v,o[k])
with zipfile.ZipFile(book) as z:
    charts=[n for n in z.namelist() if re.fullmatch(r'xl/(?:drawings/)?charts/chart\d+\.xml',n)]
    assert len(charts)==1
    xml=z.read(charts[0]).decode()
    assert '355C85' in xml and 'C78736' in xml
    assert '量化摘要' in xml
stats={'saved_workbook_sheets':len(w.sheetnames),'rows_checked':72,'formula_errors':errors,
       'charts':len(charts),'order_types':dict(collections.Counter(o['type'] for o in d['orders'])),
       'done_by_type':dict(collections.Counter(o['type'] for o in d['orders'] if o['result_status']=='done')),
       'statistical_notes':'Nonuniform tests with human assistance; not autonomous accuracy or reliability trials.'}
print(json.dumps(stats,ensure_ascii=False,indent=2))
