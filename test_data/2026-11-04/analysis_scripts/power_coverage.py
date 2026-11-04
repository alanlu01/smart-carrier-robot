"""Check that missing sensor flags are not mistaken for healthy readings."""
import collections
import json
import sqlite3
from pathlib import Path
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

cls = get_message('std_msgs/msg/String')
for meta in sorted(Path('/home/robot/test_logs').glob('20*/rosbag/metadata.yaml')):
    if meta.parent.parent.name[:8]<'20260922':
        continue
    md = yaml.safe_load(meta.read_text())['rosbag2_bagfile_information']
    def rows():
        if md['storage_identifier']=='sqlite3':
            for f in meta.parent.glob('*.db3'):
                db = sqlite3.connect(f'file:{f}?mode=ro',uri=True)
                tid = db.execute("SELECT id FROM topics WHERE name='/power_status'").fetchone()
                if tid:
                    for t,data in db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',tid):
                        yield t/1e9,data
                db.close()
        else:
            rd=rosbag2_py.SequentialReader()
            rd.open(rosbag2_py.StorageOptions(uri=str(meta.parent),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
            rd.set_filter(rosbag2_py.StorageFilter(topics=['/power_status']))
            while rd.has_next():
                n,data,t=rd.read_next()
                yield t/1e9,data
    counts=collections.defaultdict(collections.Counter)
    transitions=[]
    last={}
    sample=None
    for t,data in rows():
        s=json.loads(deserialize_message(data,cls).data)
        if sample is None: sample=s
        for ch in ('ch1','ch2','ch3'):
            x=s.get(ch,{})
            counts[ch]['samples']+=1
            if isinstance(x.get('sensor_ok'),bool):
                counts[ch]['known_sensor_flag']+=1
            if isinstance(x.get('current'),(float,int)):
                counts[ch]['known_current']+=1
            sig=(x.get('status'),x.get('sensor_ok'),x.get('present'))
            if sig != last.get(ch):
                transitions.append({'t':t,'channel':ch,'data':x})
                last[ch]=sig
    print(json.dumps({'run':meta.parent.parent.name,'coverage':counts,'sample':sample,'transitions':transitions},ensure_ascii=True),flush=True)
