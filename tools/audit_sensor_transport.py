"""Read-only process/profile audit. No ROS node, no secrets, no controls."""

import hashlib
import json
from pathlib import Path
from xml.etree import ElementTree as ET

KEYS = ('ROS_DOMAIN_ID','RMW_IMPLEMENTATION','RMW_FASTRTPS_USE_QOS_FROM_XML',
        'RMW_FASTRTPS_PUBLICATION_MODE','FASTRTPS_DEFAULT_PROFILES_FILE',
        'FASTDDS_DEFAULT_PROFILES_FILE')
SENSORS = ('scan_to_scan_filter_chain','odom_transport','mecanum_odom_real')


def inspect_processes(proc_root='/proc'):
    """Report selected environment keys only; file content is XML policy only."""
    rows = []
    for process in Path(proc_root).iterdir():
        if not process.name.isdecimal():
            continue
        try:
            words = process.joinpath('cmdline').read_bytes().split(b'\0')
            # Python console entrypoints can be argv[1]. Do not match shell text.
            names = [Path(w.decode(errors='replace')).name for w in words[:2] if w]
            name = next((name for name in names if name in SENSORS),None)
            if name is None:
                continue
            env = dict(entry.decode(errors='replace').split('=',1)
                       for entry in process.joinpath('environ').read_bytes().split(b'\0') if b'=' in entry)
            row = {'pid':int(process.name),'name':name,
                   'environment':{key:env.get(key) for key in KEYS},
                   'configuration_only':True,'runtime_effect_not_proven':True}
            path = env.get('FASTDDS_DEFAULT_PROFILES_FILE') or env.get('FASTRTPS_DEFAULT_PROFILES_FILE')
            if path:
                data = Path(path).read_bytes()
                root = ET.fromstring(data)
                policy = lambda tag:[node.text for node in root.iter() if node.tag.split('}')[-1]==tag]
                row.update(profile_sha256=hashlib.sha256(data).hexdigest(),
                           publish_mode=policy('kind'),udp_non_blocking_send=policy('non_blocking_send'))
            rows.append(row)
        except (OSError,ValueError,ET.ParseError) as exc:
            rows.append({'pid':int(process.name),'error':str(exc)[:300]})
    return rows


if __name__ == '__main__':
    print(json.dumps({'processes':inspect_processes()},ensure_ascii=False,indent=2))
