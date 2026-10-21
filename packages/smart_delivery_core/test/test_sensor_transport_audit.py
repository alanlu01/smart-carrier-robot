"""Audit only intended sensor processes; never persist API/environment secrets."""

import json
from pathlib import Path
import runpy
from xml.etree import ElementTree as ET


ROOT=Path(__file__).resolve().parents[3]
inspect_processes=runpy.run_path(str(ROOT/'tools/audit_sensor_transport.py'))['inspect_processes']


def test_audit_finds_python_and_cpp_entrypoints_and_whitelists_environment(tmp_path):
    profile=ROOT/'packages/smart_delivery_core/config/fastdds_sensor_publish.xml'
    for pid,command in [('1',b'/usr/bin/python3\0/opt/ros/lib/odom_transport\0'),
                        ('2',b'/opt/ros/lib/scan_to_scan_filter_chain\0')]:
        proc=tmp_path/pid;proc.mkdir()
        proc.joinpath('cmdline').write_bytes(command)
        proc.joinpath('environ').write_bytes((f'FASTDDS_DEFAULT_PROFILES_FILE={profile}\0'
            'RMW_FASTRTPS_USE_QOS_FROM_XML=1\0ROBOT_TOKEN=super_secret\0').encode())
    result=inspect_processes(tmp_path)
    assert {row['name'] for row in result}=={'odom_transport','scan_to_scan_filter_chain'}
    assert 'super_secret' not in json.dumps(result)
    assert all(row['udp_non_blocking_send']==['true'] for row in result)
    assert all('ASYNCHRONOUS' in row['publish_mode'] for row in result)
    assert all(row['configuration_only'] and row['runtime_effect_not_proven'] for row in result)


def test_xml_change_keeps_shared_memory_and_reliable_qos_unchanged():
    root=ET.parse(ROOT/'packages/smart_delivery_core/config/fastdds_sensor_publish.xml').getroot()
    values=lambda tag:[node.text for node in root.iter() if node.tag.split('}')[-1]==tag]
    assert 'SHM' in values('type') and 'UDPv4' in values('type')
    assert 'ASYNCHRONOUS' in values('kind')
    assert values('non_blocking_send')==['true']
    assert 'BEST_EFFORT' not in values('kind')  # No global reliability downgrade.
