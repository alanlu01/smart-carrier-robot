"""No launches: prove lock ownership, stale file reuse and domain isolation."""

import json
from pathlib import Path

import pytest

from smart_delivery_core.navigation_launch_guard import NavigationLaunchLease


def guard(tmp_path, domain='30'):
    (tmp_path/'proc').mkdir(exist_ok=True)
    return NavigationLaunchLease(tmp_path/'locks',tmp_path/'proc',domain)


def process(tmp_path, domain='30', name='component_container_isolated', node='nav2_container'):
    p=tmp_path/'proc/1234';p.mkdir(parents=True)
    p.joinpath('cmdline').write_bytes(f'/opt/ros/jazzy/lib/{name}\0--ros-args\0-r\0__node:={node}\0'.encode())
    p.joinpath('environ').write_bytes(f'ROS_DOMAIN_ID={domain}\0SECRET=not_logged\0'.encode())


def test_second_launch_refused_until_owner_releases_without_unlinking(tmp_path):
    first=guard(tmp_path).acquire()
    path=tmp_path/'locks/navigation_domain_30.lock';inode=path.stat().st_ino
    try:
        with pytest.raises(RuntimeError,match='拒絕重複啟動'):
            guard(tmp_path).acquire()
    finally:
        first.close()
    second=guard(tmp_path).acquire();second.close()
    assert path.stat().st_ino==inode and json.loads(path.read_text())['domain']=='30'


def test_other_domain_uses_independent_lock(tmp_path):
    first=guard(tmp_path).acquire();second=guard(tmp_path,'31').acquire()
    second.close();first.close()


def test_unguarded_existing_container_is_detected_and_never_killed(tmp_path):
    process(tmp_path)
    with pytest.raises(RuntimeError,match='尚未結束') as error:
        guard(tmp_path).acquire()
    assert '1234' in str(error.value) and 'SECRET' not in str(error.value)
    assert (tmp_path/'proc/1234/cmdline').exists()
    # A rejected attempt did not leak ownership.
    (tmp_path/'proc/1234/cmdline').write_bytes(b'other\0')
    owner=guard(tmp_path).acquire();owner.close()


@pytest.mark.parametrize('name',['bt_navigator','controller_server','amcl','map_server'])
def test_standalone_nav2_nodes_also_prevent_conflicting_bringup(tmp_path,name):
    process(tmp_path,name=name)
    with pytest.raises(RuntimeError,match='尚未結束'):
        guard(tmp_path).acquire()


def test_other_domain_and_non_navigation_containers_are_not_blocked(tmp_path):
    process(tmp_path,domain='31')
    owner=guard(tmp_path).acquire();owner.close()
    (tmp_path/'proc/1234/environ').write_bytes(b'ROS_DOMAIN_ID=30\0')
    (tmp_path/'proc/1234/cmdline').write_bytes(b'/bin/component_container\0__node:=camera_container\0')
    owner=guard(tmp_path).acquire();owner.close()


@pytest.mark.parametrize('domain',['../escape','30/1','-1','233'])
def test_invalid_domain_cannot_select_a_lock_path(tmp_path,domain):
    with pytest.raises(RuntimeError):guard(tmp_path,domain)
