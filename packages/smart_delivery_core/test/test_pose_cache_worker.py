"""Blocked storage must not block AMCL or accumulate an unbounded pose queue."""

import threading
import time

from smart_delivery_core.localization_persistence import LatestPoseWriter, load_cache, select_same_boot_pose, write_cache


def test_blocked_writer_retains_only_latest_original_observation(tmp_path):
    entered, release = threading.Event(), threading.Event()
    calls = []
    path = tmp_path/'pose.json'
    def blocked(*args, **kwargs):
        calls.append((args[2], kwargs['now']))
        if len(calls) == 1:
            entered.set()
            assert release.wait(3.)
        write_cache(*args, **kwargs)
    writer = LatestPoseWriter(path, 'boot', 'map', writer=blocked)
    try:
        assert writer.submit((1,2,0), 100.)
        assert entered.wait(1.)
        start = time.monotonic()
        for at in range(101, 201):
            assert writer.submit((at,2,0), float(at))
        assert time.monotonic()-start < .2
        state = writer.snapshot()
        assert state['active'] and state['pending'] and state['replaced'] == 99
        release.set()
        with writer.condition:
            assert writer.condition.wait_for(lambda: writer.completed == 2, timeout=2.)
        assert calls == [((1.,2.,0.),100.), ((200.,2.,0.),200.)]
        data = load_cache(path)
        assert data['saved_at'] == 200. and data['map_identity'] == 'map'
        assert select_same_boot_pose(data, 'boot', now=1000., max_age_sec=60., map_identity='map')[0] == 'same_boot_untrusted'
        assert {'serialize','file_write','atomic_replace','total_write'} <= writer.snapshot()['wall_durations'].keys()
    finally:
        release.set()
        assert writer.close()


def test_write_error_is_visible_and_next_pose_can_succeed(tmp_path):
    def failing(*args, **kwargs):
        if args[2][0] == 1:
            raise OSError('injected unavailable storage')
        write_cache(*args, **kwargs)
    writer = LatestPoseWriter(tmp_path/'pose.json', 'boot', 'map', writer=failing)
    try:
        writer.submit((1,0,0),100.)
        with writer.condition:
            assert writer.condition.wait_for(lambda: writer.failures == 1, timeout=2.)
        assert 'unavailable storage' in writer.snapshot()['last_error']
        writer.submit((2,0,0),101.)
        with writer.condition:
            assert writer.condition.wait_for(lambda: writer.completed == 1, timeout=2.)
        assert writer.snapshot()['last_error'] == ''
    finally:
        assert writer.close()


def test_shutdown_is_bounded_and_rejects_late_submissions(tmp_path):
    entered, release = threading.Event(), threading.Event()
    def blocked(*args, **kwargs):
        entered.set()
        release.wait(3.)
    writer = LatestPoseWriter(tmp_path/'pose.json', 'boot', 'map', writer=blocked)
    writer.submit((1,0,0),100.)
    assert entered.wait(1.)
    try:
        assert not writer.close(timeout=.01)
        assert not writer.submit((2,0,0),101.)
    finally:
        release.set()
        assert writer.close()
