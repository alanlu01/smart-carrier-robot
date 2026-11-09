"""Source freshness, startup evidence and on-demand TF retry regressions."""
import json
from types import SimpleNamespace as NS

from smart_delivery_core.serial_io import SerialReceiver
from smart_delivery_core.serial_recovery import FeedbackReadiness, fresh_feedback
from smart_delivery_core.navigation_health import navigation_snapshot, TARGETS
from smart_delivery_core.optional_idle_runtime import OptionalIdleRuntime
from smart_delivery_core.localization_manager import LocalizationManager
from smart_delivery_core.localization_persistence import write_cache, load_cache, select_same_boot_pose
from test_optional_idle_runtime import runtime


def test_control_heartbeat_cannot_refresh_old_wheel_evidence(monkeypatch):
    import smart_delivery_core.serial_io as module
    node = object.__new__(SerialReceiver)
    node.link = NS(snapshot=lambda _: dict(connected=True))
    node.session = 'session'
    node.feedback = FeedbackReadiness(98.)
    for now in (98.5, 99., 99.5):
        for letter in 'ABCD':
            node.feedback.observe(letter+'_RT: 0', now)
    node.complete_ros_ns = 99_500_000_000
    monkeypatch.setattr(module.time, 'time_ns', lambda: 99_600_000_000)
    p = node.control_feedback(99.6)
    assert p['stop_proven'] and fresh_feedback(p,99_600_000_000,0.)
    assert len(json.dumps(p)) < 512
    monkeypatch.setattr(module.time, 'time_ns', lambda: 100_300_000_000)
    stale = node.control_feedback(100.3)
    assert stale['sample_ros_stamp_ns'] == 100_300_000_000
    assert stale['complete_ros_stamp_ns'] == 99_500_000_000
    assert not stale['healthy'] and not fresh_feedback(stale,100_300_000_000,0.)


def test_stale_active_reply_and_inactive_node_have_distinct_reasons():
    records = {name:dict(id=3,label='active',at=100.) for name in TARGETS}
    assert navigation_snapshot(records,104.)['reason'] == 'response_stale'
    records['amcl']['id'] = 2
    assert navigation_snapshot(records,101.)['reason'] == 'lifecycle_inactive'
    assert navigation_snapshot({},101.)['reason'] == 'awaiting_responses'


def test_explicit_configured_pose_bypasses_cache_without_rearming_it(tmp_path):
    path = tmp_path/'pose.json'
    write_cache(path,'boot',pose=(2.,5.,.1),map_identity='map')
    node = NS(startup_pose_mode='configured',initial_pose_source='fixed_power_on',
              initial_x=0.,initial_y=0.,initial_yaw=0.,boot_id='boot',map_identity='map',
              pose_cache_path=str(path),auto_initialize=True)
    LocalizationManager._configure_startup_pose(node)
    assert node.initial_pose_source == 'configured_explicit'
    assert (node.initial_x,node.initial_y,node.initial_yaw) == (0.,0.,0.)
    assert select_same_boot_pose(load_cache(path),'boot',map_identity='map')[0] == 'same_boot_untrusted'


def test_tf_retry_timer_only_runs_while_pairing_is_pending(runtime):
    manager,_,_ = runtime
    flags = dict(cancelled=True,resets=0,cancels=0)
    def reset(): flags.update(cancelled=False,resets=flags['resets']+1)
    def cancel(): flags.update(cancelled=True,cancels=flags['cancels']+1)
    manager.scan_retry_timer = NS(is_canceled=lambda:flags['cancelled'],reset=reset,cancel=cancel)
    manager.state='idle'
    manager._sync_scan_retry_timer()
    assert flags['resets'] == 0
    manager.state='observing';manager.pending_scan=(100.,object())
    manager._sync_scan_retry_timer();manager._sync_scan_retry_timer()
    assert flags['resets'] == 1
    manager.pending_scan=None;manager._sync_scan_retry_timer()
    assert flags['cancelled'] and flags['cancels'] == 1
