"""A restarted AMCL must never require stale poses before it can initialize."""
from types import SimpleNamespace as NS
from test_localization_recovery import manager
from smart_delivery_core.navigation_retry import transient_navigation_failure, schedule_tf_retry
from smart_delivery_core.delivery_console import localization_wait_label
from smart_delivery_core.recovery_spin import RecoverySpin
import math


def prepare(node,clock):
    node.state='MANUAL_REQUIRED';node.manual_intervention_kind='pipeline'
    node.last_amcl_at=0.;node.auto_motion_recovery=True
    node.navigation_health={'state':'ready'};node.navigation_health_at=clock.now
    node._lateral_sensor_blockers=lambda _:[];node._lateral_stop_ready=lambda _:True
    node.global_localization_client=NS(service_is_ready=lambda:True)
    node.get_clock=lambda:NS(now=lambda:NS(nanoseconds=int(clock.now*1e9)))
    node._cancel_navigation=lambda:None


def test_bootstrap_requires_two_seconds_safe_stop_then_new_source_pose(manager):
    n,t=manager;prepare(n,t)
    assert n._tick_pipeline_bootstrap(t.now) and not n.started_spins
    t.now+=2.1;n.navigation_health_at=t.now
    assert n._tick_pipeline_bootstrap(t.now)
    assert n.started_spins==['global'] and not n.ready
    n.last_amcl_at=t.now+.1;n.latest_pose_observed_at=t.now-10.
    assert n._tick_pipeline_bootstrap(t.now+.2)  # late old pose is not recovery
    n.latest_pose_observed_at=t.now+.1
    assert not n._tick_pipeline_bootstrap(t.now+.2)
    assert not n.ready  # ordinary global geometry/six-sample verification still needed


def test_sensor_gap_resets_bootstrap_quiet_window(manager):
    n,t=manager;prepare(n,t)
    n._tick_pipeline_bootstrap(t.now)
    t.now+=1.;n._lateral_sensor_blockers=lambda _:['stale_scan']
    assert not n._tick_pipeline_bootstrap(t.now)
    t.now+=1.;n.navigation_health_at=t.now;n._lateral_sensor_blockers=lambda _:[]
    n._tick_pipeline_bootstrap(t.now)
    assert not n.started_spins


def test_bootstrap_timeout_stops_without_repeated_global_reset(manager):
    n,t=manager;prepare(n,t)
    n.pipeline_bootstrap_started=80.;n.pipeline_bootstrap_source_ns=80_000_000_000
    assert n._tick_pipeline_bootstrap(t.now)
    assert n.manual_reasons and not n.started_spins
    assert n.pipeline_bootstrap_started is None


def test_only_explicit_transport_failures_retry_with_persistent_bound():
    assert transient_navigation_failure(0,'Timed out while waiting for action server to acknowledge goal request for follow_path')
    assert transient_navigation_failure(0,'Failed to get result for follow_path in node halt!')
    for msg in ('Failed to make progress','No valid path','Goal outside map','Collision detected'):
        assert not transient_navigation_failure(0,msg)
    task={}
    assert schedule_tf_retry(task,100.) and schedule_tf_retry(task,110.)
    assert not schedule_tf_retry(task,120.)
    assert localization_wait_label('MANUAL_REQUIRED','pipeline','等待 AMCL 姿態恢復')=='等待 AMCL 姿態恢復'


def test_overshoot_cannot_be_reported_as_precise_completion():
    turn=RecoverySpin(2*math.pi,0.,0.,1)
    turn.travel=math.radians(365.)
    assert turn.stopped(10.,True)=='fail' and turn.reason=='endpoint_overshoot'
