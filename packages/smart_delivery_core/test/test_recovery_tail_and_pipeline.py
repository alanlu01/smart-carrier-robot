"""Actual manager methods: tail completion and pipeline faults, no motion I/O."""
from types import SimpleNamespace
from smart_delivery_core.recovery_translation import lateral_speed
from smart_delivery_core.localization_manager import LocalizationManager
from smart_delivery_core.delivery_console import localization_label
from test_localization_recovery import manager,lateral_harness
from smart_delivery_core.pipeline_recheck import PipelineRecheck, TrustedPipelineReference


def test_recorded_273556_mm_tail_finishes_without_requiring_unachievable_small_command():
    assert lateral_speed(.2735564237599696)==0.
    assert lateral_speed(.270)>0.
    assert lateral_speed(.31)==0.


def test_pipeline_loss_blocks_verification_even_while_odometry_timer_is_fresh(manager):
    n,c=manager
    n.recovery_feedback=SimpleNamespace(last_complete_at=99.,is_stale=lambda now,limit:True)
    assert 'chassis_feedback_unavailable' in n._verification_data_blockers(c.now)
    n.state='LOCALIZED';n._tick_state_machine()
    assert n.started_spins and '資料管線' in n.started_spins[-1]


def test_recovered_pipeline_reenters_verification_without_sending_a_new_seed(manager):
    n,c=manager
    n.state='MANUAL_REQUIRED';n.manual_intervention_kind='pipeline'
    n._set_state=lambda state,reason:setattr(n,'state',state)
    n._tick_state_machine()
    assert n.state=='VERIFYING' and n.stable_samples==0 and not n.ready
    assert n.verification_pose_anchor==n.latest_pose and not n.started_spins


def test_pipeline_manual_pose_is_received_but_does_not_override_missing_feedback(manager):
    n,c=manager
    n.state='MANUAL_REQUIRED';n.manual_intervention_kind='pipeline'
    n.manual_initial_pose_pending=False
    n.recovery_feedback=SimpleNamespace(last_complete_at=None)
    n._external_initial_pose_callback(None)
    assert n.state=='MANUAL_REQUIRED' and not n.ready
    assert '資料中斷' in localization_label(n.state,'pipeline')


def test_inactive_or_missing_navigation_health_keeps_dispatch_stopped(manager):
    n,c=manager
    n.navigation_health=dict(state='unavailable');n.navigation_health_at=100.
    assert 'nav2_services_unavailable' in n._verification_data_blockers(c.now)
    n.navigation_health=dict(state='ready');n.navigation_health_at=90.
    assert 'nav2_services_unavailable' in n._verification_data_blockers(c.now)


def test_real_pipeline_entry_keeps_history_even_if_fault_happened_while_driving(manager):
    n,c=manager
    n.state='LOCALIZED';n.commanded_motion=n.odom_motion=True
    n.last_odom_pose=(0.,0.,0.)
    n.pipeline_trusted_reference=TrustedPipelineReference(99.,99000000000,(1.,4.,0.),
        n.last_odom_pose,.2,.1,.8)
    n.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=100000000000))
    n.cancelled=[];n._cancel_navigation=lambda:n.cancelled.append(True)
    n._set_state=lambda state,reason:setattr(n,'state',state)
    before=list(n.map_score_window)
    LocalizationManager._begin_recovery(n,'chassis_feedback_unavailable',sensor_pipeline_timeout=True)
    assert n.pipeline_recheck is not None and n.recovery_step=='pipeline_wait'
    assert n.pipeline_recheck.required==6 and n.pipeline_recheck.reference==(1.,4.,0.)
    assert list(n.map_score_window)==before and n.cancelled==[True]
    assert n.state=='RECOVERING_LOCAL' and not n.started_spins
    assert localization_label(n.state,n.manual_intervention_kind)=='資料中斷，等待停車驗證'


def prepare_pipeline(n):
    n.pipeline_recheck=PipelineRecheck(100.,100000000000,(1.,4.,0.))
    n.recovery_step='pipeline_wait'
    n._pipeline_transform=lambda:(1.,4.,0.)
    n._mild_map_sources_fresh=lambda:True
    n._sensor_tf_is_ready=lambda:True
    n.recovery_stop=SimpleNamespace(ready=lambda now:True)
    n.latest_map_score_raw=.8


def fresh_pipeline(n,c,at):
    c.now=at
    n.latest_raw_scan_at=n.latest_scan_at=n.latest_odom_at=n.last_amcl_at=n.latest_map_score_at=at
    n.latest_pose_observed_at=at
    n.latest_scan=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=int(at),nanosec=0)))


def test_actual_manager_pipeline_hold_resumes_after_six_new_observations(manager):
    n,c=manager;prepare_pipeline(n)
    for at in range(101,106):
        fresh_pipeline(n,c,float(at));n._tick_state_machine()
        assert not n.ready and not n.started_spins
    fresh_pipeline(n,c,106.);n._tick_state_machine()
    assert n.ready and n.state=='LOCALIZED' and not n.started_spins


def test_actual_manager_does_not_accept_kidnapped_transform_as_pipeline_resume(manager):
    n,c=manager;prepare_pipeline(n)
    n._pipeline_transform=lambda:(2.,4.,0.)
    for at in range(101,110):
        fresh_pipeline(n,c,float(at));n._tick_state_machine()
    assert not n.ready and n.started_spins==['資料恢復後位置仍不確定']


def test_pre_outage_mild_quality_anchor_does_not_release_until_new_healthy_evidence(manager):
    from geometry_msgs.msg import TransformStamped
    from smart_delivery_core.localization_health import pose_quality
    n,c=manager
    n.state='LOCALIZED';n.last_odom_pose=(0.,0.,0.)
    n.latest_pose_observed_at=c.now;n.latest_odom_stamp_ns=int(c.now*1e9)
    n.latest_map_score_raw=.9
    cov=[0.]*36;cov[0]=cov[7]=.304**2;cov[35]=.1**2
    n.latest_quality=pose_quality(cov,.25,.17,.5,.45)
    transform=TransformStamped();transform.header.stamp.sec=100
    transform.transform.rotation.w=1.
    n.tf_buffer=SimpleNamespace(lookup_transform=lambda *_a,**_k:transform)
    n._capture_pipeline_reference(c.now)
    assert getattr(n,'pipeline_trusted_reference',None) is None
    assert n.pipeline_continuity_reference.xy_std==.304
    n._cancel_navigation=lambda:None
    n._set_state=lambda state,reason:setattr(n,'state',state)
    LocalizationManager._begin_recovery(n,'test',sensor_pipeline_timeout=True)
    assert n.pipeline_recheck.reference_evidence['kind']=='continuity'
    n._pipeline_transform=lambda:(0.,0.,0.)
    n._mild_map_sources_fresh=n._sensor_tf_is_ready=lambda:True
    n.recovery_stop=SimpleNamespace(ready=lambda _:True)
    fresh_pipeline(n,c,101.);n._tick_state_machine()
    assert not n.ready and n.pipeline_recheck.samples==0  # Mild covariance cannot release.
    cov[0]=cov[7]=.2**2;n.latest_quality=pose_quality(cov,.25,.17,.5,.45)
    for at in range(102,107):
        fresh_pipeline(n,c,float(at));n._tick_state_machine()
        assert not n.ready and not n.started_spins
    fresh_pipeline(n,c,107.);n._tick_state_machine()
    assert n.ready and not n.started_spins


def test_critical_or_post_outage_data_cannot_create_a_continuity_reference(manager):
    n,c=manager;n.state='RECOVERING_LOCAL';n.latest_map_score_raw=.9
    n._capture_pipeline_reference(c.now)
    assert getattr(n,'pipeline_continuity_reference',None) is None
    n.state='LOCALIZED'
    n.latest_quality=SimpleNamespace(critical=True)
    n._capture_pipeline_reference(c.now)
    assert getattr(n,'pipeline_continuity_reference',None) is None
