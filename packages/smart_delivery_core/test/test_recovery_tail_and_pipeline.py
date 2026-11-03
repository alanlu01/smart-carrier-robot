"""Actual manager methods: tail completion and pipeline faults, no motion I/O."""
from types import SimpleNamespace
from smart_delivery_core.recovery_translation import lateral_speed
from smart_delivery_core.localization_manager import LocalizationManager
from smart_delivery_core.delivery_console import localization_label
from test_localization_recovery import manager,lateral_harness


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
