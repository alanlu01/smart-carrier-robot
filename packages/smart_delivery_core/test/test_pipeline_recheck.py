"""Production outage verifier: no hardware, no permissive pose or stale-data bypass."""
import pytest
from smart_delivery_core.pipeline_recheck import PipelineRecheck, map_odom_pose


def observe(guard, at, **changes):
    args=dict(available=True,stopped=True,healthy=True,new_amcl=True,map_at=at,
              transform=(0.,0.,0.))
    args.update(changes)
    return guard.observe(at,**args)


def test_six_independent_stopped_observations_resume_without_pose_reset():
    g=PipelineRecheck(100.,100000000000,(0.,0.,0.))
    for at in range(101,106):assert observe(g,float(at))=='wait'
    assert observe(g,106.)=='resume'
    assert g.samples==6


def test_repeated_map_timestamp_and_pre_outage_amcl_cannot_resume():
    g=PipelineRecheck(100.,100000000000,(0.,0.,0.))
    for _ in range(10):assert observe(g,101.,map_at=101.)=='wait'
    assert g.samples==1
    assert observe(g,102.,new_amcl=False)=='wait' and g.samples==0


def test_repeated_amcl_header_with_new_maps_cannot_create_six_confirmations():
    g=PipelineRecheck(100.,100000000000,(0.,0.,0.))
    for at in range(101,108):
        assert observe(g,float(at),amcl_stamp_ns=101000000000)=='wait'
    assert g.samples==1


@pytest.mark.parametrize('fault', ['available','stopped','healthy','new_amcl'])
def test_each_safety_condition_blocks_release_and_breaks_confirmation(fault):
    g=PipelineRecheck(100.,100000000000,(0.,0.,0.))
    for at in range(101,106):observe(g,float(at))
    assert observe(g,106.,**{fault:False})=='wait'
    assert g.samples==0


def test_actual_map_odom_jump_goes_to_pose_search_instead_of_resume():
    g=PipelineRecheck(100.,100000000000,(0.,0.,0.))
    assert observe(g,101.,transform=(1.,0.,0.))=='wait'
    assert observe(g,109.1,transform=(1.,0.,0.))=='pose_recovery'
    assert g.samples==0


def test_intermittent_data_cannot_restart_fixed_deadline_forever():
    g=PipelineRecheck(100.,100000000000,(0.,0.,0.))
    observe(g,101.)
    observe(g,110.,available=False)
    observe(g,120.,available=False)
    assert observe(g,130.1,available=False)=='manual_pipeline'


def test_without_prior_trusted_reference_never_accept_a_high_score_alone():
    g=PipelineRecheck(100.,100000000000,None)
    for at in range(101,109):assert observe(g,float(at))=='wait'
    assert observe(g,109.)=='pose_recovery'


def test_map_odom_reference_compensates_real_ordinary_motion():
    assert map_odom_pose((2.,5.,0.),(1.,1.,0.))==pytest.approx((1.,4.,0.))
    assert map_odom_pose((2.1,5.,0.),(1.1,1.,0.))==pytest.approx((1.,4.,0.))


def test_waiting_for_data_pauses_verification_budget_but_not_total_deadline():
    g=PipelineRecheck(100.,100000000000,(0.,0.,0.))
    observe(g,101.)
    observe(g,102.)
    observe(g,110.,available=False)
    assert observe(g,115.)=='wait'
    assert g.verification_elapsed==pytest.approx(1.)
    assert observe(g,131.,available=False)=='manual_pipeline'
