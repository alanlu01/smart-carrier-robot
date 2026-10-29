"""Parking proof is necessary; AMCL micro-adjustments are not physical motion."""
from smart_delivery_core.standby_idle_clock import StandbyIdleClock


POINT=dict(name='A',x=2.,y=5.18,yaw=0.)

def tick(clock,now,**changes):
    args=dict(parked=True,safe=True,stationary=True,
              pose=dict(x=2.16,y=5.18,yaw=.126),odom=(0.,0.,0.))
    args.update(changes)
    return clock.update(now,**args)


def test_recorded_sixteen_cm_seven_degree_drift_does_not_reset_verified_idle():
    clock=StandbyIdleClock()
    assert not tick(clock,0.)
    clock.mark_parked(POINT)
    for step in range(121):
        assert tick(clock,step*.5)
    assert clock.elapsed==60.


def test_actual_movement_wrong_map_and_no_parking_proof_reset():
    clock=StandbyIdleClock();clock.mark_parked(POINT)
    for step in range(20):tick(clock,step*.5)
    assert not tick(clock,10.,odom=(.03,0.,0.))
    assert clock.elapsed==0.
    tick(clock,10.5)
    assert not tick(clock,11.,safe=False)
    assert clock.elapsed==0.
    assert not tick(clock,11.5,parked=False)


def test_short_large_amcl_jitter_pauses_but_persistent_deviation_resets():
    clock=StandbyIdleClock();clock.mark_parked(POINT)
    for step in range(20):tick(clock,step*.5)
    before=clock.elapsed
    assert not tick(clock,10.,pose=dict(x=2.25,y=5.18,yaw=0.))
    assert tick(clock,10.5) and clock.elapsed==before
    assert not tick(clock,11.,pose=dict(x=3.,y=5.18,yaw=0.))
    assert not tick(clock,13.,pose=dict(x=3.,y=5.18,yaw=0.))
    assert clock.elapsed==0.


def test_executor_gap_cannot_count_as_healthy_stationary_time():
    clock=StandbyIdleClock();clock.mark_parked(POINT)
    tick(clock,0.);tick(clock,.5)
    tick(clock,30.)
    assert clock.elapsed==0.


def test_short_tf_gap_preserves_only_existing_elapsed_time():
    clock=StandbyIdleClock();clock.mark_parked(POINT)
    tick(clock,0.);tick(clock,.5);before=clock.elapsed
    assert not tick(clock,1.,safe=False,pose=None,transient_tf_gap=True)
    assert clock.elapsed==before
    assert tick(clock,1.5) and clock.elapsed==before
    assert tick(clock,2.) and clock.elapsed==before+.5


def test_persistent_gap_and_movement_still_reset_idle_time():
    clock=StandbyIdleClock();clock.mark_parked(POINT)
    tick(clock,0.);tick(clock,.5)
    tick(clock,1.,safe=False,pose=None,transient_tf_gap=True)
    tick(clock,2.1,safe=False,pose=None,transient_tf_gap=True)
    assert clock.elapsed==0.
    tick(clock,2.5);tick(clock,3.)
    tick(clock,3.5,safe=False,pose=None,odom=(.03,0,0),transient_tf_gap=True)
    assert clock.elapsed==0.


def test_returning_fresh_after_one_second_cannot_preserve_unobserved_time():
    clock=StandbyIdleClock();clock.mark_parked(POINT)
    tick(clock,0.);tick(clock,.5)
    tick(clock,1.,safe=False,pose=None,transient_tf_gap=True)
    tick(clock,2.5)
    assert clock.elapsed==0.
