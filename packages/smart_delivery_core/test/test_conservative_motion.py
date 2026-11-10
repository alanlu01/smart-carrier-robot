"""Exercise slow-feedback turns and consistency across planning and smoothing."""
import math
from pathlib import Path

import pytest
import yaml

from smart_delivery_core.recovery_spin import RecoverySpin


CONFIG = yaml.safe_load((Path(__file__).parents[1] / 'config/my_nav2_params.yaml').read_text(encoding='utf-8'))


def test_planner_and_smoother_share_physical_motion_limits():
    planner = CONFIG['controller_server']['ros__parameters']['FollowPath']
    smoother = CONFIG['velocity_smoother']['ros__parameters']
    assert [planner[k] for k in ('vx_max', 'vy_max', 'wz_max')] == smoother['max_velocity']
    assert [planner[k] for k in ('vx_min', 'vy_min')] == smoother['min_velocity'][:2]
    assert smoother['min_velocity'][2] == -planner['wz_max']
    assert [planner[k] for k in ('ax_max', 'ay_max', 'az_max')] == smoother['max_accel']
    assert planner['ax_min'] == smoother['max_decel'][0]
    assert planner['ay_min'] == smoother['max_decel'][1]
    assert all(-deceleration >= acceleration for acceleration, deceleration in
               zip(smoother['max_accel'], smoother['max_decel']))
    behavior = CONFIG['behavior_server']['ros__parameters']
    assert behavior['max_rotational_vel'] <= planner['wz_max']
    assert behavior['rotational_acc_lim'] <= planner['az_max']
    localization = CONFIG['localization_manager']['ros__parameters']
    assert localization['lateral_recovery_speed'] <= planner['vy_max']


@pytest.mark.parametrize('phase', [0., .1, .25, .49])
@pytest.mark.parametrize('sign', [-1., 1.])
def test_two_hz_wheels_complete_bounded_turn_in_both_directions(phase, sign):
    """Model held filtered wheel velocity, a 5 Hz supervisor and finite braking."""
    behavior = CONFIG['behavior_server']['ros__parameters']
    maximum = behavior['max_rotational_vel']
    acceleration = behavior['rotational_acc_lim']
    active_limit = CONFIG['localization_manager']['ros__parameters']['spin_time_allowance_sec']
    turn = RecoverySpin(sign*2*math.pi, 0., 0., 1, active_limit)
    rate = filtered = held = physical_yaw = odom_yaw = 0.
    last_sample = next_supervisor = 0.
    next_sample, next_odom = phase, .05
    stopped_since = None
    commands = []
    for step in range(1, 6001):
        now = step*.01
        command = sign*maximum*turn.speed_limit()
        commands.append(command)
        rate += max(-acceleration*.01, min(acceleration*.01, command-rate))
        filtered = .85*filtered+.15*rate
        physical_yaw += rate*.01
        if now+1e-9 >= next_sample:
            held, last_sample = filtered, now
            next_sample += .5
        if now+1e-9 >= next_odom:
            odom_yaw += held*.05
            assert turn.observe(math.atan2(math.sin(odom_yaw), math.cos(odom_yaw)),
                                round(now*1e9), held)
            next_odom += .05
        if abs(rate) < .02 and abs(held) < .02:
            stopped_since = now if stopped_since is None else stopped_since
        else:
            stopped_since = None
        if now+1e-9 < next_supervisor:
            continue
        next_supervisor += .2
        action = turn.tick(now, blocked=False, available=True,
                           feedback_age_sec=now-last_sample)
        assert action not in ('fail', 'cancel_failed')
        if action == 'cancel_endpoint' and turn.phase == 'running':
            turn.phase = 'stopping'
            turn.cancel_confirmed = True
        if turn.phase != 'running' and stopped_since is not None and now-stopped_since >= 1.:
            result = turn.stopped(now, True)
            assert result != 'fail'
            if result == 'complete':
                assert abs(math.degrees(turn.travel)-360.) <= 3.
                assert abs(sign*math.degrees(physical_yaw)-360.) <= 5.
                assert turn.active_elapsed < active_limit
                assert now < 35.
                assert turn.resumes <= 2
                assert all(sign*value >= 0. for value in commands)
                return
    pytest.fail('Low-rate feedback turn exceeded its existing bounded session')


def test_older_wheels_reserve_more_braking_angle_without_weakening_stop_gates():
    turns = [RecoverySpin(2*math.pi, 0., 0., 1) for _ in range(2)]
    for turn, age in zip(turns, (0., .5)):
        turn.tail_rate = .3
        assert turn.tick(.2, blocked=False, available=True, feedback_age_sec=age) == 'run'
    assert turns[1].braking_reserve > turns[0].braking_reserve
    assert turns[1].tick(.4, blocked=False, available=False) == 'cancel_pause'
    overshoot = RecoverySpin(2*math.pi, 0., 0., 1)
    overshoot.travel = math.radians(364.)
    assert overshoot.stopped(1., True) == 'fail'
    assert overshoot.reason == 'endpoint_overshoot'
