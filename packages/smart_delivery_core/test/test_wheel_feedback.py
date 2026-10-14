import pytest

from smart_delivery_core.wheel_feedback import (
    WheelSampleAssembler,
    WheelStopEvidence,
    parse_wheel_report,
)


def test_parse_wheel_report_ignores_banner_and_reads_speed():
    assert parse_wheel_report("--- robot ready ---") is None
    assert parse_wheel_report("B_PWM: 12, B_RT: -123.500000") == ("B", -123.5)


def test_assembler_commits_only_after_all_four_wheels_arrive():
    assembler = WheelSampleAssembler(batch_window_sec=0.1, started_at=0.0)

    assert assembler.add_line("A_RT: 1", 0.50) is None
    assert assembler.add_line("B_RT: 2", 0.51) is None
    assert assembler.add_line("C_RT: 3", 0.52) is None
    assert assembler.add_line("D_RT: 4", 0.53) == {
        "A": 1.0,
        "B": 2.0,
        "C": 3.0,
        "D": 4.0,
    }
    assert assembler.last_complete_at == pytest.approx(0.53)
    assert assembler.complete_sample_count == 1


def test_assembler_accepts_ros_delivery_spread_within_configured_window():
    assembler = WheelSampleAssembler(batch_window_sec=0.25, started_at=0.0)

    assert assembler.add_line("A_RT: 1", 0.50) is None
    assert assembler.add_line("B_RT: 2", 0.55) is None
    assert assembler.add_line("C_RT: 3", 0.60) is None
    assert assembler.add_line("D_RT: 4", 0.65) is not None


def test_expired_partial_batch_is_not_mixed_with_next_group():
    assembler = WheelSampleAssembler(batch_window_sec=0.1, started_at=0.0)

    assembler.add_line("A_RT: 100", 0.50)
    assembler.add_line("B_RT: 100", 0.51)
    assert assembler.add_line("C_RT: 3", 1.00) is None
    assert assembler.add_line("D_RT: 4", 1.01) is None
    assert assembler.add_line("A_RT: 1", 1.02) is None
    assert assembler.add_line("B_RT: 2", 1.03) == {
        "A": 1.0,
        "B": 2.0,
        "C": 3.0,
        "D": 4.0,
    }


def test_watchdog_uses_last_complete_group_and_recovers():
    assembler = WheelSampleAssembler(batch_window_sec=0.1, started_at=10.0)
    assert not assembler.is_stale(10.79, timeout_sec=0.8)
    assert assembler.is_stale(10.81, timeout_sec=0.8)

    for index, wheel in enumerate("ABCD"):
        assembler.add_line(f"{wheel}_RT: 0", 11.0 + index * 0.01)

    assert not assembler.is_stale(11.82, timeout_sec=0.8)
    assert assembler.is_stale(11.84, timeout_sec=0.8)


def zero_group(evidence, at):
    for letter in 'ABCD':
        evidence.observe(letter+'_RT: 0', at)


def test_three_independent_zero_groups_at_two_hz_prove_stop():
    evidence = WheelStopEvidence(0.)
    for at in (0., .5):
        zero_group(evidence, at)
        assert not evidence.ready(at)
    zero_group(evidence, 1.)
    assert evidence.ready(1.5)
    assert not evidence.ready(1.651)
    assert evidence.zero_groups == 0


def test_repeated_ticks_or_bursts_cannot_fabricate_one_second_stop():
    evidence = WheelStopEvidence(0.)
    zero_group(evidence, 0.)
    for at in (.1, .2, .3, .4, .5):
        assert not evidence.ready(at)
    for at in (.51, .52, .53):
        zero_group(evidence, at)
    assert not evidence.ready(.53)


@pytest.mark.parametrize('bad', ['A_RT: 10', 'D_RT: 1e999', 'B_RT: nan', 'C_RT: bad'])
def test_any_single_moving_or_invalid_wheel_resets_stop_proof(bad):
    evidence = WheelStopEvidence(0.)
    for at in (0., .5, 1.):
        zero_group(evidence, at)
    assert evidence.ready(1.)
    evidence.observe(bad, 1.01)
    assert not evidence.ready(1.01)


def test_feedback_gap_and_partial_groups_do_not_unlock():
    evidence = WheelStopEvidence(0.)
    for at in (0., .5):
        zero_group(evidence, at)
    zero_group(evidence, 1.2)
    assert evidence.zero_groups == 1
    for at in (1.3, 1.4, 1.5):
        for letter in 'ABC':
            evidence.observe(letter+'_RT: 0', at)
    assert not evidence.ready(1.6)
