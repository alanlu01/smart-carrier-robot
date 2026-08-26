import pytest

from smart_delivery_core.wheel_feedback import (
    WheelSampleAssembler,
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
