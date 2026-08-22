"""Unit tests for the PI step experiment plan and telemetry parser."""

import pytest

from modubot_serial_bridge.pi_step_experiment import (
    build_step_plan,
    overshoot_percent,
    parse_pi_telemetry,
)


def test_step_plan_is_balanced():
    plan = build_step_plan(
        speeds=[0.1, 0.2, 0.3],
        direction='both',
        repetitions=5,
        order='interleaved',
        seed=42,
    )
    assert len(plan) == 60
    assert sum(run.control_condition == 'baseline' for run in plan) == 30
    assert sum(run.control_condition == 'pi' for run in plan) == 30


def test_parse_pi_telemetry():
    values = parse_pi_telemetry(
        'T 1000 2.5 2.4 18.0 23 2.5 2.6 19.0 24'
    )
    assert values is not None
    assert values[0] == pytest.approx(1000.0)
    assert values[-1] == pytest.approx(24.0)
    assert parse_pi_telemetry('O 1 1 50') is None


def test_overshoot_respects_command_direction():
    assert overshoot_percent([-0.1, -0.22, -0.2], -0.2) == pytest.approx(10.0)
    assert overshoot_percent([0.3, -0.2], -0.2) == pytest.approx(0.0)
