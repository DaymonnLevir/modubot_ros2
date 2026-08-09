"""Unit tests for trajectory experiment geometry and analysis."""

import csv
import math

import pytest

from modubot_serial_bridge.trajectory_experiment_core import (
    FeedforwardMap,
    OdometryState,
    build_trajectory_plan,
    desired_pose,
    integrate_differential_drive,
)


def test_desired_pose_for_straight_and_left_arc():
    assert desired_pose(1.5, 0.0) == pytest.approx((1.5, 0.0, 0.0))
    x_m, y_m, yaw = desired_pose(math.pi / 2.0, 1.0)
    assert (x_m, y_m, yaw) == pytest.approx((1.0, 1.0, math.pi / 2.0))


def test_differential_drive_integration():
    state = integrate_differential_drive(OdometryState(), 1.0, 1.0, 0.225)
    assert state.x_m == pytest.approx(1.0)
    assert state.y_m == pytest.approx(0.0)
    assert state.path_length_m == pytest.approx(1.0)


def test_randomized_plan_is_reproducible():
    arguments = dict(
        trajectories=['straight', 'arc_left', 'arc_right'],
        repetitions=3,
        straight_distance_m=1.5,
        arc_length_m=0.8,
        arc_radius_m=0.8,
        linear_speed_mps=0.25,
        wheel_separation_m=0.225,
        order='randomized',
        seed=42,
    )
    first = build_trajectory_plan(**arguments)
    second = build_trajectory_plan(**arguments)
    assert [run.run_id for run in first] == [run.run_id for run in second]
    assert len(first) == 9


def test_feedforward_map_interpolates_each_wheel(tmp_path):
    map_path = tmp_path / 'inverse_map_seed.csv'
    fields = [
        'direction',
        'wheel',
        'target_speed_magnitude_mps',
        'normalized_command',
    ]
    with map_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for wheel, offset in [('left', 0.0), ('right', 0.01)]:
            writer.writerow(
                {
                    'direction': 'forward',
                    'wheel': wheel,
                    'target_speed_magnitude_mps': 0.2,
                    'normalized_command': 0.1 + offset,
                }
            )
            writer.writerow(
                {
                    'direction': 'forward',
                    'wheel': wheel,
                    'target_speed_magnitude_mps': 0.4,
                    'normalized_command': 0.2 + offset,
                }
            )
    calibration = FeedforwardMap.from_csv(map_path)
    assert calibration.command_for_speed('left', 0.3) == pytest.approx(0.15)
    assert calibration.command_for_speed('right', 0.3) == pytest.approx(0.16)
    with pytest.raises(ValueError):
        calibration.command_for_speed('left', 0.5)
