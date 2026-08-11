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


def test_signed_angular_displacement_is_not_absolute_travel():
    state = OdometryState()
    integrate_differential_drive(state, -0.1, 0.1, 0.2)
    integrate_differential_drive(state, 0.02, -0.02, 0.2)
    assert state.angular_displacement_rad == pytest.approx(0.8)
    assert state.angular_travel_rad == pytest.approx(1.2)


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


def test_in_place_rotations_have_opposite_wheel_speeds():
    runs = build_trajectory_plan(
        trajectories=['rotation_left', 'rotation_right'],
        repetitions=1,
        straight_distance_m=1.5,
        arc_length_m=0.8,
        arc_radius_m=0.8,
        linear_speed_mps=0.2,
        wheel_separation_m=0.207,
        order='interleaved',
        rotation_angle_rad=math.pi / 2.0,
        angular_speed_rps=0.4,
    )
    left, right = runs
    assert left.target_path_length_m == 0.0
    assert left.target_yaw_rad == pytest.approx(math.pi / 2.0)
    assert left.target_left_speed_mps == pytest.approx(
        -left.target_right_speed_mps
    )
    assert right.target_yaw_rad == pytest.approx(-math.pi / 2.0)
    assert right.target_left_speed_mps == pytest.approx(
        -right.target_right_speed_mps
    )


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
