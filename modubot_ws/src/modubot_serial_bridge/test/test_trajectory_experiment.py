"""Unit tests for trajectory experiment geometry and analysis."""

import argparse
import csv
import math
from unittest.mock import patch

import pytest

from modubot_serial_bridge.odometry_trajectory_experiment import (
    execute_with_emergency_retries,
    parse_trajectories,
)

from modubot_serial_bridge.trajectory_experiment_core import (
    FeedforwardMap,
    OdometryState,
    build_trajectory_plan,
    desired_pose,
    desired_pose_for_run,
    integrate_differential_drive,
    trajectory_segment_index,
    wheel_speed_segments,
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


def test_figure_eight_plan_is_continuous_and_switches_curvature():
    run = build_trajectory_plan(
        trajectories=['figure_eight'],
        repetitions=1,
        straight_distance_m=1.5,
        arc_length_m=0.8,
        arc_radius_m=0.8,
        linear_speed_mps=0.2,
        wheel_separation_m=0.207,
        order='interleaved',
        figure_eight_radius_m=0.35,
        figure_eight_cycles=2,
        figure_eight_start_direction='left',
    )[0]

    circle_length = 2.0 * math.pi * 0.35
    assert run.target_path_length_m == pytest.approx(4.0 * circle_length)
    assert run.target_yaw_rad == 0.0
    assert run.figure_eight_cycles == 2
    assert run.target_left_speed_mps < run.target_right_speed_mps
    assert wheel_speed_segments(run)[1] == pytest.approx(
        (run.target_right_speed_mps, run.target_left_speed_mps)
    )
    assert trajectory_segment_index(run, 0.0) == 0
    assert trajectory_segment_index(run, circle_length) == 1
    assert trajectory_segment_index(run, 2.0 * circle_length) == 2
    assert trajectory_segment_index(run, run.target_path_length_m) == 3


def test_figure_eight_ideal_pose_returns_to_crossing_after_each_circle():
    run = build_trajectory_plan(
        trajectories=['figure_eight'],
        repetitions=1,
        straight_distance_m=1.5,
        arc_length_m=0.8,
        arc_radius_m=0.8,
        linear_speed_mps=0.2,
        wheel_separation_m=0.207,
        order='interleaved',
        figure_eight_radius_m=0.35,
        figure_eight_cycles=1,
        figure_eight_start_direction='left',
    )[0]

    circle_length = 2.0 * math.pi * 0.35
    assert desired_pose_for_run(run, circle_length) == pytest.approx(
        (0.0, 0.0, 0.0)
    )
    assert desired_pose_for_run(run, run.target_path_length_m) == pytest.approx(
        (0.0, 0.0, 0.0)
    )


def test_figure_eight_name_is_accepted_by_command_line_parser():
    assert parse_trajectories('figure_eight') == ['figure_eight']
    assert 'figure_eight' in parse_trajectories('all')


def test_trajectory_emergency_stop_preserves_attempt_and_retries():
    run = build_trajectory_plan(
        trajectories=['figure_eight'],
        repetitions=1,
        straight_distance_m=1.5,
        arc_length_m=0.8,
        arc_radius_m=0.8,
        linear_speed_mps=0.2,
        wheel_separation_m=0.207,
        order='interleaved',
    )[0]

    class FakeRunner:
        def __init__(self):
            self.calls = []

        def execute(self, _run, _commands, attempt):
            self.calls.append(attempt)
            status = (
                'emergency_stop_by_operator'
                if attempt == 1
                else 'completed'
            )
            return {
                'run_id': _run.run_id if attempt == 1 else _run.run_id + '_A02',
                'status': status,
                'odom_final_path_m': 0.4,
                'odom_path_at_stop_m': 4.4,
                'elapsed_command_s': 25.0,
                'battery_mean_v': 24.2,
            }

    runner = FakeRunner()
    args = argparse.Namespace(countdown=0, automatic=False)
    attempts = {}
    with patch('builtins.input', return_value=''):
        summary, outcome = execute_with_emergency_retries(
            runner,
            args,
            run,
            [(1.0, 2.0), (2.0, 1.0)],
            attempts,
        )

    assert outcome == 'completed'
    assert summary['status'] == 'completed'
    assert runner.calls == [1, 2]
    assert attempts[run.run_id] == 2


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
