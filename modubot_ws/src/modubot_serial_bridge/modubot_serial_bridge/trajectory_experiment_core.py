"""Geometry and feedforward helpers for ModuBot trajectory experiments."""

from __future__ import annotations

import csv
import math
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence, Tuple


@dataclass(frozen=True)
class TrajectoryRun:
    """One repeatable trajectory in an experiment campaign."""

    run_id: str
    trajectory: str
    repetition: int
    target_path_length_m: float
    curvature_per_m: float
    target_linear_speed_mps: float
    target_left_speed_mps: float
    target_right_speed_mps: float


@dataclass
class OdometryState:
    """Differential-drive odometry state relative to the start of a run."""

    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0
    path_length_m: float = 0.0


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def desired_pose(
    progress_m: float, curvature_per_m: float
) -> Tuple[float, float, float]:
    """Return the ideal pose after a path-length progress value."""
    if abs(curvature_per_m) < 1e-12:
        return progress_m, 0.0, 0.0
    heading = curvature_per_m * progress_m
    x_m = math.sin(heading) / curvature_per_m
    y_m = (1.0 - math.cos(heading)) / curvature_per_m
    return x_m, y_m, heading


def integrate_differential_drive(
    state: OdometryState,
    left_distance_m: float,
    right_distance_m: float,
    wheel_separation_m: float,
) -> OdometryState:
    """Integrate one pair of wheel increments using midpoint integration."""
    center_distance = (left_distance_m + right_distance_m) / 2.0
    heading_change = (
        right_distance_m - left_distance_m
    ) / wheel_separation_m
    midpoint_heading = state.yaw_rad + heading_change / 2.0
    state.x_m += center_distance * math.cos(midpoint_heading)
    state.y_m += center_distance * math.sin(midpoint_heading)
    state.yaw_rad = wrap_angle(state.yaw_rad + heading_change)
    state.path_length_m += abs(center_distance)
    return state


def build_trajectory_plan(
    trajectories: Sequence[str],
    repetitions: int,
    straight_distance_m: float,
    arc_length_m: float,
    arc_radius_m: float,
    linear_speed_mps: float,
    wheel_separation_m: float,
    order: str = 'randomized',
    seed: int = 42,
) -> List[TrajectoryRun]:
    """Build a balanced plan of straight and constant-curvature runs."""
    plan: List[TrajectoryRun] = []
    labels = {
        'straight': 'STRAIGHT',
        'arc_left': 'ARC_LEFT',
        'arc_right': 'ARC_RIGHT',
    }
    for repetition in range(1, repetitions + 1):
        for trajectory in trajectories:
            if trajectory not in labels:
                raise ValueError(f'Unsupported trajectory: {trajectory}')
            if trajectory == 'straight':
                curvature = 0.0
                path_length = straight_distance_m
            else:
                sign = 1.0 if trajectory == 'arc_left' else -1.0
                curvature = sign / arc_radius_m
                path_length = arc_length_m
            left_speed = linear_speed_mps * (
                1.0 - curvature * wheel_separation_m / 2.0
            )
            right_speed = linear_speed_mps * (
                1.0 + curvature * wheel_separation_m / 2.0
            )
            plan.append(
                TrajectoryRun(
                    run_id=f'{labels[trajectory]}_R{repetition:02d}',
                    trajectory=trajectory,
                    repetition=repetition,
                    target_path_length_m=path_length,
                    curvature_per_m=curvature,
                    target_linear_speed_mps=linear_speed_mps,
                    target_left_speed_mps=left_speed,
                    target_right_speed_mps=right_speed,
                )
            )

    if order == 'randomized':
        random.Random(seed).shuffle(plan)
    elif order == 'blocked':
        plan.sort(key=lambda item: (item.trajectory, item.repetition))
    elif order != 'interleaved':
        raise ValueError(f'Unsupported plan order: {order}')
    return plan


class FeedforwardMap:
    """Piecewise-linear inverse wheel-speed map from calibration output."""

    def __init__(
        self,
        points: Dict[Tuple[str, str], List[Tuple[float, float]]],
        source: Path,
    ) -> None:
        self.points = points
        self.source = source

    @classmethod
    def from_csv(cls, path: Path) -> 'FeedforwardMap':
        points: Dict[Tuple[str, str], List[Tuple[float, float]]] = {}
        with path.open(newline='', encoding='utf-8-sig') as stream:
            reader = csv.DictReader(stream)
            required = {
                'direction',
                'wheel',
                'target_speed_magnitude_mps',
                'normalized_command',
            }
            if (
                reader.fieldnames is None
                or not required.issubset(reader.fieldnames)
            ):
                raise ValueError(
                    'Feedforward CSV must be an inverse_map_seed.csv file.'
                )
            for row in reader:
                direction = row['direction'].strip().lower()
                wheel = row['wheel'].strip().lower()
                speed = float(row['target_speed_magnitude_mps'])
                command = float(row['normalized_command'])
                points.setdefault((direction, wheel), []).append(
                    (speed, command)
                )
        for values in points.values():
            values.sort(key=lambda item: item[0])
        return cls(points=points, source=path)

    def command_for_speed(self, wheel: str, speed_mps: float) -> float:
        """Interpolate the calibrated command for a signed wheel speed."""
        if abs(speed_mps) < 1e-12:
            return 0.0
        direction = 'forward' if speed_mps > 0.0 else 'reverse'
        values = self.points.get((direction, wheel))
        if not values:
            raise ValueError(
                f'No {direction}/{wheel} values in {self.source}.'
            )
        magnitude = abs(speed_mps)
        minimum = values[0][0]
        maximum = values[-1][0]
        if magnitude < minimum or magnitude > maximum:
            raise ValueError(
                f'{wheel} target {magnitude:.4f} m/s is outside the '
                'calibrated '
                f'{direction} range [{minimum:.4f}, {maximum:.4f}] m/s.'
            )
        pairs = zip(values, values[1:])
        for (speed_a, command_a), (speed_b, command_b) in pairs:
            if speed_a <= magnitude <= speed_b:
                if abs(speed_b - speed_a) < 1e-12:
                    return command_a
                ratio = (magnitude - speed_a) / (speed_b - speed_a)
                return command_a + ratio * (command_b - command_a)
        return values[-1][1]


def commands_for_run(
    run: TrajectoryRun,
    control_mode: str,
    v_wheel_max_mps: float,
    feedforward_map: FeedforwardMap | None,
) -> Tuple[float, float]:
    """Convert desired wheel speeds to normalized actuator commands."""
    if control_mode == 'feedforward':
        if feedforward_map is None:
            raise ValueError('feedforward control requires --feedforward-map.')
        return (
            feedforward_map.command_for_speed(
                'left', run.target_left_speed_mps
            ),
            feedforward_map.command_for_speed(
                'right', run.target_right_speed_mps
            ),
        )
    if control_mode != 'raw':
        raise ValueError(f'Unsupported control mode: {control_mode}')
    return (
        run.target_left_speed_mps / v_wheel_max_mps,
        run.target_right_speed_mps / v_wheel_max_mps,
    )
