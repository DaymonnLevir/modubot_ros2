#!/usr/bin/env python3
"""Drive finite trajectories and stop from serial or ROS odometry."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from collections import deque
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Sequence, Tuple

from .feedforward_calibration import (
    CALIBRATION_EXCEPTIONS,
    EmergencyStopMonitor,
    Esp32Serial,
    firmware_abort_reason,
    norm_to_dac,
    parse_odom_line,
)
from .experiment_support import (
    SummaryTable,
    battery_statistics,
    csv_value,
    execution_run_id,
    parse_battery_line,
)
from .trajectory_experiment_core import (
    FeedforwardMap,
    OdometryState,
    TrajectoryRun,
    build_trajectory_plan,
    commands_for_run,
    desired_pose_for_run,
    integrate_differential_drive,
    trajectory_segment_index,
    wheel_speed_segments,
)


SCRIPT_VERSION = '1.3.0'

CommandPair = Tuple[float, float]
RunCommands = Dict[str, List[CommandPair]]


@dataclass(frozen=True)
class RosOdomPoint:
    """Minimal odometry sample used by the ROS simulation backend."""

    stamp_s: float
    received_monotonic_s: float
    x_m: float
    y_m: float
    yaw_rad: float


SAMPLE_FIELDS = [
    'timestamp_utc',
    'run_id',
    'planned_run_id',
    'attempt',
    'trajectory',
    'repetition',
    'phase',
    'trajectory_segment',
    'figure_eight_cycle',
    'elapsed_run_s',
    'elapsed_phase_s',
    'target_path_length_m',
    'target_yaw_rad',
    'target_curvature_per_m',
    'active_curvature_per_m',
    'target_linear_speed_mps',
    'target_left_speed_mps',
    'target_right_speed_mps',
    'active_target_left_speed_mps',
    'active_target_right_speed_mps',
    'normalized_left',
    'normalized_right',
    'command_type',
    'command_left',
    'command_right',
    'command_linear_mps',
    'command_angular_rps',
    'dac_left',
    'dac_right',
    'delta_ticks_left',
    'delta_ticks_right',
    'dt_ms',
    'left_distance_m',
    'right_distance_m',
    'left_speed_mps',
    'right_speed_mps',
    'body_linear_speed_mps',
    'body_angular_speed_rps',
    'odom_x_m',
    'odom_y_m',
    'odom_yaw_rad',
    'odom_path_length_m',
    'odom_angular_travel_rad',
    'odom_angular_displacement_rad',
    'desired_x_m',
    'desired_y_m',
    'desired_yaw_rad',
    'battery_voltage_v',
]

SUMMARY_FIELDS = [
    'run_id',
    'planned_run_id',
    'attempt',
    'status',
    'trajectory',
    'repetition',
    'control_mode',
    'backend',
    'target_path_length_m',
    'target_yaw_rad',
    'target_curvature_per_m',
    'target_linear_speed_mps',
    'target_left_speed_mps',
    'target_right_speed_mps',
    'figure_eight_radius_m',
    'figure_eight_cycles',
    'figure_eight_start_direction',
    'normalized_left',
    'normalized_right',
    'command_type',
    'command_left',
    'command_right',
    'command_linear_mps',
    'command_angular_rps',
    'command_start_utc',
    'command_stop_utc',
    'elapsed_command_s',
    'odom_path_at_stop_m',
    'odom_stop_overshoot_m',
    'odom_angle_at_stop_rad',
    'odom_angle_overshoot_rad',
    'odom_final_path_m',
    'odom_final_x_m',
    'odom_final_y_m',
    'odom_final_yaw_rad',
    'battery_samples',
    'battery_start_v',
    'battery_end_v',
    'battery_mean_v',
    'battery_min_v',
    'battery_max_v',
    'battery_drop_v',
    'sample_file',
]


def default_output_dir() -> Path:
    configured = os.environ.get('MODUBOT_TRAJECTORY_DIR')
    if configured:
        return Path(configured).expanduser()
    docker_workspace = Path('/workspace/modubot_ws')
    if docker_workspace.is_dir():
        return docker_workspace / 'trajectory_data'
    return Path.home() / 'modubot_trajectory_data'


def parse_trajectories(value: str) -> List[str]:
    """Parse one or more comma-separated trajectory names."""
    if value.strip().lower() == 'all':
        return [
            'straight',
            'rotation_left',
            'rotation_right',
            'arc_left',
            'arc_right',
            'figure_eight',
        ]
    allowed = {
        'straight',
        'rotation_left',
        'rotation_right',
        'arc_left',
        'arc_right',
        'figure_eight',
    }
    result: List[str] = []
    for token in value.split(','):
        name = token.strip().lower()
        if not name:
            continue
        if name not in allowed:
            raise argparse.ArgumentTypeError(
                'Trajectories must be straight, rotation_left, '
                'rotation_right, arc_left, arc_right, figure_eight, or all.'
            )
        if name not in result:
            result.append(name)
    if not result:
        raise argparse.ArgumentTypeError(
            'At least one trajectory is required.'
        )
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            'Drive straight, rotation, constant-curvature, or figure-eight '
            'paths and stop from integrated wheel odometry.'
        )
    )
    parser.add_argument(
        '--backend',
        choices=['serial', 'ros'],
        default='serial',
        help=(
            'serial controls the ESP32; ros publishes Twist and consumes '
            'nav_msgs/Odometry for simulation.'
        ),
    )
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--cmd-vel-topic', default='/cmd_vel_safe')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument(
        '--trajectories', type=parse_trajectories, default=['straight']
    )
    parser.add_argument('--repetitions', type=int, default=5)
    parser.add_argument('--distance', type=float, default=1.5)
    parser.add_argument('--arc-length', type=float, default=0.8)
    parser.add_argument('--arc-radius', type=float, default=0.8)
    parser.add_argument('--figure-eight-radius', type=float, default=0.35)
    parser.add_argument('--figure-eight-cycles', type=int, default=1)
    parser.add_argument(
        '--figure-eight-start-direction',
        choices=['left', 'right'],
        default='left',
    )
    parser.add_argument('--linear-speed', type=float, default=0.25)
    parser.add_argument('--rotation-angle-deg', type=float, default=90.0)
    parser.add_argument('--angular-speed', type=float, default=0.4)
    parser.add_argument(
        '--control-mode',
        choices=['raw', 'feedforward', 'pi'],
        default='pi',
        help='Physical ESP32 mode; ignored when --backend ros is selected.',
    )
    parser.add_argument('--feedforward-map', type=Path)
    parser.add_argument('--v-wheel-max', type=float, default=0.6)
    parser.add_argument('--max-normalized-command', type=float, default=0.30)
    parser.add_argument('--kp', type=float, default=12.0)
    parser.add_argument('--ki', type=float, default=40.0)
    parser.add_argument('--kd', type=float, default=0.0)
    parser.add_argument('--kff', type=float, default=0.0)
    parser.add_argument('--dac-min', type=float, default=0.0)
    parser.add_argument(
        '--order',
        choices=['randomized', 'interleaved', 'blocked'],
        default='randomized',
    )
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--pre-time', type=float, default=2.0)
    parser.add_argument('--post-time', type=float, default=2.0)
    parser.add_argument('--countdown', type=int, default=3)
    parser.add_argument('--send-rate', type=float, default=20.0)
    parser.add_argument('--odom-timeout', type=float, default=1.0)
    parser.add_argument('--max-duration', type=float, default=60.0)
    parser.add_argument('--preflight-timeout', type=float, default=3.0)
    parser.add_argument('--battery-timeout', type=float, default=3.0)
    parser.add_argument('--startup-wait', type=float, default=2.0)
    parser.add_argument('--ticks-left', type=float, default=91.0)
    parser.add_argument('--ticks-right', type=float, default=91.0)
    parser.add_argument('--wheel-radius-left', type=float, default=0.078)
    parser.add_argument('--wheel-radius-right', type=float, default=0.078)
    parser.add_argument('--wheel-separation', type=float, default=0.225)
    parser.add_argument('--arena-width', type=float, default=2.0)
    parser.add_argument('--arena-height', type=float, default=1.2)
    parser.add_argument('--surface', default='not_recorded')
    parser.add_argument('--battery-start-v', type=float)
    parser.add_argument('--video-file', default='not_recorded')
    parser.add_argument('--notes', default='')
    parser.add_argument(
        '--output-dir', type=Path, default=default_output_dir()
    )
    parser.add_argument('--campaign-name', default='odometry_trajectory')
    parser.add_argument(
        '--automatic',
        action='store_true',
        help='Run without operator confirmation between executions.',
    )
    parser.add_argument('--inter-run-wait', type=float, default=5.0)
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Validate and print the plan without opening the selected backend.',
    )
    return parser


def validate_args(args: argparse.Namespace) -> None:
    positive = {
        'baud': args.baud,
        'repetitions': args.repetitions,
        'distance': args.distance,
        'arc_length': args.arc_length,
        'arc_radius': args.arc_radius,
        'figure_eight_radius': args.figure_eight_radius,
        'figure_eight_cycles': args.figure_eight_cycles,
        'linear_speed': args.linear_speed,
        'rotation_angle_deg': args.rotation_angle_deg,
        'angular_speed': args.angular_speed,
        'v_wheel_max': args.v_wheel_max,
        'max_normalized_command': args.max_normalized_command,
        'send_rate': args.send_rate,
        'odom_timeout': args.odom_timeout,
        'max_duration': args.max_duration,
        'preflight_timeout': args.preflight_timeout,
        'battery_timeout': args.battery_timeout,
        'ticks_left': args.ticks_left,
        'ticks_right': args.ticks_right,
        'wheel_radius_left': args.wheel_radius_left,
        'wheel_radius_right': args.wheel_radius_right,
        'wheel_separation': args.wheel_separation,
        'arena_width': args.arena_width,
        'arena_height': args.arena_height,
    }
    for name, value in positive.items():
        if value <= 0:
            raise ValueError(f'{name} must be greater than zero.')
    if args.pre_time < 0 or args.post_time < 0 or args.startup_wait < 0:
        raise ValueError('Pre, post, and startup times cannot be negative.')
    if args.countdown < 0:
        raise ValueError('countdown cannot be negative.')
    if args.max_normalized_command > 1.0:
        raise ValueError('max_normalized_command cannot exceed 1.0.')
    if (
        args.backend == 'serial'
        and args.control_mode == 'feedforward'
        and args.feedforward_map is None
    ):
        raise ValueError('feedforward mode requires --feedforward-map.')
    if args.feedforward_map is not None and not args.feedforward_map.is_file():
        raise ValueError(f'Feedforward map not found: {args.feedforward_map}')
    if args.battery_start_v is not None and args.battery_start_v <= 0:
        raise ValueError('battery_start_v must be greater than zero.')
    if args.inter_run_wait < 0:
        raise ValueError('inter_run_wait cannot be negative.')


def prepare_plan(
    args: argparse.Namespace,
) -> Tuple[List[TrajectoryRun], RunCommands]:
    calibration = (
        FeedforwardMap.from_csv(args.feedforward_map)
        if args.feedforward_map is not None
        else None
    )
    plan = build_trajectory_plan(
        trajectories=args.trajectories,
        repetitions=args.repetitions,
        straight_distance_m=args.distance,
        arc_length_m=args.arc_length,
        arc_radius_m=args.arc_radius,
        linear_speed_mps=args.linear_speed,
        wheel_separation_m=args.wheel_separation,
        order=args.order,
        seed=args.seed,
        rotation_angle_rad=math.radians(args.rotation_angle_deg),
        angular_speed_rps=args.angular_speed,
        figure_eight_radius_m=args.figure_eight_radius,
        figure_eight_cycles=args.figure_eight_cycles,
        figure_eight_start_direction=args.figure_eight_start_direction,
    )
    commands: RunCommands = {}
    for run in plan:
        pairs: List[CommandPair] = []
        for left_speed, right_speed in wheel_speed_segments(run):
            if args.backend == 'ros':
                pair = (
                    (left_speed + right_speed) / 2.0,
                    (right_speed - left_speed) / args.wheel_separation,
                )
            elif args.control_mode == 'pi':
                pair = (
                    left_speed / args.wheel_radius_left,
                    right_speed / args.wheel_radius_right,
                )
            else:
                segment_run = TrajectoryRun(
                    **{
                        **asdict(run),
                        'target_left_speed_mps': left_speed,
                        'target_right_speed_mps': right_speed,
                    }
                )
                pair = commands_for_run(
                    segment_run,
                    args.control_mode,
                    args.v_wheel_max,
                    calibration,
                )
            if (
                args.backend == 'serial'
                and args.control_mode != 'pi'
                and max(abs(pair[0]), abs(pair[1]))
                > args.max_normalized_command
            ):
                raise ValueError(
                    f'{run.run_id} requires commands {pair}, exceeding '
                    '--max-normalized-command='
                    f'{args.max_normalized_command:.3f}.'
                )
            pairs.append(pair)
        commands[run.run_id] = pairs
    return plan, commands


def print_plan(
    plan: Sequence[TrajectoryRun],
    commands: RunCommands,
    args: argparse.Namespace,
) -> None:
    print('\nPlanned odometry-stopped runs:')
    for index, run in enumerate(plan, start=1):
        left, right = commands[run.run_id][0]
        if args.backend == 'ros':
            command_text = f'v={left:+.4f} m/s, w={right:+.4f} rad/s'
        else:
            command_label = 'omega' if args.control_mode == 'pi' else 'u'
            command_text = (
                f'{command_label}L={left:+.4f}, '
                f'{command_label}R={right:+.4f}'
            )
        print(
            f'  {index:02d}/{len(plan):02d} {run.run_id}: '
            f's={run.target_path_length_m:.3f} m, '
            f'yaw={math.degrees(run.target_yaw_rad):+.1f} deg, '
            f'k={run.curvature_per_m:+.3f} 1/m, '
            f'vL={run.target_left_speed_mps:.3f} m/s, '
            f'vR={run.target_right_speed_mps:.3f} m/s, '
            f'{command_text}'
        )
        if run.trajectory == 'figure_eight':
            second_left, second_right = commands[run.run_id][1]
            if args.backend == 'ros':
                segment_text = (
                    f'segment-2 v={second_left:+.4f} m/s, '
                    f'w={second_right:+.4f} rad/s'
                )
            else:
                segment_text = (
                    f'segment-2 {command_label}L={second_left:+.4f}, '
                    f'{command_label}R={second_right:+.4f}'
                )
            print(
                f'      figure-eight: radius={run.figure_eight_radius_m:.3f} m, '
                f'cycles={run.figure_eight_cycles}, '
                f'start={run.figure_eight_start_direction}, {segment_text}'
            )
    print(
        f'  order={args.order}, seed={args.seed}, '
        f'backend={args.backend}, control={effective_control_mode(args)}, '
        f'video={args.video_file}'
    )


def effective_control_mode(args: argparse.Namespace) -> str:
    """Return the control label that accurately describes the backend."""
    return 'simulated_velocity' if args.backend == 'ros' else args.control_mode


def command_type(args: argparse.Namespace) -> str:
    """Return the command semantics stored in metadata and CSV files."""
    if args.backend == 'ros':
        return 'cmd_vel_linear_angular'
    if args.control_mode == 'pi':
        return 'wheel_angular_speed_rps'
    return 'normalized_dac'


def create_campaign_dir(args: argparse.Namespace) -> Path:
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    path = args.output_dir.expanduser() / f'{args.campaign_name}_{timestamp}'
    path.mkdir(parents=True, exist_ok=False)
    (path / 'samples').mkdir()
    return path


def write_metadata(
    output_dir: Path,
    args: argparse.Namespace,
    plan: Sequence[TrajectoryRun],
    commands: RunCommands,
) -> None:
    if args.backend == 'ros':
        method = (
            'The process publishes geometry_msgs/Twist and stops each run '
            'from nav_msgs/Odometry. This simulation backend validates '
            'trajectory geometry and data collection; it does not emulate '
            'the ESP32 PI loop, motor dead zone, battery, or physical casters.'
        )
        battery_protocol = 'not_available_in_simulation'
    else:
        method = (
            'The ESP32 serial link is owned exclusively by this process. '
            'Integrated wheel increments stop each run at the requested '
            'odometric center-path length. AprilTag video is recorded '
            'independently and measured offline.'
        )
        battery_protocol = 'BAT:<voltage_v>'
    metadata = {
        'script_version': SCRIPT_VERSION,
        'created_at_utc': datetime.now(timezone.utc).isoformat(),
        'method': method,
        'odometry_definition': (
            'center increment=(left_distance+right_distance)/2; stop progress '
            'is accumulated center distance for straight/arcs and accumulated '
            'signed yaw in the commanded direction for in-place rotations.'
        ),
        'battery_protocol': battery_protocol,
        'arguments': {
            key: str(value) if isinstance(value, Path) else value
            for key, value in vars(args).items()
        },
        'plan': [
            {
                **asdict(run),
                'command_type': command_type(args),
                'command_left': (
                    None
                    if args.backend == 'ros'
                    else commands[run.run_id][0][0]
                ),
                'command_right': (
                    None
                    if args.backend == 'ros'
                    else commands[run.run_id][0][1]
                ),
                'command_linear_mps': (
                    commands[run.run_id][0][0]
                    if args.backend == 'ros'
                    else None
                ),
                'command_angular_rps': (
                    commands[run.run_id][0][1]
                    if args.backend == 'ros'
                    else None
                ),
                'command_segments': [
                    (
                        {'linear_x_mps': pair[0], 'angular_z_rps': pair[1]}
                        if args.backend == 'ros'
                        else {'left': pair[0], 'right': pair[1]}
                    )
                    for pair in commands[run.run_id]
                ],
            }
            for run in plan
        ],
    }
    (output_dir / 'metadata.json').write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False), encoding='utf-8'
    )


class ExperimentRunner:
    """Execute finite trajectories as sole owner of the ESP32 serial port."""

    def __init__(self, args: argparse.Namespace, output_dir: Path) -> None:
        self.args = args
        self.output_dir = output_dir
        self.summary_table = SummaryTable(
            output_dir / 'summary.csv', SUMMARY_FIELDS
        )
        self.link = Esp32Serial(args.port, args.baud, args.startup_wait)
        print('Waiting for ESP32 odometry telemetry...')
        try:
            first = self.link.wait_for_telemetry(args.preflight_timeout)
            first_battery = self.link.wait_for_battery(
                args.battery_timeout
            )
        except RuntimeError:
            self.link.close()
            raise
        print(f'ESP32 telemetry confirmed: O {first[0]} {first[1]} {first[2]}')
        print(f'Battery telemetry confirmed: {first_battery:.2f} V')
        if args.control_mode == 'pi':
            self.link.write_line(f'K {args.kp} {args.ki} {args.kd}')
            self.link.write_line(f'F {args.kff} {args.dac_min}')
            self.link.write_line('M 1')
        else:
            self.link.write_line('M 0')
        self.link.stop()

    def close(self) -> None:
        self.link.close()

    def _sample_row(
        self,
        run: TrajectoryRun,
        current_run_id: str,
        attempt: int,
        phase: str,
        elapsed_run_s: float,
        elapsed_phase_s: float,
        command: CommandPair,
        trajectory_segment: int,
        telemetry: Tuple[int, int, int],
        state: OdometryState,
        battery_voltage: Optional[float],
    ) -> Dict[str, object]:
        delta_left, delta_right, dt_ms = telemetry
        left_distance = (
            2.0 * math.pi * self.args.wheel_radius_left * delta_left
            / self.args.ticks_left
        )
        right_distance = (
            2.0 * math.pi * self.args.wheel_radius_right * delta_right
            / self.args.ticks_right
        )
        dt_s = dt_ms / 1000.0
        left_speed = left_distance / dt_s
        right_speed = right_distance / dt_s
        if phase in {'command', 'post_stop'}:
            integrate_differential_drive(
                state,
                left_distance,
                right_distance,
                self.args.wheel_separation,
            )
        if run.trajectory.startswith('rotation_'):
            desired_x = 0.0
            desired_y = 0.0
            direction = 1.0 if run.target_yaw_rad >= 0.0 else -1.0
            directed_progress = max(
                0.0, direction * state.angular_displacement_rad
            )
            desired_yaw = direction * min(
                directed_progress, abs(run.target_yaw_rad)
            )
        else:
            desired_x, desired_y, desired_yaw = desired_pose_for_run(
                run,
                state.path_length_m,
            )
        speed_segments = wheel_speed_segments(run)
        speed_segment = (
            trajectory_segment % 2
            if run.trajectory == 'figure_eight'
            else 0
        )
        active_left_speed, active_right_speed = speed_segments[speed_segment]
        if run.trajectory == 'figure_eight':
            initial_sign = 1.0 if run.curvature_per_m >= 0.0 else -1.0
            active_sign = (
                initial_sign
                if trajectory_segment % 2 == 0
                else -initial_sign
            )
            active_curvature = active_sign / run.figure_eight_radius_m
            figure_eight_cycle: Optional[int] = trajectory_segment // 2 + 1
        else:
            active_curvature = run.curvature_per_m
            figure_eight_cycle = None
        is_pi = self.args.control_mode == 'pi'
        return {
            'timestamp_utc': datetime.now(timezone.utc).isoformat(),
            'run_id': current_run_id,
            'planned_run_id': run.run_id,
            'attempt': attempt,
            'trajectory': run.trajectory,
            'repetition': run.repetition,
            'phase': phase,
            'trajectory_segment': trajectory_segment + 1,
            'figure_eight_cycle': figure_eight_cycle,
            'elapsed_run_s': elapsed_run_s,
            'elapsed_phase_s': elapsed_phase_s,
            'target_path_length_m': run.target_path_length_m,
            'target_yaw_rad': run.target_yaw_rad,
            'target_curvature_per_m': run.curvature_per_m,
            'active_curvature_per_m': active_curvature,
            'target_linear_speed_mps': run.target_linear_speed_mps,
            'target_left_speed_mps': run.target_left_speed_mps,
            'target_right_speed_mps': run.target_right_speed_mps,
            'active_target_left_speed_mps': active_left_speed,
            'active_target_right_speed_mps': active_right_speed,
            'normalized_left': None if is_pi else command[0],
            'normalized_right': None if is_pi else command[1],
            'command_type': (
                'wheel_angular_speed_rps' if is_pi else 'normalized_dac'
            ),
            'command_left': command[0],
            'command_right': command[1],
            'command_linear_mps': None,
            'command_angular_rps': None,
            'dac_left': None if is_pi else norm_to_dac(command[0]),
            'dac_right': None if is_pi else norm_to_dac(command[1]),
            'delta_ticks_left': delta_left,
            'delta_ticks_right': delta_right,
            'dt_ms': dt_ms,
            'left_distance_m': left_distance,
            'right_distance_m': right_distance,
            'left_speed_mps': left_speed,
            'right_speed_mps': right_speed,
            'body_linear_speed_mps': (left_speed + right_speed) / 2.0,
            'body_angular_speed_rps': (
                right_speed - left_speed
            ) / self.args.wheel_separation,
            'odom_x_m': state.x_m,
            'odom_y_m': state.y_m,
            'odom_yaw_rad': state.yaw_rad,
            'odom_path_length_m': state.path_length_m,
            'odom_angular_travel_rad': state.angular_travel_rad,
            'odom_angular_displacement_rad': (
                state.angular_displacement_rad
            ),
            'desired_x_m': desired_x,
            'desired_y_m': desired_y,
            'desired_yaw_rad': desired_yaw,
            'battery_voltage_v': battery_voltage,
        }

    def execute(
        self,
        run: TrajectoryRun,
        commands: Sequence[CommandPair],
        attempt: int = 1,
    ) -> Dict[str, object]:
        current_run_id = execution_run_id(run.run_id, attempt)
        sample_path = self.output_dir / 'samples' / f'{current_run_id}.csv'
        self.link.stop()
        self.link.clear_input()
        state = OdometryState()
        run_start = time.monotonic()
        last_odom: Optional[float] = None
        stop_progress = 0.0
        stop_angle = 0.0
        command_elapsed = 0.0
        command_start_utc: Optional[str] = None
        command_stop_utc: Optional[str] = None
        battery_values: List[float] = []
        latest_battery: Optional[float] = None
        emergency_stopped = False
        active_command = commands[0]
        active_segment = 0

        with EmergencyStopMonitor() as emergency, sample_path.open(
            'w', newline='', encoding='utf-8'
        ) as stream:
            if emergency.enabled:
                print('EMERGENCY STOP armed: press SPACE or E.')
            else:
                print(
                    'WARNING: keyboard emergency stop is unavailable; '
                    'keep the physical emergency stop accessible.'
                )
            writer = csv.DictWriter(stream, fieldnames=SAMPLE_FIELDS)
            writer.writeheader()
            try:
                for phase in ['pre_stop', 'command', 'post_stop']:
                    phase_start = time.monotonic()
                    next_send = phase_start
                    reached_target = False
                    if phase == 'command':
                        state = OdometryState()
                        command_start_utc = datetime.now(
                            timezone.utc
                        ).isoformat()
                    else:
                        self.link.stop()

                    while True:
                        now = time.monotonic()
                        elapsed_phase = now - phase_start
                        if emergency.poll():
                            self.link.stop(repeats=1)
                            emergency_stopped = True
                            stop_progress = state.path_length_m
                            stop_angle = state.angular_displacement_rad
                            command_elapsed = elapsed_phase
                            command_stop_utc = datetime.now(
                                timezone.utc
                            ).isoformat()
                            break
                        if (
                            phase == 'pre_stop'
                            and elapsed_phase >= self.args.pre_time
                        ):
                            break
                        if (
                            phase == 'post_stop'
                            and elapsed_phase >= self.args.post_time
                        ):
                            break
                        if phase == 'command':
                            command_elapsed = elapsed_phase
                            if elapsed_phase >= self.args.max_duration:
                                raise RuntimeError(
                                    f'{run.run_id} exceeded --max-duration '
                                    'before '
                                    'reaching the odometric target.'
                                )

                        if now >= next_send:
                            if phase == 'command':
                                active_segment = trajectory_segment_index(
                                    run, state.path_length_m
                                )
                                command_index = (
                                    active_segment % 2
                                    if run.trajectory == 'figure_eight'
                                    else 0
                                )
                                active_command = commands[command_index]
                                if self.args.control_mode == 'pi':
                                    self.link.write_line(
                                        f'W {active_command[0]:.6f} '
                                        f'{active_command[1]:.6f}'
                                    )
                                else:
                                    self.link.command(
                                        active_command[0], active_command[1]
                                    )
                            else:
                                self.link.write_line('S')
                            next_send = now + 1.0 / self.args.send_rate

                        for line in self.link.read_lines():
                            abort_reason = firmware_abort_reason(line)
                            if abort_reason is not None:
                                raise RuntimeError(
                                    f'ESP32 reported "{abort_reason}". '
                                    'The current run is invalid and the robot '
                                    'was stopped.'
                                )
                            battery_voltage = parse_battery_line(line)
                            if battery_voltage is not None:
                                latest_battery = battery_voltage
                                battery_values.append(battery_voltage)
                                continue
                            telemetry = parse_odom_line(line)
                            if telemetry is None:
                                continue
                            last_odom = time.monotonic()
                            active_command = (
                                active_command
                                if phase == 'command'
                                else (0.0, 0.0)
                            )
                            row = self._sample_row(
                                run,
                                current_run_id,
                                attempt,
                                phase,
                                now - run_start,
                                elapsed_phase,
                                active_command,
                                active_segment,
                                telemetry,
                                state,
                                latest_battery,
                            )
                            writer.writerow(
                                {
                                    key: csv_value(value)
                                    for key, value in row.items()
                                }
                            )
                            progress = (
                                (
                                    1.0
                                    if run.target_yaw_rad >= 0.0
                                    else -1.0
                                ) * state.angular_displacement_rad
                                if run.trajectory.startswith('rotation_')
                                else state.path_length_m
                            )
                            target_progress = (
                                abs(run.target_yaw_rad)
                                if run.trajectory.startswith('rotation_')
                                else run.target_path_length_m
                            )
                            if phase == 'command' and progress >= target_progress:
                                stop_progress = state.path_length_m
                                stop_angle = state.angular_displacement_rad
                                command_elapsed = elapsed_phase
                                command_stop_utc = datetime.now(
                                    timezone.utc
                                ).isoformat()
                                self.link.stop()
                                reached_target = True
                                break

                        if reached_target:
                            break

                        if (
                            phase == 'command'
                            and elapsed_phase > self.args.odom_timeout
                            and (
                                last_odom is None
                                or now - last_odom > self.args.odom_timeout
                            )
                        ):
                            raise RuntimeError(
                                'ESP32 odometry became stale during motion; '
                                'the robot was stopped.'
                            )
                        time.sleep(0.005)
                    if emergency_stopped:
                        break
            finally:
                self.link.stop()
                stream.flush()

        summary: Dict[str, object] = {
            'run_id': current_run_id,
            'planned_run_id': run.run_id,
            'attempt': attempt,
            'status': (
                'emergency_stop_by_operator'
                if emergency_stopped
                else 'completed'
            ),
            'trajectory': run.trajectory,
            'repetition': run.repetition,
            'control_mode': self.args.control_mode,
            'backend': 'serial',
            'target_path_length_m': run.target_path_length_m,
            'target_yaw_rad': run.target_yaw_rad,
            'target_curvature_per_m': run.curvature_per_m,
            'target_linear_speed_mps': run.target_linear_speed_mps,
            'target_left_speed_mps': run.target_left_speed_mps,
            'target_right_speed_mps': run.target_right_speed_mps,
            'figure_eight_radius_m': run.figure_eight_radius_m,
            'figure_eight_cycles': run.figure_eight_cycles,
            'figure_eight_start_direction': (
                run.figure_eight_start_direction
            ),
            'normalized_left': (
                None if self.args.control_mode == 'pi' else commands[0][0]
            ),
            'normalized_right': (
                None if self.args.control_mode == 'pi' else commands[0][1]
            ),
            'command_type': (
                'wheel_angular_speed_rps'
                if self.args.control_mode == 'pi'
                else 'normalized_dac'
            ),
            'command_left': commands[0][0],
            'command_right': commands[0][1],
            'command_linear_mps': None,
            'command_angular_rps': None,
            'command_start_utc': command_start_utc,
            'command_stop_utc': command_stop_utc,
            'elapsed_command_s': command_elapsed,
            'odom_path_at_stop_m': stop_progress,
            'odom_stop_overshoot_m': stop_progress - run.target_path_length_m,
            'odom_angle_at_stop_rad': stop_angle,
            'odom_angle_overshoot_rad': (
                abs(stop_angle) - abs(run.target_yaw_rad)
                if run.trajectory.startswith('rotation_')
                else None
            ),
            'odom_final_path_m': state.path_length_m,
            'odom_final_x_m': state.x_m,
            'odom_final_y_m': state.y_m,
            'odom_final_yaw_rad': state.yaw_rad,
            **battery_statistics(battery_values),
            'sample_file': str(sample_path.relative_to(self.output_dir)),
        }
        self.summary_table.append(summary)
        return summary

    def supersede(self, execution_id: str) -> None:
        if not self.summary_table.supersede(execution_id):
            raise RuntimeError(
                f'Could not mark {execution_id} as repeated.'
            )

    def record_skipped(
        self, run: TrajectoryRun, commands: Sequence[CommandPair]
    ) -> None:
        is_pi = self.args.control_mode == 'pi'
        row = {field: None for field in SUMMARY_FIELDS}
        row.update(
            {
                'run_id': run.run_id,
                'planned_run_id': run.run_id,
                'attempt': 0,
                'status': 'skipped_by_operator',
                'trajectory': run.trajectory,
                'repetition': run.repetition,
                'control_mode': self.args.control_mode,
                'backend': 'serial',
                'target_path_length_m': run.target_path_length_m,
                'target_yaw_rad': run.target_yaw_rad,
                'target_curvature_per_m': run.curvature_per_m,
                'target_linear_speed_mps': run.target_linear_speed_mps,
                'target_left_speed_mps': run.target_left_speed_mps,
                'target_right_speed_mps': run.target_right_speed_mps,
                'figure_eight_radius_m': run.figure_eight_radius_m,
                'figure_eight_cycles': run.figure_eight_cycles,
                'figure_eight_start_direction': (
                    run.figure_eight_start_direction
                ),
                'normalized_left': None if is_pi else commands[0][0],
                'normalized_right': None if is_pi else commands[0][1],
                'command_type': (
                    'wheel_angular_speed_rps'
                    if is_pi
                    else 'normalized_dac'
                ),
                'command_left': commands[0][0],
                'command_right': commands[0][1],
                'command_linear_mps': None,
                'command_angular_rps': None,
            }
        )
        self.summary_table.append(row)


class RosExperimentRunner(ExperimentRunner):
    """Execute trajectories through Twist commands and ROS odometry."""

    def __init__(self, args: argparse.Namespace, output_dir: Path) -> None:
        self.args = args
        self.output_dir = output_dir
        self.summary_table = SummaryTable(
            output_dir / 'summary.csv', SUMMARY_FIELDS
        )
        try:
            import rclpy
            from geometry_msgs.msg import Twist
            from nav_msgs.msg import Odometry
        except ImportError as exc:
            raise RuntimeError(
                'The ROS backend requires rclpy, geometry_msgs, and nav_msgs.'
            ) from exc

        self.rclpy = rclpy
        self.twist_type = Twist
        self.odom_queue: Deque[RosOdomPoint] = deque(maxlen=10000)
        self.latest_odom: Optional[RosOdomPoint] = None
        self._owns_rclpy_context = not rclpy.ok()
        if self._owns_rclpy_context:
            rclpy.init(args=None)
        self.node = rclpy.create_node('modubot_trajectory_experiment')
        self.publisher = self.node.create_publisher(
            Twist, args.cmd_vel_topic, 10
        )
        self.subscription = self.node.create_subscription(
            Odometry, args.odom_topic, self._on_odom, 50
        )

        print(
            f'Waiting for ROS odometry on {args.odom_topic} and a command '
            f'subscriber on {args.cmd_vel_topic}...'
        )
        deadline = time.monotonic() + args.preflight_timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if (
                self.latest_odom is not None
                and self.publisher.get_subscription_count() > 0
            ):
                break
        else:
            odom_status = (
                'received' if self.latest_odom is not None else 'missing'
            )
            subscriber_count = self.publisher.get_subscription_count()
            self.close()
            raise RuntimeError(
                'ROS backend preflight timed out: '
                f'odometry={odom_status}, cmd_vel subscribers='
                f'{subscriber_count}. Start Gazebo and verify the topics.'
            )
        print(
            f'ROS backend ready: {args.odom_topic} -> experiment -> '
            f'{args.cmd_vel_topic}. Battery and embedded PI are not simulated.'
        )
        self.stop()

    @staticmethod
    def _yaw_from_quaternion(quaternion: Any) -> float:
        siny_cosp = 2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        )
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _angle_delta(current: float, previous: float) -> float:
        return math.atan2(
            math.sin(current - previous),
            math.cos(current - previous),
        )

    def _on_odom(self, message: Any) -> None:
        stamp = message.header.stamp
        point = RosOdomPoint(
            stamp_s=float(stamp.sec) + float(stamp.nanosec) * 1.0e-9,
            received_monotonic_s=time.monotonic(),
            x_m=float(message.pose.pose.position.x),
            y_m=float(message.pose.pose.position.y),
            yaw_rad=self._yaw_from_quaternion(
                message.pose.pose.orientation
            ),
        )
        self.latest_odom = point
        self.odom_queue.append(point)

    def publish(self, command: CommandPair) -> None:
        message = self.twist_type()
        message.linear.x = float(command[0])
        message.angular.z = float(command[1])
        self.publisher.publish(message)

    def stop(self, repeats: int = 3) -> None:
        for _ in range(repeats):
            self.publish((0.0, 0.0))
            self.rclpy.spin_once(self.node, timeout_sec=0.01)

    def close(self) -> None:
        node = getattr(self, 'node', None)
        if node is None:
            return
        try:
            self.stop(repeats=5)
        finally:
            node.destroy_node()
            self.node = None
            if self._owns_rclpy_context and self.rclpy.ok():
                self.rclpy.shutdown()

    def _sample_from_odom(
        self,
        run: TrajectoryRun,
        current_run_id: str,
        attempt: int,
        phase: str,
        elapsed_run_s: float,
        elapsed_phase_s: float,
        command: CommandPair,
        trajectory_segment: int,
        previous: RosOdomPoint,
        current: RosOdomPoint,
        state: OdometryState,
    ) -> Dict[str, object]:
        dt_s = current.stamp_s - previous.stamp_s
        if dt_s <= 0.0:
            raise ValueError('ROS odometry timestamp did not advance.')
        delta_yaw = self._angle_delta(current.yaw_rad, previous.yaw_rad)
        midpoint_yaw = previous.yaw_rad + delta_yaw / 2.0
        delta_x = current.x_m - previous.x_m
        delta_y = current.y_m - previous.y_m
        center_distance = (
            delta_x * math.cos(midpoint_yaw)
            + delta_y * math.sin(midpoint_yaw)
        )
        left_distance = (
            center_distance
            - delta_yaw * self.args.wheel_separation / 2.0
        )
        right_distance = (
            center_distance
            + delta_yaw * self.args.wheel_separation / 2.0
        )
        synthetic_ticks_left = (
            left_distance
            * self.args.ticks_left
            / (2.0 * math.pi * self.args.wheel_radius_left)
        )
        synthetic_ticks_right = (
            right_distance
            * self.args.ticks_right
            / (2.0 * math.pi * self.args.wheel_radius_right)
        )
        row = super()._sample_row(
            run,
            current_run_id,
            attempt,
            phase,
            elapsed_run_s,
            elapsed_phase_s,
            command,
            trajectory_segment,
            (
                synthetic_ticks_left,
                synthetic_ticks_right,
                dt_s * 1000.0,
            ),
            state,
            None,
        )
        row.update(
            {
                'normalized_left': None,
                'normalized_right': None,
                'command_type': 'cmd_vel_linear_angular',
                'command_left': None,
                'command_right': None,
                'command_linear_mps': command[0],
                'command_angular_rps': command[1],
                'dac_left': None,
                'dac_right': None,
                'delta_ticks_left': None,
                'delta_ticks_right': None,
            }
        )
        return row

    def execute(
        self,
        run: TrajectoryRun,
        commands: Sequence[CommandPair],
        attempt: int = 1,
    ) -> Dict[str, object]:
        current_run_id = execution_run_id(run.run_id, attempt)
        sample_path = self.output_dir / 'samples' / f'{current_run_id}.csv'
        self.stop()
        self.odom_queue.clear()
        state = OdometryState()
        run_start = time.monotonic()
        previous_odom = self.latest_odom
        last_odom = (
            previous_odom.received_monotonic_s
            if previous_odom is not None
            else None
        )
        stop_progress = 0.0
        stop_angle = 0.0
        command_elapsed = 0.0
        command_start_utc: Optional[str] = None
        command_stop_utc: Optional[str] = None
        emergency_stopped = False
        active_command = commands[0]
        active_segment = 0

        with EmergencyStopMonitor() as emergency, sample_path.open(
            'w', newline='', encoding='utf-8'
        ) as stream:
            if emergency.enabled:
                print('SIMULATION STOP armed: press SPACE or E.')
            writer = csv.DictWriter(stream, fieldnames=SAMPLE_FIELDS)
            writer.writeheader()
            try:
                for phase in ['pre_stop', 'command', 'post_stop']:
                    phase_start = time.monotonic()
                    next_send = phase_start
                    reached_target = False
                    if phase == 'command':
                        state = OdometryState()
                        active_segment = 0
                        self.odom_queue.clear()
                        previous_odom = self.latest_odom
                        command_start_utc = datetime.now(
                            timezone.utc
                        ).isoformat()
                    else:
                        self.stop()

                    while True:
                        now = time.monotonic()
                        elapsed_phase = now - phase_start
                        if emergency.poll():
                            self.stop(repeats=1)
                            emergency_stopped = True
                            stop_progress = state.path_length_m
                            stop_angle = state.angular_displacement_rad
                            command_elapsed = elapsed_phase
                            command_stop_utc = datetime.now(
                                timezone.utc
                            ).isoformat()
                            break
                        if (
                            phase == 'pre_stop'
                            and elapsed_phase >= self.args.pre_time
                        ):
                            break
                        if (
                            phase == 'post_stop'
                            and elapsed_phase >= self.args.post_time
                        ):
                            break
                        if phase == 'command':
                            command_elapsed = elapsed_phase
                            if elapsed_phase >= self.args.max_duration:
                                raise RuntimeError(
                                    f'{run.run_id} exceeded --max-duration '
                                    'before reaching the ROS odometry target.'
                                )

                        if now >= next_send:
                            if phase == 'command':
                                active_segment = trajectory_segment_index(
                                    run, state.path_length_m
                                )
                                command_index = (
                                    active_segment % 2
                                    if run.trajectory == 'figure_eight'
                                    else 0
                                )
                                active_command = commands[command_index]
                                self.publish(active_command)
                            else:
                                self.publish((0.0, 0.0))
                            next_send = now + 1.0 / self.args.send_rate

                        self.rclpy.spin_once(self.node, timeout_sec=0.005)
                        while self.odom_queue:
                            current_odom = self.odom_queue.popleft()
                            last_odom = current_odom.received_monotonic_s
                            if previous_odom is None:
                                previous_odom = current_odom
                                continue
                            if current_odom.stamp_s <= previous_odom.stamp_s:
                                previous_odom = current_odom
                                continue
                            row_command = (
                                active_command
                                if phase == 'command'
                                else (0.0, 0.0)
                            )
                            row = self._sample_from_odom(
                                run,
                                current_run_id,
                                attempt,
                                phase,
                                now - run_start,
                                elapsed_phase,
                                row_command,
                                active_segment,
                                previous_odom,
                                current_odom,
                                state,
                            )
                            previous_odom = current_odom
                            writer.writerow(
                                {
                                    key: csv_value(value)
                                    for key, value in row.items()
                                }
                            )
                            progress = (
                                (
                                    1.0
                                    if run.target_yaw_rad >= 0.0
                                    else -1.0
                                ) * state.angular_displacement_rad
                                if run.trajectory.startswith('rotation_')
                                else state.path_length_m
                            )
                            target_progress = (
                                abs(run.target_yaw_rad)
                                if run.trajectory.startswith('rotation_')
                                else run.target_path_length_m
                            )
                            if phase == 'command' and progress >= target_progress:
                                stop_progress = state.path_length_m
                                stop_angle = state.angular_displacement_rad
                                command_elapsed = elapsed_phase
                                command_stop_utc = datetime.now(
                                    timezone.utc
                                ).isoformat()
                                self.stop()
                                reached_target = True
                                break

                        if reached_target:
                            break
                        if (
                            phase == 'command'
                            and elapsed_phase > self.args.odom_timeout
                            and (
                                last_odom is None
                                or now - last_odom > self.args.odom_timeout
                            )
                        ):
                            raise RuntimeError(
                                'ROS odometry became stale during motion; '
                                'the simulated robot was stopped.'
                            )
                    if emergency_stopped:
                        break
            finally:
                self.stop()
                stream.flush()

        summary: Dict[str, object] = {
            'run_id': current_run_id,
            'planned_run_id': run.run_id,
            'attempt': attempt,
            'status': (
                'emergency_stop_by_operator'
                if emergency_stopped
                else 'completed'
            ),
            'trajectory': run.trajectory,
            'repetition': run.repetition,
            'control_mode': 'simulated_velocity',
            'backend': 'ros',
            'target_path_length_m': run.target_path_length_m,
            'target_yaw_rad': run.target_yaw_rad,
            'target_curvature_per_m': run.curvature_per_m,
            'target_linear_speed_mps': run.target_linear_speed_mps,
            'target_left_speed_mps': run.target_left_speed_mps,
            'target_right_speed_mps': run.target_right_speed_mps,
            'figure_eight_radius_m': run.figure_eight_radius_m,
            'figure_eight_cycles': run.figure_eight_cycles,
            'figure_eight_start_direction': run.figure_eight_start_direction,
            'normalized_left': None,
            'normalized_right': None,
            'command_type': 'cmd_vel_linear_angular',
            'command_left': None,
            'command_right': None,
            'command_linear_mps': commands[0][0],
            'command_angular_rps': commands[0][1],
            'command_start_utc': command_start_utc,
            'command_stop_utc': command_stop_utc,
            'elapsed_command_s': command_elapsed,
            'odom_path_at_stop_m': stop_progress,
            'odom_stop_overshoot_m': stop_progress - run.target_path_length_m,
            'odom_angle_at_stop_rad': stop_angle,
            'odom_angle_overshoot_rad': (
                abs(stop_angle) - abs(run.target_yaw_rad)
                if run.trajectory.startswith('rotation_')
                else None
            ),
            'odom_final_path_m': state.path_length_m,
            'odom_final_x_m': state.x_m,
            'odom_final_y_m': state.y_m,
            'odom_final_yaw_rad': state.yaw_rad,
            **battery_statistics([]),
            'sample_file': str(sample_path.relative_to(self.output_dir)),
        }
        self.summary_table.append(summary)
        return summary

    def record_skipped(
        self, run: TrajectoryRun, commands: Sequence[CommandPair]
    ) -> None:
        row = {field: None for field in SUMMARY_FIELDS}
        row.update(
            {
                'run_id': run.run_id,
                'planned_run_id': run.run_id,
                'attempt': 0,
                'status': 'skipped_by_operator',
                'trajectory': run.trajectory,
                'repetition': run.repetition,
                'control_mode': 'simulated_velocity',
                'backend': 'ros',
                'target_path_length_m': run.target_path_length_m,
                'target_yaw_rad': run.target_yaw_rad,
                'target_curvature_per_m': run.curvature_per_m,
                'target_linear_speed_mps': run.target_linear_speed_mps,
                'target_left_speed_mps': run.target_left_speed_mps,
                'target_right_speed_mps': run.target_right_speed_mps,
                'figure_eight_radius_m': run.figure_eight_radius_m,
                'figure_eight_cycles': run.figure_eight_cycles,
                'figure_eight_start_direction': (
                    run.figure_eight_start_direction
                ),
                'command_type': 'cmd_vel_linear_angular',
                'command_linear_mps': commands[0][0],
                'command_angular_rps': commands[0][1],
            }
        )
        self.summary_table.append(row)


def confirm_start(automatic: bool) -> bool:
    if automatic:
        print(
            'AUTOMATIC MODE: the plan will run without reposition prompts.'
        )
        return True
    confirmation = input(
        'Type INICIAR after the arena is clear and the emergency stop is '
        'ready: '
    ).strip().upper()
    return confirmation == 'INICIAR'


def next_run_action(
    args: argparse.Namespace,
    run: TrajectoryRun,
    repeat_run_id: Optional[str] = None,
) -> str:
    if args.automatic:
        if args.inter_run_wait > 0.0:
            print(
                f'Automatic wait before {run.run_id}: '
                f'{args.inter_run_wait:.1f} s'
            )
            time.sleep(args.inter_run_wait)
        return 'run'
    repeat_help = f', r=repeat {repeat_run_id}' if repeat_run_id else ''
    response = input(
        f'Reposition for {run.run_id}, keep the video recording, then '
        f'ENTER=run, s=skip, q=finish{repeat_help}: '
    ).strip().lower()
    if response == 'q':
        return 'finish'
    if response == 's':
        return 'skip'
    if response == 'r' and repeat_run_id:
        return 'repeat'
    return 'run'


def run_countdown(seconds: int, run_id: str) -> None:
    print(f'VIDEO MARKER: next run is {run_id}')
    for remaining in range(seconds, 0, -1):
        print(f'  starting {run_id} in {remaining}...')
        time.sleep(1.0)
    print(
        f'VIDEO MARKER: START {run_id} at '
        f'{datetime.now(timezone.utc).isoformat()}'
    )


def print_run_result(summary: Dict[str, object]) -> None:
    """Print a video marker and the result of one trajectory attempt."""
    run_id = str(summary['run_id'])
    if summary['status'] == 'emergency_stop_by_operator':
        print(
            f'VIDEO MARKER: EMERGENCY STOP {run_id}; '
            f"odometry={float(summary['odom_final_path_m']):.4f} m; "
            'attempt invalid'
        )
        return
    battery = summary['battery_mean_v']
    if summary.get('backend') == 'ros':
        battery_text = 'not simulated'
    else:
        battery_text = (
            f'{float(battery):.2f} V' if battery is not None else 'missing'
        )
    print(
        f'VIDEO MARKER: STOP {run_id}; odometry='
        f"{float(summary['odom_path_at_stop_m']):.4f} m; "
        f"duration={float(summary['elapsed_command_s']):.3f} s; "
        f'battery={battery_text}'
    )


def execute_with_emergency_retries(
    runner: ExperimentRunner,
    args: argparse.Namespace,
    run: TrajectoryRun,
    commands: Sequence[CommandPair],
    attempts: Dict[str, int],
) -> Tuple[Dict[str, object], str]:
    """Run one trajectory and preserve emergency-stopped attempts."""
    while True:
        attempt = attempts.get(run.run_id, 0) + 1
        attempts[run.run_id] = attempt
        current_run_id = execution_run_id(run.run_id, attempt)
        run_countdown(args.countdown, current_run_id)
        summary = runner.execute(run, commands, attempt)
        print_run_result(summary)
        if summary['status'] != 'emergency_stop_by_operator':
            return summary, 'completed'
        if args.automatic:
            print('Automatic campaign stopped after the emergency command.')
            return summary, 'finish'

        response = input(
            'Reposition the robot after the emergency stop. '
            'ENTER=retry, s=skip this step, q=finish campaign: '
        ).strip().lower()
        if response == 'q':
            return summary, 'finish'
        if response == 's':
            return summary, 'skip'


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.backend == 'ros':
        args.control_mode = 'simulated_velocity'
    try:
        validate_args(args)
        plan, commands = prepare_plan(args)
    except ValueError as exc:
        parser.error(str(exc))
    print_plan(plan, commands, args)
    if args.dry_run:
        return 0

    if args.backend == 'ros':
        print(
            '\nSIMULATION: commands will be published as Twist on '
            f'{args.cmd_vel_topic} and stopped from {args.odom_topic}. '
            'This does not evaluate the embedded PI controller or battery.'
        )
    else:
        print(
            '\nSAFETY: this program directly owns the ESP32 port. Stop '
            'cmdvel_to_serial and serial_odom_node, clear the arena, and keep '
            'a physical emergency stop accessible. The phone video is '
            'independent.'
        )
    if not confirm_start(args.automatic):
        print('Campaign cancelled.')
        return 1

    output_dir = create_campaign_dir(args)
    write_metadata(output_dir, args, plan, commands)
    runner: Optional[ExperimentRunner] = None
    return_code = 0
    try:
        runner = (
            RosExperimentRunner(args, output_dir)
            if args.backend == 'ros'
            else ExperimentRunner(args, output_dir)
        )
        index = 0
        attempts: Dict[str, int] = {}
        last_run: Optional[TrajectoryRun] = None
        last_execution_id: Optional[str] = None
        while index < len(plan):
            run = plan[index]
            action = next_run_action(args, run, last_execution_id)
            if action == 'finish':
                break
            if action == 'repeat':
                if last_run is None or last_execution_id is None:
                    continue
                run = last_run
                previous_execution_id = last_execution_id
                summary, outcome = execute_with_emergency_retries(
                    runner,
                    args,
                    run,
                    commands[run.run_id],
                    attempts,
                )
                if outcome == 'finish':
                    break
                if outcome == 'skip':
                    continue
                runner.supersede(previous_execution_id)
                last_execution_id = str(summary['run_id'])
                continue
            if action == 'skip':
                runner.record_skipped(run, commands[run.run_id])
                last_run = None
                last_execution_id = None
                index += 1
                continue
            summary, outcome = execute_with_emergency_retries(
                runner,
                args,
                run,
                commands[run.run_id],
                attempts,
            )
            if outcome == 'finish':
                break
            if outcome == 'skip':
                last_run = None
                last_execution_id = None
                index += 1
                continue
            last_run = run
            last_execution_id = str(summary['run_id'])
            index += 1
    except KeyboardInterrupt:
        print('\nEmergency interruption received; sending stop command.')
        return_code = 130
    except CALIBRATION_EXCEPTIONS + (ValueError,) as exc:
        print(f'Campaign aborted safely: {exc}', file=sys.stderr)
        return_code = 2
    finally:
        if runner is not None:
            runner.close()
        print(f'Data directory: {output_dir}')
    return return_code


if __name__ == '__main__':
    raise SystemExit(main())
