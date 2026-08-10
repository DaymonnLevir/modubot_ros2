#!/usr/bin/env python3
"""Drive finite trajectories and stop from the ESP32 wheel odometry."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

from .feedforward_calibration import (
    CALIBRATION_EXCEPTIONS,
    Esp32Serial,
    csv_value,
    norm_to_dac,
    parse_odom_line,
)
from .trajectory_experiment_core import (
    FeedforwardMap,
    OdometryState,
    TrajectoryRun,
    build_trajectory_plan,
    commands_for_run,
    desired_pose,
    integrate_differential_drive,
)


SCRIPT_VERSION = '1.0.0'

SAMPLE_FIELDS = [
    'timestamp_utc',
    'run_id',
    'trajectory',
    'repetition',
    'phase',
    'elapsed_run_s',
    'elapsed_phase_s',
    'target_path_length_m',
    'target_curvature_per_m',
    'target_linear_speed_mps',
    'target_left_speed_mps',
    'target_right_speed_mps',
    'normalized_left',
    'normalized_right',
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
    'desired_x_m',
    'desired_y_m',
    'desired_yaw_rad',
]

SUMMARY_FIELDS = [
    'run_id',
    'status',
    'trajectory',
    'repetition',
    'control_mode',
    'target_path_length_m',
    'target_curvature_per_m',
    'target_linear_speed_mps',
    'target_left_speed_mps',
    'target_right_speed_mps',
    'normalized_left',
    'normalized_right',
    'command_start_utc',
    'command_stop_utc',
    'elapsed_command_s',
    'odom_path_at_stop_m',
    'odom_stop_overshoot_m',
    'odom_final_path_m',
    'odom_final_x_m',
    'odom_final_y_m',
    'odom_final_yaw_rad',
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
        return ['straight', 'arc_left', 'arc_right']
    allowed = {'straight', 'arc_left', 'arc_right'}
    result: List[str] = []
    for token in value.split(','):
        name = token.strip().lower()
        if not name:
            continue
        if name not in allowed:
            raise argparse.ArgumentTypeError(
                'Trajectories must be straight, arc_left, arc_right, or all.'
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
            'Drive straight or constant-curvature paths and stop when the '
            'integrated wheel odometry reaches the requested path length.'
        )
    )
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument(
        '--trajectories', type=parse_trajectories, default=['straight']
    )
    parser.add_argument('--repetitions', type=int, default=5)
    parser.add_argument('--distance', type=float, default=1.5)
    parser.add_argument('--arc-length', type=float, default=0.8)
    parser.add_argument('--arc-radius', type=float, default=0.8)
    parser.add_argument('--linear-speed', type=float, default=0.25)
    parser.add_argument(
        '--control-mode', choices=['raw', 'feedforward'], default='feedforward'
    )
    parser.add_argument('--feedforward-map', type=Path)
    parser.add_argument('--v-wheel-max', type=float, default=0.6)
    parser.add_argument('--max-normalized-command', type=float, default=0.30)
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
    parser.add_argument('--max-duration', type=float, default=20.0)
    parser.add_argument('--preflight-timeout', type=float, default=3.0)
    parser.add_argument('--startup-wait', type=float, default=2.0)
    parser.add_argument('--ticks-left', type=float, default=91.0)
    parser.add_argument('--ticks-right', type=float, default=91.0)
    parser.add_argument('--wheel-radius-left', type=float, default=0.078)
    parser.add_argument('--wheel-radius-right', type=float, default=0.078)
    parser.add_argument('--wheel-separation', type=float, default=0.207)
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
        '--dry-run',
        action='store_true',
        help='Validate and print the plan without opening the serial port.',
    )
    return parser


def validate_args(args: argparse.Namespace) -> None:
    positive = {
        'baud': args.baud,
        'repetitions': args.repetitions,
        'distance': args.distance,
        'arc_length': args.arc_length,
        'arc_radius': args.arc_radius,
        'linear_speed': args.linear_speed,
        'v_wheel_max': args.v_wheel_max,
        'max_normalized_command': args.max_normalized_command,
        'send_rate': args.send_rate,
        'odom_timeout': args.odom_timeout,
        'max_duration': args.max_duration,
        'preflight_timeout': args.preflight_timeout,
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
    if args.control_mode == 'feedforward' and args.feedforward_map is None:
        raise ValueError('feedforward mode requires --feedforward-map.')
    if args.feedforward_map is not None and not args.feedforward_map.is_file():
        raise ValueError(f'Feedforward map not found: {args.feedforward_map}')
    if args.battery_start_v is not None and args.battery_start_v <= 0:
        raise ValueError('battery_start_v must be greater than zero.')


def prepare_plan(
    args: argparse.Namespace,
) -> Tuple[List[TrajectoryRun], Dict[str, Tuple[float, float]]]:
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
    )
    commands: Dict[str, Tuple[float, float]] = {}
    for run in plan:
        pair = commands_for_run(
            run, args.control_mode, args.v_wheel_max, calibration
        )
        if max(abs(pair[0]), abs(pair[1])) > args.max_normalized_command:
            raise ValueError(
                f'{run.run_id} requires commands {pair}, exceeding '
                f'--max-normalized-command={args.max_normalized_command:.3f}.'
            )
        commands[run.run_id] = pair
    return plan, commands


def print_plan(
    plan: Sequence[TrajectoryRun],
    commands: Dict[str, Tuple[float, float]],
    args: argparse.Namespace,
) -> None:
    print('\nPlanned odometry-stopped runs:')
    for index, run in enumerate(plan, start=1):
        left, right = commands[run.run_id]
        print(
            f'  {index:02d}/{len(plan):02d} {run.run_id}: '
            f's={run.target_path_length_m:.3f} m, '
            f'k={run.curvature_per_m:+.3f} 1/m, '
            f'vL={run.target_left_speed_mps:.3f} m/s, '
            f'vR={run.target_right_speed_mps:.3f} m/s, '
            f'uL={left:+.4f}, uR={right:+.4f}'
        )
    print(
        f'  order={args.order}, seed={args.seed}, '
        f'control={args.control_mode}, video={args.video_file}'
    )


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
    commands: Dict[str, Tuple[float, float]],
) -> None:
    metadata = {
        'script_version': SCRIPT_VERSION,
        'created_at_utc': datetime.now(timezone.utc).isoformat(),
        'method': (
            'The ESP32 serial link is owned exclusively by this process. '
            'Integrated wheel increments stop each run at the requested '
            'odometric center-path length. AprilTag video is recorded '
            'independently and measured offline.'
        ),
        'odometry_definition': (
            'center increment=(left_distance+right_distance)/2; stop progress '
            'is the sum of the absolute center increments during the command.'
        ),
        'arguments': {
            key: str(value) if isinstance(value, Path) else value
            for key, value in vars(args).items()
        },
        'plan': [
            {
                **asdict(run),
                'normalized_left': commands[run.run_id][0],
                'normalized_right': commands[run.run_id][1],
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
        self.summary_stream = (output_dir / 'summary.csv').open(
            'w', newline='', encoding='utf-8'
        )
        self.summary_writer = csv.DictWriter(
            self.summary_stream, fieldnames=SUMMARY_FIELDS
        )
        self.summary_writer.writeheader()
        self.summary_stream.flush()
        self.link = Esp32Serial(args.port, args.baud, args.startup_wait)
        print('Waiting for ESP32 odometry telemetry...')
        first = self.link.wait_for_telemetry(args.preflight_timeout)
        print(f'ESP32 telemetry confirmed: O {first[0]} {first[1]} {first[2]}')

    def close(self) -> None:
        try:
            self.link.close()
        finally:
            self.summary_stream.close()

    def _sample_row(
        self,
        run: TrajectoryRun,
        phase: str,
        elapsed_run_s: float,
        elapsed_phase_s: float,
        command: Tuple[float, float],
        telemetry: Tuple[int, int, int],
        state: OdometryState,
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
        desired_x, desired_y, desired_yaw = desired_pose(
            min(state.path_length_m, run.target_path_length_m),
            run.curvature_per_m,
        )
        return {
            'timestamp_utc': datetime.now(timezone.utc).isoformat(),
            'run_id': run.run_id,
            'trajectory': run.trajectory,
            'repetition': run.repetition,
            'phase': phase,
            'elapsed_run_s': elapsed_run_s,
            'elapsed_phase_s': elapsed_phase_s,
            'target_path_length_m': run.target_path_length_m,
            'target_curvature_per_m': run.curvature_per_m,
            'target_linear_speed_mps': run.target_linear_speed_mps,
            'target_left_speed_mps': run.target_left_speed_mps,
            'target_right_speed_mps': run.target_right_speed_mps,
            'normalized_left': command[0],
            'normalized_right': command[1],
            'dac_left': norm_to_dac(command[0]),
            'dac_right': norm_to_dac(command[1]),
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
            'desired_x_m': desired_x,
            'desired_y_m': desired_y,
            'desired_yaw_rad': desired_yaw,
        }

    def execute(
        self,
        run: TrajectoryRun,
        command: Tuple[float, float],
    ) -> Dict[str, object]:
        sample_path = self.output_dir / 'samples' / f'{run.run_id}.csv'
        self.link.stop()
        self.link.clear_input()
        state = OdometryState()
        run_start = time.monotonic()
        last_odom: Optional[float] = None
        stop_progress = 0.0
        command_elapsed = 0.0
        command_start_utc: Optional[str] = None
        command_stop_utc: Optional[str] = None

        with sample_path.open('w', newline='', encoding='utf-8') as stream:
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
                                self.link.command(command[0], command[1])
                            else:
                                self.link.write_line('S')
                            next_send = now + 1.0 / self.args.send_rate

                        for line in self.link.read_lines():
                            telemetry = parse_odom_line(line)
                            if telemetry is None:
                                continue
                            last_odom = time.monotonic()
                            active_command = (
                                command if phase == 'command' else (0.0, 0.0)
                            )
                            row = self._sample_row(
                                run,
                                phase,
                                now - run_start,
                                elapsed_phase,
                                active_command,
                                telemetry,
                                state,
                            )
                            writer.writerow(
                                {
                                    key: csv_value(value)
                                    for key, value in row.items()
                                }
                            )
                            if (
                                phase == 'command'
                                and state.path_length_m
                                >= run.target_path_length_m
                            ):
                                stop_progress = state.path_length_m
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
            finally:
                self.link.stop()
                stream.flush()

        summary: Dict[str, object] = {
            'run_id': run.run_id,
            'status': 'completed',
            'trajectory': run.trajectory,
            'repetition': run.repetition,
            'control_mode': self.args.control_mode,
            'target_path_length_m': run.target_path_length_m,
            'target_curvature_per_m': run.curvature_per_m,
            'target_linear_speed_mps': run.target_linear_speed_mps,
            'target_left_speed_mps': run.target_left_speed_mps,
            'target_right_speed_mps': run.target_right_speed_mps,
            'normalized_left': command[0],
            'normalized_right': command[1],
            'command_start_utc': command_start_utc,
            'command_stop_utc': command_stop_utc,
            'elapsed_command_s': command_elapsed,
            'odom_path_at_stop_m': stop_progress,
            'odom_stop_overshoot_m': stop_progress - run.target_path_length_m,
            'odom_final_path_m': state.path_length_m,
            'odom_final_x_m': state.x_m,
            'odom_final_y_m': state.y_m,
            'odom_final_yaw_rad': state.yaw_rad,
            'sample_file': str(sample_path.relative_to(self.output_dir)),
        }
        self.summary_writer.writerow(
            {key: csv_value(value) for key, value in summary.items()}
        )
        self.summary_stream.flush()
        return summary


def confirm_start() -> bool:
    confirmation = input(
        'Type INICIAR after the arena is clear and the emergency stop is '
        'ready: '
    ).strip().upper()
    return confirmation == 'INICIAR'


def next_run_action(run: TrajectoryRun) -> str:
    response = input(
        f'Reposition for {run.run_id}, keep the video recording, then '
        'ENTER=run, s=skip, q=finish: '
    ).strip().lower()
    if response == 'q':
        return 'finish'
    if response == 's':
        return 'skip'
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


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        validate_args(args)
        plan, commands = prepare_plan(args)
    except ValueError as exc:
        parser.error(str(exc))
    print_plan(plan, commands, args)
    if args.dry_run:
        return 0

    print(
        '\nSAFETY: this program directly owns the ESP32 port. Stop '
        'cmdvel_to_serial and serial_odom_node, clear the arena, and keep a '
        'physical emergency stop accessible. The phone video is independent.'
    )
    if not confirm_start():
        print('Campaign cancelled.')
        return 1

    output_dir = create_campaign_dir(args)
    write_metadata(output_dir, args, plan, commands)
    runner: Optional[ExperimentRunner] = None
    return_code = 0
    try:
        runner = ExperimentRunner(args, output_dir)
        for run in plan:
            action = next_run_action(run)
            if action == 'finish':
                break
            if action == 'skip':
                continue
            run_countdown(args.countdown, run.run_id)
            summary = runner.execute(run, commands[run.run_id])
            print(
                f"VIDEO MARKER: STOP {run.run_id}; odometry="
                f"{float(summary['odom_path_at_stop_m']):.4f} m; "
                f"duration={float(summary['elapsed_command_s']):.3f} s"
            )
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
