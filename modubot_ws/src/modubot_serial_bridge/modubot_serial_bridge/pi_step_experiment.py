#!/usr/bin/env python3
"""Run repeatable open-loop versus PI wheel-speed step experiments."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import random
import statistics
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

from .experiment_support import (
    SummaryTable,
    battery_statistics,
    csv_value,
    execution_run_id,
    parse_battery_line,
)
from .feedforward_calibration import (
    CALIBRATION_EXCEPTIONS,
    Esp32Serial,
)
from .trajectory_experiment_core import FeedforwardMap


SCRIPT_VERSION = '1.0.0'
TELEMETRY_FIELDS = [
    'firmware_time_ms',
    'setpoint_left_rps',
    'measured_left_rps',
    'controller_left',
    'dac_left',
    'setpoint_right_rps',
    'measured_right_rps',
    'controller_right',
    'dac_right',
]
SAMPLE_FIELDS = [
    'timestamp_utc',
    'run_id',
    'planned_run_id',
    'attempt',
    'control_condition',
    'direction',
    'target_speed_mps',
    'repetition',
    'phase',
    'elapsed_run_s',
    'elapsed_phase_s',
    *TELEMETRY_FIELDS,
    'setpoint_left_mps',
    'measured_left_mps',
    'setpoint_right_mps',
    'measured_right_mps',
    'battery_voltage_v',
]
SUMMARY_FIELDS = [
    'run_id',
    'planned_run_id',
    'attempt',
    'status',
    'control_condition',
    'baseline_mode',
    'direction',
    'target_speed_mps',
    'repetition',
    'command_left',
    'command_right',
    'command_time_s',
    'steady_samples',
    'left_mean_mps',
    'right_mean_mps',
    'left_steady_error_mps',
    'right_steady_error_mps',
    'left_rmse_mps',
    'right_rmse_mps',
    'left_rise_time_s',
    'right_rise_time_s',
    'left_overshoot_percent',
    'right_overshoot_percent',
    'battery_samples',
    'battery_start_v',
    'battery_end_v',
    'battery_mean_v',
    'battery_min_v',
    'battery_max_v',
    'battery_drop_v',
    'sample_file',
]


@dataclass(frozen=True)
class StepRun:
    """One planned speed step under a selected low-level controller."""

    run_id: str
    control_condition: str
    direction: str
    target_speed_mps: float
    repetition: int


def parse_positive_values(value: str) -> List[float]:
    result: List[float] = []
    for token in value.split(','):
        number = float(token.strip())
        if number <= 0.0:
            raise argparse.ArgumentTypeError('Speeds must be positive.')
        if number not in result:
            result.append(number)
    if not result:
        raise argparse.ArgumentTypeError('At least one speed is required.')
    return result


def parse_pi_telemetry(line: str) -> Optional[List[float]]:
    parts = line.strip().split()
    if len(parts) != 10 or parts[0] != 'T':
        return None
    try:
        return [float(value) for value in parts[1:]]
    except ValueError:
        return None


def build_step_plan(
    speeds: Sequence[float],
    direction: str,
    repetitions: int,
    order: str,
    seed: int,
) -> List[StepRun]:
    directions = ['forward', 'reverse'] if direction == 'both' else [direction]
    plan: List[StepRun] = []
    for repetition in range(1, repetitions + 1):
        for current_direction in directions:
            sign = 'FWD' if current_direction == 'forward' else 'REV'
            for speed in speeds:
                speed_label = int(round(speed * 1000.0))
                for condition in ['baseline', 'pi']:
                    plan.append(
                        StepRun(
                            run_id=(
                                f'STEP_{sign}_{speed_label:04d}MMPS_'
                                f'{condition.upper()}_R{repetition:02d}'
                            ),
                            control_condition=condition,
                            direction=current_direction,
                            target_speed_mps=speed,
                            repetition=repetition,
                        )
                    )
    if order == 'randomized':
        random.Random(seed).shuffle(plan)
    elif order == 'blocked':
        plan.sort(
            key=lambda run: (
                run.direction,
                run.target_speed_mps,
                run.control_condition,
                run.repetition,
            )
        )
    elif order != 'interleaved':
        raise ValueError(f'Unsupported order: {order}')
    return plan


def default_output_dir() -> Path:
    configured = os.environ.get('MODUBOT_STEP_DIR')
    if configured:
        return Path(configured).expanduser()
    workspace = Path('/workspace/modubot_ws')
    if workspace.is_dir():
        return workspace / 'step_data'
    return Path.home() / 'modubot_step_data'


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Compare open-loop and embedded PI wheel-speed steps.'
    )
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument(
        '--speeds',
        type=parse_positive_values,
        default=parse_positive_values('0.10,0.20,0.30'),
    )
    parser.add_argument(
        '--direction', choices=['forward', 'reverse', 'both'], default='both'
    )
    parser.add_argument('--repetitions', type=int, default=5)
    parser.add_argument(
        '--baseline-mode', choices=['raw', 'feedforward'], default='raw'
    )
    parser.add_argument('--feedforward-map', type=Path)
    parser.add_argument('--v-wheel-max', type=float, default=0.6)
    parser.add_argument('--wheel-radius-left', type=float, default=0.078)
    parser.add_argument('--wheel-radius-right', type=float, default=0.078)
    parser.add_argument('--kp', type=float, default=12.0)
    parser.add_argument('--ki', type=float, default=40.0)
    parser.add_argument('--kd', type=float, default=0.0)
    parser.add_argument('--kff', type=float, default=0.0)
    parser.add_argument('--dac-min', type=float, default=0.0)
    parser.add_argument('--pre-time', type=float, default=2.0)
    parser.add_argument('--command-time', type=float, default=6.0)
    parser.add_argument('--post-time', type=float, default=2.0)
    parser.add_argument('--steady-start', type=float, default=3.0)
    parser.add_argument('--send-rate', type=float, default=20.0)
    parser.add_argument('--telemetry-rate', type=int, default=25)
    parser.add_argument('--preflight-timeout', type=float, default=3.0)
    parser.add_argument('--battery-timeout', type=float, default=3.0)
    parser.add_argument('--startup-wait', type=float, default=2.0)
    parser.add_argument(
        '--order',
        choices=['randomized', 'interleaved', 'blocked'],
        default='interleaved',
    )
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--surface', default='not_recorded')
    parser.add_argument('--notes', default='')
    parser.add_argument('--campaign-name', default='pi_step')
    parser.add_argument('--output-dir', type=Path, default=default_output_dir())
    parser.add_argument('--automatic', action='store_true')
    parser.add_argument('--inter-run-wait', type=float, default=5.0)
    parser.add_argument('--dry-run', action='store_true')
    return parser


def validate_args(args: argparse.Namespace) -> None:
    positive = {
        'baud': args.baud,
        'repetitions': args.repetitions,
        'v_wheel_max': args.v_wheel_max,
        'wheel_radius_left': args.wheel_radius_left,
        'wheel_radius_right': args.wheel_radius_right,
        'command_time': args.command_time,
        'send_rate': args.send_rate,
        'telemetry_rate': args.telemetry_rate,
        'preflight_timeout': args.preflight_timeout,
        'battery_timeout': args.battery_timeout,
    }
    for name, value in positive.items():
        if value <= 0:
            raise ValueError(f'{name} must be greater than zero.')
    if min(
        args.pre_time,
        args.post_time,
        args.steady_start,
        args.startup_wait,
        args.inter_run_wait,
    ) < 0:
        raise ValueError('Time arguments cannot be negative.')
    if args.steady_start >= args.command_time:
        raise ValueError('steady_start must be inside command_time.')
    if min(args.kp, args.ki, args.kd, args.kff, args.dac_min) < 0:
        raise ValueError('Controller gains and dac_min cannot be negative.')
    if args.baseline_mode == 'feedforward':
        if args.feedforward_map is None:
            raise ValueError('feedforward baseline requires --feedforward-map.')
        if not args.feedforward_map.is_file():
            raise ValueError(f'Map not found: {args.feedforward_map}')


def baseline_command(
    run: StepRun,
    args: argparse.Namespace,
    calibration: Optional[FeedforwardMap],
) -> Tuple[float, float]:
    signed_speed = (
        run.target_speed_mps
        if run.direction == 'forward'
        else -run.target_speed_mps
    )
    if args.baseline_mode == 'feedforward':
        assert calibration is not None
        return (
            calibration.command_for_speed('left', signed_speed),
            calibration.command_for_speed('right', signed_speed),
        )
    command = signed_speed / args.v_wheel_max
    if abs(command) > 1.0:
        raise ValueError(
            f'{run.run_id} requires normalized command {command:.3f}.'
        )
    return command, command


def command_for_run(
    run: StepRun,
    args: argparse.Namespace,
    calibration: Optional[FeedforwardMap],
) -> Tuple[float, float]:
    if run.control_condition == 'baseline':
        return baseline_command(run, args, calibration)
    signed_speed = (
        run.target_speed_mps
        if run.direction == 'forward'
        else -run.target_speed_mps
    )
    return (
        signed_speed / args.wheel_radius_left,
        signed_speed / args.wheel_radius_right,
    )


def create_campaign_dir(args: argparse.Namespace) -> Path:
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    path = args.output_dir.expanduser() / f'{args.campaign_name}_{stamp}'
    path.mkdir(parents=True, exist_ok=False)
    (path / 'samples').mkdir()
    return path


def mean_or_none(values: Sequence[float]) -> Optional[float]:
    return statistics.fmean(values) if values else None


def rmse(values: Sequence[float], target: float) -> Optional[float]:
    if not values:
        return None
    return math.sqrt(statistics.fmean((value - target) ** 2 for value in values))


def overshoot_percent(values: Sequence[float], target: float) -> Optional[float]:
    if not values or abs(target) < 1e-12:
        return None
    direction = 1.0 if target > 0.0 else -1.0
    peak = max(direction * value for value in values)
    return 100.0 * max(0.0, peak - abs(target)) / abs(target)


class StepRunner:
    """Exclusive serial runner for one step-response campaign."""

    def __init__(self, args: argparse.Namespace, output_dir: Path) -> None:
        self.args = args
        self.output_dir = output_dir
        self.summary = SummaryTable(output_dir / 'summary.csv', SUMMARY_FIELDS)
        self.link = Esp32Serial(args.port, args.baud, args.startup_wait)
        try:
            first = self.link.wait_for_telemetry(args.preflight_timeout)
            self.link.write_line(f'P 1 {args.telemetry_rate}')
            self._wait_for_pi_telemetry(args.preflight_timeout)
            first_battery = self.link.wait_for_battery(
                args.battery_timeout
            )
        except RuntimeError:
            self.link.close()
            raise
        print(f'ESP32 telemetry confirmed: O {first[0]} {first[1]} {first[2]}')
        print(f'Battery telemetry confirmed: {first_battery:.2f} V')

    def _wait_for_pi_telemetry(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for line in self.link.read_lines():
                if parse_pi_telemetry(line) is not None:
                    print('PI telemetry confirmed.')
                    return
            time.sleep(0.01)
        raise RuntimeError(
            'No T telemetry received after enabling P. Verify that the '
            'uploaded firmware supports PI telemetry.'
        )

    def close(self) -> None:
        try:
            self.link.write_line('P 0')
        finally:
            self.link.close()

    def configure(self, condition: str) -> None:
        self.link.stop()
        if condition == 'pi':
            self.link.write_line(
                f'K {self.args.kp} {self.args.ki} {self.args.kd}'
            )
            self.link.write_line(
                f'F {self.args.kff} {self.args.dac_min}'
            )
            self.link.write_line('M 1')
        else:
            self.link.write_line('M 0')

    def send_command(self, run: StepRun, command: Tuple[float, float]) -> None:
        if run.control_condition == 'pi':
            self.link.write_line(f'W {command[0]:.6f} {command[1]:.6f}')
        else:
            self.link.command(command[0], command[1])

    def execute(
        self,
        run: StepRun,
        command: Tuple[float, float],
        attempt: int,
    ) -> Dict[str, object]:
        current_id = execution_run_id(run.run_id, attempt)
        path = self.output_dir / 'samples' / f'{current_id}.csv'
        self.configure(run.control_condition)
        self.link.clear_input()
        target = (
            run.target_speed_mps
            if run.direction == 'forward'
            else -run.target_speed_mps
        )
        latest_battery: Optional[float] = None
        battery_values: List[float] = []
        steady_left: List[float] = []
        steady_right: List[float] = []
        command_left_values: List[float] = []
        command_right_values: List[float] = []
        rise_left: Optional[float] = None
        rise_right: Optional[float] = None
        run_start = time.monotonic()

        with path.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=SAMPLE_FIELDS)
            writer.writeheader()
            try:
                for phase, duration in [
                    ('pre_stop', self.args.pre_time),
                    ('command', self.args.command_time),
                    ('post_stop', self.args.post_time),
                ]:
                    phase_start = time.monotonic()
                    next_send = phase_start
                    self.link.stop()
                    while time.monotonic() - phase_start < duration:
                        now = time.monotonic()
                        elapsed = now - phase_start
                        if now >= next_send:
                            if phase == 'command':
                                self.send_command(run, command)
                            else:
                                self.link.write_line('S')
                            next_send = now + 1.0 / self.args.send_rate
                        for line in self.link.read_lines():
                            battery = parse_battery_line(line)
                            if battery is not None:
                                latest_battery = battery
                                battery_values.append(battery)
                                continue
                            values = parse_pi_telemetry(line)
                            if values is None:
                                continue
                            data = dict(zip(TELEMETRY_FIELDS, values))
                            left_mps = (
                                data['measured_left_rps']
                                * self.args.wheel_radius_left
                            )
                            right_mps = (
                                data['measured_right_rps']
                                * self.args.wheel_radius_right
                            )
                            setpoint_left_mps = (
                                data['setpoint_left_rps']
                                * self.args.wheel_radius_left
                            )
                            setpoint_right_mps = (
                                data['setpoint_right_rps']
                                * self.args.wheel_radius_right
                            )
                            row = {
                                'timestamp_utc': datetime.now(
                                    timezone.utc
                                ).isoformat(),
                                'run_id': current_id,
                                'planned_run_id': run.run_id,
                                'attempt': attempt,
                                'control_condition': run.control_condition,
                                'direction': run.direction,
                                'target_speed_mps': target,
                                'repetition': run.repetition,
                                'phase': phase,
                                'elapsed_run_s': now - run_start,
                                'elapsed_phase_s': elapsed,
                                **data,
                                'setpoint_left_mps': setpoint_left_mps,
                                'measured_left_mps': left_mps,
                                'setpoint_right_mps': setpoint_right_mps,
                                'measured_right_mps': right_mps,
                                'battery_voltage_v': latest_battery,
                            }
                            writer.writerow(
                                {
                                    key: csv_value(row.get(key))
                                    for key in SAMPLE_FIELDS
                                }
                            )
                            if phase == 'command':
                                command_left_values.append(left_mps)
                                command_right_values.append(right_mps)
                                threshold = 0.9 * abs(target)
                                direction = 1.0 if target > 0.0 else -1.0
                                if (
                                    rise_left is None
                                    and direction * left_mps >= threshold
                                ):
                                    rise_left = elapsed
                                if (
                                    rise_right is None
                                    and direction * right_mps >= threshold
                                ):
                                    rise_right = elapsed
                                if elapsed >= self.args.steady_start:
                                    steady_left.append(left_mps)
                                    steady_right.append(right_mps)
                        time.sleep(0.005)
            finally:
                self.link.stop()

        left_mean = mean_or_none(steady_left)
        right_mean = mean_or_none(steady_right)
        summary: Dict[str, object] = {
            'run_id': current_id,
            'planned_run_id': run.run_id,
            'attempt': attempt,
            'status': (
                'completed' if steady_left else 'no_steady_samples'
            ),
            'control_condition': run.control_condition,
            'baseline_mode': (
                self.args.baseline_mode
                if run.control_condition == 'baseline'
                else None
            ),
            'direction': run.direction,
            'target_speed_mps': target,
            'repetition': run.repetition,
            'command_left': command[0],
            'command_right': command[1],
            'command_time_s': self.args.command_time,
            'steady_samples': len(steady_left),
            'left_mean_mps': left_mean,
            'right_mean_mps': right_mean,
            'left_steady_error_mps': (
                left_mean - target if left_mean is not None else None
            ),
            'right_steady_error_mps': (
                right_mean - target if right_mean is not None else None
            ),
            'left_rmse_mps': rmse(command_left_values, target),
            'right_rmse_mps': rmse(command_right_values, target),
            'left_rise_time_s': rise_left,
            'right_rise_time_s': rise_right,
            'left_overshoot_percent': overshoot_percent(
                command_left_values, target
            ),
            'right_overshoot_percent': overshoot_percent(
                command_right_values, target
            ),
            **battery_statistics(battery_values),
            'sample_file': str(path.relative_to(self.output_dir)),
        }
        self.summary.append(summary)
        return summary

    def supersede(self, execution_id: str) -> None:
        if not self.summary.supersede(execution_id):
            raise RuntimeError(f'Could not supersede {execution_id}.')

    def record_skipped(
        self, run: StepRun, command: Tuple[float, float]
    ) -> None:
        target = (
            run.target_speed_mps
            if run.direction == 'forward'
            else -run.target_speed_mps
        )
        row = {field: None for field in SUMMARY_FIELDS}
        row.update(
            {
                'run_id': run.run_id,
                'planned_run_id': run.run_id,
                'attempt': 0,
                'status': 'skipped_by_operator',
                'control_condition': run.control_condition,
                'baseline_mode': (
                    self.args.baseline_mode
                    if run.control_condition == 'baseline'
                    else None
                ),
                'direction': run.direction,
                'target_speed_mps': target,
                'repetition': run.repetition,
                'command_left': command[0],
                'command_right': command[1],
                'command_time_s': self.args.command_time,
            }
        )
        self.summary.append(row)


def next_action(
    args: argparse.Namespace,
    run: StepRun,
    repeat_id: Optional[str],
) -> str:
    if args.automatic:
        time.sleep(args.inter_run_wait)
        return 'run'
    repeat_help = f', r=repeat {repeat_id}' if repeat_id else ''
    answer = input(
        f'Reposition for {run.run_id}. ENTER=run, s=skip, q=finish'
        f'{repeat_help}: '
    ).strip().lower()
    if answer == 'q':
        return 'finish'
    if answer == 's':
        return 'skip'
    if answer == 'r' and repeat_id:
        return 'repeat'
    return 'run'


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        validate_args(args)
        plan = build_step_plan(
            args.speeds,
            args.direction,
            args.repetitions,
            args.order,
            args.seed,
        )
        calibration = (
            FeedforwardMap.from_csv(args.feedforward_map)
            if args.feedforward_map is not None
            else None
        )
        commands = {
            run.run_id: command_for_run(run, args, calibration)
            for run in plan
        }
    except ValueError as exc:
        parser.error(str(exc))

    print(f'Planned step executions: {len(plan)}')
    for index, run in enumerate(plan, start=1):
        print(
            f'  {index:02d}/{len(plan):02d} {run.run_id}: '
            f'{run.control_condition}, {run.target_speed_mps:.3f} m/s'
        )
    if args.dry_run:
        return 0
    if not args.automatic:
        answer = input('Type INICIAR after clearing the area: ').strip().upper()
        if answer != 'INICIAR':
            return 1

    output_dir = create_campaign_dir(args)
    metadata = {
        'script_version': SCRIPT_VERSION,
        'created_at_utc': datetime.now(timezone.utc).isoformat(),
        'arguments': {
            key: str(value) if isinstance(value, Path) else value
            for key, value in vars(args).items()
        },
        'plan': [
            {**asdict(run), 'command': commands[run.run_id]}
            for run in plan
        ],
        'battery_protocol': 'BAT:<voltage_v>',
    }
    (output_dir / 'metadata.json').write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False), encoding='utf-8'
    )

    runner: Optional[StepRunner] = None
    return_code = 0
    try:
        runner = StepRunner(args, output_dir)
        index = 0
        attempts: Dict[str, int] = {}
        last_run: Optional[StepRun] = None
        last_id: Optional[str] = None
        while index < len(plan):
            run = plan[index]
            action = next_action(args, run, last_id)
            if action == 'finish':
                break
            if action == 'skip':
                runner.record_skipped(run, commands[run.run_id])
                last_run = None
                last_id = None
                index += 1
                continue
            if action == 'repeat':
                if last_run is None or last_id is None:
                    continue
                runner.supersede(last_id)
                run = last_run
            else:
                index += 1
            attempt = attempts.get(run.run_id, 0) + 1
            attempts[run.run_id] = attempt
            result = runner.execute(run, commands[run.run_id], attempt)
            last_run = run
            last_id = str(result['run_id'])
            print(
                f"Completed {last_id}: L={result['left_mean_mps']} m/s, "
                f"R={result['right_mean_mps']} m/s, "
                f"battery={result['battery_mean_v']} V"
            )
    except KeyboardInterrupt:
        print('\nInterrupted; brake command sent.')
        return_code = 130
    except CALIBRATION_EXCEPTIONS + (ValueError,) as exc:
        print(f'Experiment aborted safely: {exc}', file=sys.stderr)
        return_code = 2
    finally:
        if runner is not None:
            runner.close()
        print(f'Data directory: {output_dir}')
    return return_code


if __name__ == '__main__':
    raise SystemExit(main())
