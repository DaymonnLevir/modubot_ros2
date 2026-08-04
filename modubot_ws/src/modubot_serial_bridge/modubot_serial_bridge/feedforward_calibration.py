#!/usr/bin/env python3
"""Interactive ground-test runner for ModuBot feedforward calibration.

This process is the sole owner of the ESP32 serial port during a campaign. It
sends normalized left/right commands, parses ``O dL dR dt_ms`` telemetry, and
writes both raw samples and steady-state summaries to CSV files.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import statistics
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

try:
    import serial
except ImportError:
    serial = None


if serial is None:
    SERIAL_EXCEPTIONS: Tuple[type, ...] = ()
else:
    SERIAL_EXCEPTIONS = (
        serial.SerialException,
        serial.SerialTimeoutException,
    )
CALIBRATION_EXCEPTIONS = (RuntimeError,) + SERIAL_EXCEPTIONS


SCRIPT_VERSION = '1.1.0'
DEFAULT_LEVELS = '0.05,0.075,0.10,0.125,0.15,0.175,0.20,0.25,0.30'

SAMPLE_FIELDS = [
    'timestamp_utc',
    'run_id',
    'direction',
    'mode',
    'level_index',
    'repetition',
    'phase',
    'elapsed_run_s',
    'elapsed_phase_s',
    'normalized_left',
    'normalized_right',
    'dac_left',
    'dac_right',
    'nominal_voltage_left_v',
    'nominal_voltage_right_v',
    'equivalent_cmd_vel_mps',
    'delta_ticks_left',
    'delta_ticks_right',
    'dt_ms',
    'speed_left_mps',
    'speed_right_mps',
    'body_linear_mps',
    'body_angular_rps',
]

SUMMARY_FIELDS = [
    'run_id',
    'status',
    'direction',
    'mode',
    'level_index',
    'repetition',
    'normalized_left',
    'normalized_right',
    'dac_left',
    'dac_right',
    'nominal_voltage_left_v',
    'nominal_voltage_right_v',
    'equivalent_cmd_vel_mps',
    'command_time_s',
    'steady_start_s',
    'steady_end_margin_s',
    'steady_samples',
    'speed_left_mean_mps',
    'speed_left_median_mps',
    'speed_left_std_mps',
    'speed_left_min_mps',
    'speed_left_max_mps',
    'speed_right_mean_mps',
    'speed_right_median_mps',
    'speed_right_std_mps',
    'speed_right_min_mps',
    'speed_right_max_mps',
    'body_linear_mean_mps',
    'body_angular_mean_rps',
    'left_right_difference_mps',
    'asymmetry_percent',
    'sample_file',
]


@dataclass(frozen=True)
class PlannedRun:
    run_id: str
    direction: str
    mode: str
    level_index: int
    repetition: int
    magnitude: float
    command_left: float
    command_right: float


def norm_to_dac(value: float) -> int:
    """Reproduce the ESP32 positive ``roundf(abs(value) * 255)`` rule."""
    magnitude = abs(float(value))
    if magnitude < 0.02:
        return 0
    magnitude = min(magnitude, 1.0)
    return int(math.floor(magnitude * 255.0 + 0.5))


def dac_to_nominal_voltage(dac_value: int, reference_voltage: float) -> float:
    return reference_voltage * float(dac_value) / 255.0


def parse_levels(text: str) -> List[float]:
    levels: List[float] = []
    for token in text.split(','):
        token = token.strip()
        if not token:
            continue
        value = float(token)
        if value < 0.02 or value > 1.0:
            raise argparse.ArgumentTypeError(
                'Each level must be in [0.02, 1.0].'
            )
        if value not in levels:
            levels.append(value)
    if not levels:
        raise argparse.ArgumentTypeError('At least one level is required.')
    return levels


def default_output_dir() -> Path:
    configured = os.environ.get('MODUBOT_CALIBRATION_DIR')
    if configured:
        return Path(configured).expanduser()
    docker_workspace = Path('/workspace/modubot_ws')
    if docker_workspace.is_dir():
        return docker_workspace / 'calibration_data'
    return Path.home() / 'calibracao_feedforward_modubot'


def command_pair(direction: str, mode: str, magnitude: float) -> Tuple[float, float]:
    signed = magnitude if direction == 'forward' else -magnitude
    if mode == 'both':
        return signed, signed
    if mode == 'left':
        return signed, 0.0
    if mode == 'right':
        return 0.0, signed
    raise ValueError(f'Unsupported mode: {mode}')


def build_plan(
    levels: Sequence[float],
    direction: str,
    mode: str,
    repetitions: int,
) -> List[PlannedRun]:
    directions = ['forward', 'reverse'] if direction == 'both' else [direction]
    plan: List[PlannedRun] = []
    for current_direction in directions:
        prefix = 'FWD' if current_direction == 'forward' else 'REV'
        for repetition in range(1, repetitions + 1):
            for level_index, magnitude in enumerate(levels, start=1):
                left, right = command_pair(current_direction, mode, magnitude)
                dac = norm_to_dac(magnitude)
                run_id = (
                    f'{prefix}_D{dac:03d}_N{int(round(magnitude * 1000)):04d}'
                    f'_R{repetition:02d}'
                )
                plan.append(
                    PlannedRun(
                        run_id=run_id,
                        direction=current_direction,
                        mode=mode,
                        level_index=level_index,
                        repetition=repetition,
                        magnitude=magnitude,
                        command_left=left,
                        command_right=right,
                    )
                )
    return plan


def parse_odom_line(line: str) -> Optional[Tuple[int, int, int]]:
    parts = line.strip().split()
    if len(parts) != 4 or parts[0] != 'O':
        return None
    try:
        delta_left = int(parts[1])
        delta_right = int(parts[2])
        dt_ms = int(parts[3])
    except ValueError:
        return None
    if dt_ms <= 0:
        return None
    return delta_left, delta_right, dt_ms


def wheel_speed(
    delta_ticks: int,
    dt_ms: int,
    ticks_per_revolution: float,
    wheel_radius: float,
) -> float:
    dt_s = dt_ms / 1000.0
    return (
        2.0
        * math.pi
        * wheel_radius
        * float(delta_ticks)
        / ticks_per_revolution
        / dt_s
    )


def safe_mean(values: Sequence[float]) -> Optional[float]:
    return statistics.fmean(values) if values else None


def descriptive_stats(values: Sequence[float]) -> Dict[str, Optional[float]]:
    if not values:
        return {
            'mean': None,
            'median': None,
            'std': None,
            'min': None,
            'max': None,
        }
    return {
        'mean': statistics.fmean(values),
        'median': statistics.median(values),
        'std': statistics.stdev(values) if len(values) > 1 else 0.0,
        'min': min(values),
        'max': max(values),
    }


def csv_value(value: object) -> object:
    if value is None:
        return ''
    if isinstance(value, float):
        return f'{value:.9f}'
    return value


class Esp32Serial:
    """Exclusive command and telemetry connection to the ModuBot ESP32."""

    def __init__(self, port: str, baud: int, startup_wait: float) -> None:
        if serial is None:
            raise RuntimeError(
                'pyserial is not installed. Install the python3-serial package.'
            )
        kwargs = {
            'port': port,
            'baudrate': baud,
            'timeout': 0.0,
            'write_timeout': 0.5,
        }
        if os.name == 'posix':
            kwargs['exclusive'] = True
        try:
            self.serial = serial.Serial(**kwargs)
        except TypeError:
            kwargs.pop('exclusive', None)
            self.serial = serial.Serial(**kwargs)
        self._rx_buffer = bytearray()
        time.sleep(startup_wait)
        self.serial.reset_input_buffer()
        self.stop()

    def write_line(self, line: str) -> None:
        self.serial.write((line + '\n').encode('ascii'))
        self.serial.flush()

    def command(self, left: float, right: float) -> None:
        self.write_line(f'V {left:.3f} {right:.3f}')

    def stop(self, repeats: int = 5) -> None:
        for _ in range(repeats):
            try:
                self.write_line('S')
            except SERIAL_EXCEPTIONS:
                break
            time.sleep(0.02)

    def read_lines(self) -> Iterable[str]:
        waiting = self.serial.in_waiting
        if waiting:
            self._rx_buffer.extend(self.serial.read(waiting))
        while b'\n' in self._rx_buffer:
            raw, _, remainder = self._rx_buffer.partition(b'\n')
            self._rx_buffer = bytearray(remainder)
            yield raw.decode('utf-8', errors='replace').strip()

    def clear_input(self) -> None:
        self._rx_buffer.clear()
        self.serial.reset_input_buffer()

    def wait_for_telemetry(self, timeout: float) -> Tuple[int, int, int]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for line in self.read_lines():
                parsed = parse_odom_line(line)
                if parsed is not None:
                    return parsed
            time.sleep(0.01)
        raise RuntimeError(
            'The serial port did not produce valid ESP32 odometry telemetry. '
            'Confirm that the selected device is the ModuBot ESP32.'
        )

    def close(self) -> None:
        self.stop()
        self.serial.close()


class CampaignRunner:
    def __init__(self, args: argparse.Namespace, output_dir: Path) -> None:
        self.args = args
        self.output_dir = output_dir
        self.samples_dir = output_dir / 'samples'
        self.samples_dir.mkdir(parents=True, exist_ok=False)
        self.summary_path = output_dir / 'summary.csv'
        self.summary_stream = self.summary_path.open(
            'w', newline='', encoding='utf-8'
        )
        self.summary_writer = csv.DictWriter(
            self.summary_stream, fieldnames=SUMMARY_FIELDS
        )
        self.summary_writer.writeheader()
        self.summary_stream.flush()
        self.link = Esp32Serial(args.port, args.baud, args.startup_wait)
        print('Waiting for ESP32 odometry telemetry before enabling commands...')
        try:
            first_telemetry = self.link.wait_for_telemetry(
                args.preflight_timeout
            )
        except RuntimeError:
            self.link.close()
            self.summary_stream.close()
            raise
        print(
            'ESP32 telemetry confirmed: '
            f'O {first_telemetry[0]} {first_telemetry[1]} '
            f'{first_telemetry[2]}'
        )

    def close(self) -> None:
        try:
            self.link.close()
        finally:
            self.summary_stream.close()

    def _phase_command(self, phase: str, run: PlannedRun) -> Tuple[float, float]:
        if phase == 'command':
            return run.command_left, run.command_right
        return 0.0, 0.0

    def _equivalent_cmd_vel(self, run: PlannedRun) -> Optional[float]:
        if run.mode != 'both':
            return None
        sign = 1.0 if run.direction == 'forward' else -1.0
        return sign * run.magnitude * self.args.v_wheel_max

    def execute(self, run: PlannedRun) -> Dict[str, object]:
        sample_path = self.samples_dir / f'{run.run_id}.csv'
        steady_left: List[float] = []
        steady_right: List[float] = []
        steady_linear: List[float] = []
        steady_angular: List[float] = []
        self.link.stop()
        self.link.clear_input()
        run_start = time.monotonic()
        last_odom: Optional[float] = None

        phases = [
            ('pre_stop', self.args.pre_time),
            ('command', self.args.command_time),
            ('post_stop', self.args.post_time),
        ]

        with sample_path.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=SAMPLE_FIELDS)
            writer.writeheader()

            try:
                for phase, duration in phases:
                    phase_start = time.monotonic()
                    next_send = phase_start
                    if phase != 'command':
                        self.link.stop()

                    while True:
                        now = time.monotonic()
                        elapsed_phase = now - phase_start
                        if elapsed_phase >= duration:
                            break

                        if now >= next_send:
                            if phase == 'command':
                                self.link.command(
                                    run.command_left, run.command_right
                                )
                            else:
                                self.link.write_line('S')
                            next_send = now + 1.0 / self.args.send_rate

                        for line in self.link.read_lines():
                            parsed = parse_odom_line(line)
                            if parsed is None:
                                continue
                            last_odom = time.monotonic()
                            delta_left, delta_right, dt_ms = parsed
                            speed_left = wheel_speed(
                                delta_left,
                                dt_ms,
                                self.args.ticks_left,
                                self.args.wheel_radius_left,
                            )
                            speed_right = wheel_speed(
                                delta_right,
                                dt_ms,
                                self.args.ticks_right,
                                self.args.wheel_radius_right,
                            )
                            body_linear = (speed_right + speed_left) / 2.0
                            body_angular = (
                                speed_right - speed_left
                            ) / self.args.wheel_separation
                            command_left, command_right = self._phase_command(
                                phase, run
                            )
                            dac_left = norm_to_dac(command_left)
                            dac_right = norm_to_dac(command_right)
                            row = {
                                'timestamp_utc': datetime.now(
                                    timezone.utc
                                ).isoformat(),
                                'run_id': run.run_id,
                                'direction': run.direction,
                                'mode': run.mode,
                                'level_index': run.level_index,
                                'repetition': run.repetition,
                                'phase': phase,
                                'elapsed_run_s': now - run_start,
                                'elapsed_phase_s': elapsed_phase,
                                'normalized_left': command_left,
                                'normalized_right': command_right,
                                'dac_left': dac_left,
                                'dac_right': dac_right,
                                'nominal_voltage_left_v': dac_to_nominal_voltage(
                                    dac_left, self.args.dac_reference_voltage
                                ),
                                'nominal_voltage_right_v': dac_to_nominal_voltage(
                                    dac_right, self.args.dac_reference_voltage
                                ),
                                'equivalent_cmd_vel_mps': (
                                    self._equivalent_cmd_vel(run)
                                    if phase == 'command'
                                    else 0.0
                                ),
                                'delta_ticks_left': delta_left,
                                'delta_ticks_right': delta_right,
                                'dt_ms': dt_ms,
                                'speed_left_mps': speed_left,
                                'speed_right_mps': speed_right,
                                'body_linear_mps': body_linear,
                                'body_angular_rps': body_angular,
                            }
                            writer.writerow(
                                {key: csv_value(value) for key, value in row.items()}
                            )

                            steady_end = (
                                self.args.command_time - self.args.steady_end_margin
                            )
                            if (
                                phase == 'command'
                                and elapsed_phase >= self.args.steady_start
                                and elapsed_phase <= steady_end
                            ):
                                steady_left.append(speed_left)
                                steady_right.append(speed_right)
                                steady_linear.append(body_linear)
                                steady_angular.append(body_angular)

                        if (
                            phase == 'command'
                            and elapsed_phase > self.args.odom_timeout
                            and (
                                last_odom is None
                                or now - last_odom > self.args.odom_timeout
                            )
                        ):
                            raise RuntimeError(
                                'No ESP32 odometry received within the configured '
                                'timeout. The robot was stopped.'
                            )
                        time.sleep(0.005)
            finally:
                self.link.stop()
                stream.flush()

        left_stats = descriptive_stats(steady_left)
        right_stats = descriptive_stats(steady_right)
        mean_left = left_stats['mean']
        mean_right = right_stats['mean']
        mean_linear = safe_mean(steady_linear)
        mean_angular = safe_mean(steady_angular)
        difference: Optional[float] = None
        asymmetry: Optional[float] = None
        if mean_left is not None and mean_right is not None:
            difference = abs(mean_right - mean_left)
            denominator = (abs(mean_right) + abs(mean_left)) / 2.0
            if denominator > 0.0:
                asymmetry = 100.0 * difference / denominator

        summary: Dict[str, object] = {
            'run_id': run.run_id,
            'status': 'completed' if steady_left else 'no_steady_samples',
            'direction': run.direction,
            'mode': run.mode,
            'level_index': run.level_index,
            'repetition': run.repetition,
            'normalized_left': run.command_left,
            'normalized_right': run.command_right,
            'dac_left': norm_to_dac(run.command_left),
            'dac_right': norm_to_dac(run.command_right),
            'nominal_voltage_left_v': dac_to_nominal_voltage(
                norm_to_dac(run.command_left), self.args.dac_reference_voltage
            ),
            'nominal_voltage_right_v': dac_to_nominal_voltage(
                norm_to_dac(run.command_right), self.args.dac_reference_voltage
            ),
            'equivalent_cmd_vel_mps': self._equivalent_cmd_vel(run),
            'command_time_s': self.args.command_time,
            'steady_start_s': self.args.steady_start,
            'steady_end_margin_s': self.args.steady_end_margin,
            'steady_samples': len(steady_left),
            'speed_left_mean_mps': left_stats['mean'],
            'speed_left_median_mps': left_stats['median'],
            'speed_left_std_mps': left_stats['std'],
            'speed_left_min_mps': left_stats['min'],
            'speed_left_max_mps': left_stats['max'],
            'speed_right_mean_mps': right_stats['mean'],
            'speed_right_median_mps': right_stats['median'],
            'speed_right_std_mps': right_stats['std'],
            'speed_right_min_mps': right_stats['min'],
            'speed_right_max_mps': right_stats['max'],
            'body_linear_mean_mps': mean_linear,
            'body_angular_mean_rps': mean_angular,
            'left_right_difference_mps': difference,
            'asymmetry_percent': asymmetry,
            'sample_file': str(sample_path.relative_to(self.output_dir)),
        }
        self.summary_writer.writerow(
            {key: csv_value(value) for key, value in summary.items()}
        )
        self.summary_stream.flush()
        return summary

    def record_skipped(self, run: PlannedRun) -> None:
        row: Dict[str, object] = {
            field: None for field in SUMMARY_FIELDS
        }
        row.update(
            {
                'run_id': run.run_id,
                'status': 'skipped_by_operator',
                'direction': run.direction,
                'mode': run.mode,
                'level_index': run.level_index,
                'repetition': run.repetition,
                'normalized_left': run.command_left,
                'normalized_right': run.command_right,
                'dac_left': norm_to_dac(run.command_left),
                'dac_right': norm_to_dac(run.command_right),
                'nominal_voltage_left_v': dac_to_nominal_voltage(
                    norm_to_dac(run.command_left),
                    self.args.dac_reference_voltage,
                ),
                'nominal_voltage_right_v': dac_to_nominal_voltage(
                    norm_to_dac(run.command_right),
                    self.args.dac_reference_voltage,
                ),
                'equivalent_cmd_vel_mps': self._equivalent_cmd_vel(run),
                'command_time_s': self.args.command_time,
                'steady_start_s': self.args.steady_start,
                'steady_end_margin_s': self.args.steady_end_margin,
                'steady_samples': 0,
            }
        )
        self.summary_writer.writerow(
            {key: csv_value(value) for key, value in row.items()}
        )
        self.summary_stream.flush()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            'Run interactive ModuBot ground calibration while recording every '
            'wheel-speed sample from the ESP32.'
        )
    )
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--levels', type=parse_levels, default=parse_levels(DEFAULT_LEVELS))
    parser.add_argument(
        '--direction', choices=['forward', 'reverse', 'both'], default='forward'
    )
    parser.add_argument('--mode', choices=['both', 'left', 'right'], default='both')
    parser.add_argument('--repetitions', type=int, default=3)
    parser.add_argument('--pre-time', type=float, default=2.0)
    parser.add_argument('--command-time', type=float, default=8.0)
    parser.add_argument('--post-time', type=float, default=2.0)
    parser.add_argument('--steady-start', type=float, default=3.0)
    parser.add_argument('--steady-end-margin', type=float, default=0.5)
    parser.add_argument('--send-rate', type=float, default=20.0)
    parser.add_argument('--odom-timeout', type=float, default=1.0)
    parser.add_argument('--preflight-timeout', type=float, default=3.0)
    parser.add_argument('--startup-wait', type=float, default=2.0)
    parser.add_argument('--ticks-left', type=float, default=91.0)
    parser.add_argument('--ticks-right', type=float, default=91.0)
    parser.add_argument('--wheel-radius-left', type=float, default=0.078)
    parser.add_argument('--wheel-radius-right', type=float, default=0.078)
    parser.add_argument('--wheel-separation', type=float, default=0.225)
    parser.add_argument('--v-wheel-max', type=float, default=0.6)
    parser.add_argument('--dac-reference-voltage', type=float, default=3.3)
    parser.add_argument('--surface', default='not_recorded')
    parser.add_argument('--robot-mass-kg', type=float)
    parser.add_argument('--battery-start-v', type=float)
    parser.add_argument('--notes', default='')
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=default_output_dir(),
    )
    parser.add_argument('--campaign-name', default='ground_calibration')
    parser.add_argument(
        '--dry-run', action='store_true', help='Print the plan without opening serial.'
    )
    return parser


def validate_args(args: argparse.Namespace) -> None:
    positive = {
        'baud': args.baud,
        'repetitions': args.repetitions,
        'command_time': args.command_time,
        'send_rate': args.send_rate,
        'odom_timeout': args.odom_timeout,
        'preflight_timeout': args.preflight_timeout,
        'ticks_left': args.ticks_left,
        'ticks_right': args.ticks_right,
        'wheel_radius_left': args.wheel_radius_left,
        'wheel_radius_right': args.wheel_radius_right,
        'wheel_separation': args.wheel_separation,
        'v_wheel_max': args.v_wheel_max,
        'dac_reference_voltage': args.dac_reference_voltage,
    }
    for name, value in positive.items():
        if value <= 0:
            raise ValueError(f'{name} must be greater than zero.')
    for name, value in {
        'robot_mass_kg': args.robot_mass_kg,
        'battery_start_v': args.battery_start_v,
    }.items():
        if value is not None and value <= 0:
            raise ValueError(f'{name} must be greater than zero when provided.')
    if args.pre_time < 0 or args.post_time < 0 or args.startup_wait < 0:
        raise ValueError('Pre, post, and startup times cannot be negative.')
    if args.steady_start < 0 or args.steady_end_margin < 0:
        raise ValueError('Steady-state window values cannot be negative.')
    if args.steady_start + args.steady_end_margin >= args.command_time:
        raise ValueError(
            'The steady-state window must fit inside the command duration.'
        )


def print_plan(plan: Sequence[PlannedRun], args: argparse.Namespace) -> None:
    print('\nPlanned calibration runs:')
    for index, run in enumerate(plan, start=1):
        dac = norm_to_dac(run.magnitude)
        voltage = dac_to_nominal_voltage(dac, args.dac_reference_voltage)
        print(
            f'  {index:02d}/{len(plan):02d} {run.run_id}: '
            f'L={run.command_left:+.3f}, R={run.command_right:+.3f}, '
            f'DAC={dac}, Vnom={voltage:.3f} V'
        )


def create_campaign_dir(args: argparse.Namespace) -> Path:
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    path = args.output_dir.expanduser() / f'{args.campaign_name}_{timestamp}'
    path.mkdir(parents=True, exist_ok=False)
    return path


def write_metadata(
    output_dir: Path,
    args: argparse.Namespace,
    plan: Sequence[PlannedRun],
) -> None:
    metadata = {
        'script_version': SCRIPT_VERSION,
        'created_at_utc': datetime.now(timezone.utc).isoformat(),
        'firmware_protocol': {
            'command': 'V <normalized_left> <normalized_right>',
            'stop': 'S',
            'telemetry': 'O <delta_ticks_left> <delta_ticks_right> <dt_ms>',
        },
        'notes': [
            'Nominal voltage is calculated from DAC/255*reference voltage; '
            'it is not a physical voltage measurement.',
            'Raw ticks and dt_ms are retained so speeds can be recomputed after '
            'ticks-per-revolution or effective radius are corrected.',
            'This process must be the only owner of the ESP32 serial port.',
        ],
        'arguments': {
            key: str(value) if isinstance(value, Path) else value
            for key, value in vars(args).items()
        },
        'plan': [asdict(run) for run in plan],
    }
    (output_dir / 'metadata.json').write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False),
        encoding='utf-8',
    )


def print_summary(summary: Dict[str, object]) -> None:
    print(f"\nCompleted {summary['run_id']} ({summary['steady_samples']} samples)")
    if summary['speed_left_mean_mps'] is None:
        print('  No steady-state samples were available.')
        return
    print(
        '  Left : '
        f"mean={float(summary['speed_left_mean_mps']):+.4f} m/s, "
        f"std={float(summary['speed_left_std_mps']):.4f} m/s"
    )
    print(
        '  Right: '
        f"mean={float(summary['speed_right_mean_mps']):+.4f} m/s, "
        f"std={float(summary['speed_right_std_mps']):.4f} m/s"
    )
    asymmetry = summary['asymmetry_percent']
    if asymmetry is not None:
        print(f'  Left/right asymmetry: {float(asymmetry):.2f}%')


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        validate_args(args)
    except ValueError as exc:
        parser.error(str(exc))

    plan = build_plan(args.levels, args.direction, args.mode, args.repetitions)
    print_plan(plan, args)
    if args.dry_run:
        return 0

    print(
        '\nSAFETY: clear the floor, keep a physical emergency stop accessible, '
        'and stop every other process that opens the ESP32 serial port.'
    )
    confirmation = input('Type INICIAR to open the serial port: ').strip().upper()
    if confirmation != 'INICIAR':
        print('Campaign cancelled.')
        return 1

    output_dir = create_campaign_dir(args)
    write_metadata(output_dir, args, plan)
    runner: Optional[CampaignRunner] = None
    try:
        runner = CampaignRunner(args, output_dir)
        for index, run in enumerate(plan, start=1):
            dac = norm_to_dac(run.magnitude)
            voltage = dac_to_nominal_voltage(
                dac, args.dac_reference_voltage
            )
            print(
                f'\n[{index}/{len(plan)}] Next: {run.run_id}\n'
                f'  direction={run.direction}, L={run.command_left:+.3f}, '
                f'R={run.command_right:+.3f}\n'
                f'  DAC={dac}, nominal voltage={voltage:.3f} V, '
                f'duration={args.command_time:.1f} s'
            )
            response = input(
                'Reposition the robot and clear the area. '
                'ENTER=run, s=skip, q=finish: '
            ).strip().lower()
            if response == 'q':
                break
            if response == 's':
                runner.record_skipped(run)
                continue
            summary = runner.execute(run)
            print_summary(summary)
    except KeyboardInterrupt:
        print('\nEmergency interruption received; sending brake command.')
        return_code = 130
    except CALIBRATION_EXCEPTIONS as exc:
        print(f'\nCalibration aborted safely: {exc}', file=sys.stderr)
        return_code = 2
    else:
        return_code = 0
    finally:
        if runner is not None:
            runner.close()
        print(f'Data directory: {output_dir}')
    return return_code


if __name__ == '__main__':
    raise SystemExit(main())
