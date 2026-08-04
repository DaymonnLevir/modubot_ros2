#!/usr/bin/env python3
"""Aggregate ModuBot calibration CSV files and generate four map plots."""

from __future__ import annotations

import argparse
import csv
import statistics
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from typing import DefaultDict, Dict, Iterable, List, Sequence, Tuple


MAP_FIELDS = [
    'direction',
    'wheel',
    'normalized_command',
    'command_magnitude',
    'dac',
    'nominal_voltage_v',
    'repetitions',
    'speed_mean_mps',
    'speed_magnitude_mean_mps',
    'speed_between_run_std_mps',
]

INVERSE_FIELDS = [
    'direction',
    'wheel',
    'target_speed_mps',
    'target_speed_magnitude_mps',
    'normalized_command',
    'dac',
    'nominal_voltage_v',
]


def read_csv(path: Path) -> List[Dict[str, str]]:
    with path.open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def float_value(row: Dict[str, str], key: str) -> float:
    return float(row[key])


def aggregate_summary(rows: Iterable[Dict[str, str]]) -> List[Dict[str, object]]:
    groups: DefaultDict[
        Tuple[str, str, float, int, float], List[float]
    ] = defaultdict(list)

    for row in rows:
        if row.get('status') != 'completed' or row.get('mode') != 'both':
            continue
        direction = row['direction']
        for wheel in ('left', 'right'):
            command = float_value(row, f'normalized_{wheel}')
            speed = float_value(row, f'speed_{wheel}_mean_mps')
            dac = int(row[f'dac_{wheel}'])
            voltage = float_value(row, f'nominal_voltage_{wheel}_v')
            groups[(direction, wheel, command, dac, voltage)].append(speed)

    output: List[Dict[str, object]] = []
    for (direction, wheel, command, dac, voltage), speeds in groups.items():
        mean_speed = statistics.fmean(speeds)
        output.append(
            {
                'direction': direction,
                'wheel': wheel,
                'normalized_command': command,
                'command_magnitude': abs(command),
                'dac': dac,
                'nominal_voltage_v': voltage,
                'repetitions': len(speeds),
                'speed_mean_mps': mean_speed,
                'speed_magnitude_mean_mps': abs(mean_speed),
                'speed_between_run_std_mps': (
                    statistics.stdev(speeds) if len(speeds) > 1 else 0.0
                ),
            }
        )
    output.sort(
        key=lambda row: (
            str(row['direction']),
            str(row['wheel']),
            float(row['command_magnitude']),
        )
    )
    return output


def write_rows(path: Path, fields: Sequence[str], rows: Iterable[Dict[str, object]]) -> None:
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def inverse_seed_rows(map_rows: Iterable[Dict[str, object]]) -> List[Dict[str, object]]:
    rows = [
        {
            'direction': row['direction'],
            'wheel': row['wheel'],
            'target_speed_mps': row['speed_mean_mps'],
            'target_speed_magnitude_mps': row['speed_magnitude_mean_mps'],
            'normalized_command': row['normalized_command'],
            'dac': row['dac'],
            'nominal_voltage_v': row['nominal_voltage_v'],
        }
        for row in map_rows
    ]
    rows.sort(
        key=lambda row: (
            str(row['direction']),
            str(row['wheel']),
            float(row['target_speed_magnitude_mps']),
        )
    )
    return rows


def import_matplotlib():
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return None
    return plt


def plot_maps(map_rows: Sequence[Dict[str, object]], output_dir: Path) -> bool:
    plt = import_matplotlib()
    if plt is None:
        return False

    styles = {
        ('forward', 'left'): ('L+', 'tab:blue', 'o'),
        ('forward', 'right'): ('R+', 'tab:orange', 's'),
        ('reverse', 'left'): ('L-', 'tab:green', '^'),
        ('reverse', 'right'): ('R-', 'tab:red', 'D'),
    }
    fig, axis = plt.subplots(figsize=(7.2, 4.6))
    for key, (label, color, marker) in styles.items():
        selected = [
            row
            for row in map_rows
            if (row['direction'], row['wheel']) == key
        ]
        if not selected:
            continue
        selected.sort(key=lambda row: float(row['command_magnitude']))
        x_values = [float(row['command_magnitude']) for row in selected]
        y_values = [float(row['speed_magnitude_mean_mps']) for row in selected]
        errors = [float(row['speed_between_run_std_mps']) for row in selected]
        axis.errorbar(
            x_values,
            y_values,
            yerr=errors,
            label=label,
            color=color,
            marker=marker,
            linewidth=1.4,
            capsize=3,
        )
    axis.set_xlabel('Normalized command magnitude')
    axis.set_ylabel('Steady wheel-speed magnitude (m/s)')
    axis.set_title('ModuBot direction-dependent actuator maps')
    axis.grid(True, alpha=0.3)
    axis.legend()
    fig.tight_layout()
    fig.savefig(output_dir / 'command_to_speed_four_maps.png', dpi=180)
    plt.close(fig)
    return True


def plot_time_series(campaign_dirs: Sequence[Path], output_dir: Path) -> int:
    plt = import_matplotlib()
    if plt is None:
        return 0
    timeseries_dir = output_dir / 'time_series'
    timeseries_dir.mkdir(exist_ok=True)
    count = 0
    for campaign_dir in campaign_dirs:
        for sample_path in sorted((campaign_dir / 'samples').glob('*.csv')):
            rows = read_csv(sample_path)
            if not rows:
                continue
            x_values = [float_value(row, 'elapsed_run_s') for row in rows]
            left = [float_value(row, 'speed_left_mps') for row in rows]
            right = [float_value(row, 'speed_right_mps') for row in rows]
            fig, axis = plt.subplots(figsize=(7.2, 3.8))
            axis.plot(x_values, left, label='Left wheel', linewidth=1.2)
            axis.plot(x_values, right, label='Right wheel', linewidth=1.2)
            axis.set_xlabel('Elapsed run time (s)')
            axis.set_ylabel('Wheel speed (m/s)')
            axis.set_title(sample_path.stem)
            axis.grid(True, alpha=0.3)
            axis.legend()
            fig.tight_layout()
            filename = f'{campaign_dir.name}__{sample_path.stem}.png'
            fig.savefig(timeseries_dir / filename, dpi=160)
            plt.close(fig)
            count += 1
    return count


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Aggregate a ModuBot feedforward-calibration campaign.'
    )
    parser.add_argument(
        '--campaign',
        type=Path,
        nargs='+',
        required=True,
        help='One campaign, or forward and reverse campaign directories.',
    )
    parser.add_argument('--output-dir', type=Path)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    campaign_dirs = [path.expanduser().resolve() for path in args.campaign]
    summary_rows: List[Dict[str, str]] = []
    for campaign_dir in campaign_dirs:
        summary_path = campaign_dir / 'summary.csv'
        if not summary_path.is_file():
            raise SystemExit(f'Summary not found: {summary_path}')
        summary_rows.extend(read_csv(summary_path))

    map_rows = aggregate_summary(summary_rows)
    if not map_rows:
        raise SystemExit(
            'No completed both-wheel runs were found in summary.csv.'
        )

    if args.output_dir is not None:
        output_dir = args.output_dir.expanduser().resolve()
    elif len(campaign_dirs) == 1:
        output_dir = campaign_dirs[0] / 'maps'
    else:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_dir = campaign_dirs[0].parent / f'combined_maps_{timestamp}'
    output_dir.mkdir(parents=True, exist_ok=True)
    write_rows(output_dir / 'map_points.csv', MAP_FIELDS, map_rows)
    write_rows(
        output_dir / 'inverse_map_seed.csv',
        INVERSE_FIELDS,
        inverse_seed_rows(map_rows),
    )
    map_plot_created = plot_maps(map_rows, output_dir)
    time_series_count = plot_time_series(campaign_dirs, output_dir)

    print(f'Map points: {output_dir / "map_points.csv"}')
    print(f'Inverse-map seed: {output_dir / "inverse_map_seed.csv"}')
    if map_plot_created:
        print(f'Four-map plot: {output_dir / "command_to_speed_four_maps.png"}')
        print(f'Time-series plots created: {time_series_count}')
    else:
        print('matplotlib is not installed; CSV outputs were still generated.')
    print(
        'Review repeated-run dispersion and monotonicity before using the '
        'inverse-map seed for interpolation.'
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
