"""Shared persistence helpers for repeatable ModuBot experiments."""

from __future__ import annotations

import csv
import math
import statistics
from pathlib import Path
from typing import Dict, List, Optional, Sequence


def parse_battery_line(line: str) -> Optional[float]:
    """Parse one ``BAT:<volts>`` firmware line."""
    if not line.strip().startswith('BAT:'):
        return None
    try:
        voltage = float(line.split(':', 1)[1].strip())
    except (IndexError, ValueError):
        return None
    if not math.isfinite(voltage) or voltage <= 0.0:
        return None
    return voltage


def battery_statistics(values: Sequence[float]) -> Dict[str, object]:
    """Return stable battery fields used by experiment summary CSV files."""
    if not values:
        return {
            'battery_samples': 0,
            'battery_start_v': None,
            'battery_end_v': None,
            'battery_mean_v': None,
            'battery_min_v': None,
            'battery_max_v': None,
            'battery_drop_v': None,
        }
    return {
        'battery_samples': len(values),
        'battery_start_v': values[0],
        'battery_end_v': values[-1],
        'battery_mean_v': statistics.fmean(values),
        'battery_min_v': min(values),
        'battery_max_v': max(values),
        'battery_drop_v': values[0] - values[-1],
    }


def execution_run_id(planned_run_id: str, attempt: int) -> str:
    """Create a unique file-safe id while preserving the planned run id."""
    if attempt < 1:
        raise ValueError('attempt must be at least one.')
    if attempt == 1:
        return planned_run_id
    return f'{planned_run_id}_A{attempt:02d}'


def csv_value(value: object) -> object:
    """Serialize optional and floating-point values consistently."""
    if value is None:
        return ''
    if isinstance(value, float):
        return f'{value:.9f}'
    return value


class SummaryTable:
    """Small rewriteable CSV table supporting superseded experiment runs."""

    def __init__(self, path: Path, fieldnames: Sequence[str]) -> None:
        self.path = path
        self.fieldnames = list(fieldnames)
        self.rows: List[Dict[str, object]] = []
        self._write()

    def append(self, row: Dict[str, object]) -> None:
        self.rows.append(dict(row))
        self._write()

    def supersede(self, execution_id: str) -> bool:
        """Mark the selected completed attempt as replaced by a retry."""
        for row in reversed(self.rows):
            if (
                row.get('run_id') == execution_id
                and row.get('status') in {'completed', 'no_steady_samples'}
            ):
                row['status'] = 'repeated_by_operator'
                self._write()
                return True
        return False

    def _write(self) -> None:
        with self.path.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=self.fieldnames)
            writer.writeheader()
            for row in self.rows:
                writer.writerow(
                    {
                        key: csv_value(row.get(key))
                        for key in self.fieldnames
                    }
                )
