"""Unit tests for experiment persistence and battery parsing."""

import csv

import pytest

from modubot_serial_bridge.experiment_support import (
    SummaryTable,
    battery_statistics,
    execution_run_id,
    parse_battery_line,
)


def test_battery_line_and_statistics():
    assert parse_battery_line('BAT:23.75') == pytest.approx(23.75)
    assert parse_battery_line('O 1 1 50') is None
    assert parse_battery_line('BAT:nan') is None
    result = battery_statistics([24.0, 23.8, 23.9])
    assert result['battery_samples'] == 3
    assert result['battery_mean_v'] == pytest.approx(23.9)
    assert result['battery_drop_v'] == pytest.approx(0.1)


def test_retry_id_and_summary_supersede(tmp_path):
    path = tmp_path / 'summary.csv'
    fields = ['run_id', 'status']
    table = SummaryTable(path, fields)
    table.append({'run_id': 'RUN_R01', 'status': 'completed'})
    assert table.supersede('RUN_R01') is True
    table.append({'run_id': 'RUN_R01_A02', 'status': 'completed'})

    assert execution_run_id('RUN_R01', 1) == 'RUN_R01'
    assert execution_run_id('RUN_R01', 2) == 'RUN_R01_A02'
    with path.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    assert rows[0]['status'] == 'repeated_by_operator'
    assert rows[1]['status'] == 'completed'
