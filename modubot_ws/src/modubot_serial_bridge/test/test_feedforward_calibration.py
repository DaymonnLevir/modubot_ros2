"""Unit tests for the feedforward calibration command-line behavior."""

import argparse
from unittest.mock import patch

from modubot_serial_bridge.feedforward_calibration import (
    build_parser,
    confirm_campaign_start,
    next_run_action,
)


def test_automatic_flag_defaults_to_false():
    args = build_parser().parse_args([])
    assert args.automatic is False


def test_automatic_mode_never_reads_operator_input():
    args = argparse.Namespace(automatic=True)
    with patch('builtins.input', side_effect=AssertionError('input called')):
        assert confirm_campaign_start(args) is True
        assert next_run_action(args) == 'run'


def test_interactive_confirmation_and_actions():
    args = argparse.Namespace(automatic=False)
    with patch('builtins.input', return_value='INICIAR'):
        assert confirm_campaign_start(args) is True
    with patch('builtins.input', return_value='s'):
        assert next_run_action(args) == 'skip'
    with patch('builtins.input', return_value='q'):
        assert next_run_action(args) == 'finish'
    with patch('builtins.input', return_value=''):
        assert next_run_action(args) == 'run'
