"""Unit tests for the feedforward calibration command-line behavior."""

import argparse
from unittest.mock import patch

from modubot_serial_bridge.feedforward_calibration import (
    Esp32Serial,
    build_parser,
    confirm_campaign_start,
    firmware_abort_reason,
    next_run_action,
    parse_control_mode_line,
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


def test_interactive_repeat_action_requires_a_previous_run():
    args = argparse.Namespace(automatic=False)
    with patch('builtins.input', return_value='R'):
        assert next_run_action(args, 'FWD_BOTH_U0050_R01') == 'repeat'
    with patch('builtins.input', return_value='R'):
        assert next_run_action(args) == 'run'


def test_control_mode_parser_accepts_only_known_status_lines():
    assert parse_control_mode_line('# modo=ABERTA kp=12.000') == 'ABERTA'
    assert parse_control_mode_line('# modo=FECHADA kp=12.000') == 'FECHADA'
    assert parse_control_mode_line('O 0 0 50') is None
    assert parse_control_mode_line('# modo=UNKNOWN') is None


def test_firmware_abort_reason_recognizes_rejected_commands_and_faults():
    assert firmware_abort_reason(
        '# comando incompatível com o modo de controle'
    ) == 'comando incompatível com o modo de controle'
    assert firmware_abort_reason(
        '# FALHA_FEEDBACK_L: DAC sem pulsos'
    ) == 'FALHA_FEEDBACK_L: DAC sem pulsos'
    assert firmware_abort_reason('# WATCHDOG: sem comando') is None


def test_open_loop_is_selected_and_confirmed_before_campaign():
    class FakeSerial:
        """Minimal serial transport that answers ``G`` with open-loop mode."""

        def __init__(self):
            self.buffer = bytearray()
            self.writes = []

        @property
        def in_waiting(self):
            return len(self.buffer)

        def write(self, payload):
            self.writes.append(payload)
            if payload == b'G\n':
                self.buffer.extend(b'# modo=ABERTA kp=12.000\n')

        def flush(self):
            pass

        def read(self, size):
            payload = bytes(self.buffer[:size])
            del self.buffer[:size]
            return payload

        def reset_input_buffer(self):
            self.buffer.clear()

    link = Esp32Serial.__new__(Esp32Serial)
    link.serial = FakeSerial()
    link._rx_buffer = bytearray()

    with patch(
        'modubot_serial_bridge.feedforward_calibration.time.sleep'
    ):
        status = link.ensure_open_loop(0.5)

    assert status.startswith('# modo=ABERTA')
    assert b'M 0\n' in link.serial.writes
    assert b'G\n' in link.serial.writes
