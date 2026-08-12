"""Tests for the RoboDC face-state command."""

import argparse

import pytest

from modubot_serial_bridge.face_command import (
    build_face_state,
    parse_color,
)


def test_face_state_matches_the_rive_application_contract():
    state = build_face_state(
        expression='happy',
        direction='NE+',
        talking=True,
        blink=True,
        color='#12ABef',
        pause_look=True,
        pause_blink=False,
    )

    assert state == {
        'talking': True,
        'dir': 10,
        'blink': True,
        'exp': 2,
        'color': '#12abef',
        'pauseLook': True,
        'pauseBlink': False,
    }


def test_face_state_supports_all_documented_extremes():
    neutral = build_face_state('neutral', 'center')
    sleepy = build_face_state('sleepy', 'nw+')
    assert neutral['exp'] == 1
    assert neutral['dir'] == 0
    assert sleepy['exp'] == 8
    assert sleepy['dir'] == 16


def test_color_requires_six_digit_html_notation():
    assert parse_color('#FFFFFF') == '#ffffff'
    with pytest.raises(argparse.ArgumentTypeError):
        parse_color('white')
    with pytest.raises(argparse.ArgumentTypeError):
        parse_color('#fff')
