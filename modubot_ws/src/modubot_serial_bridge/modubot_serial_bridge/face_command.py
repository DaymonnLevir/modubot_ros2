#!/usr/bin/env python3
"""Publish a validated RoboDC face state through ROS 2."""

from __future__ import annotations

import argparse
import json
import re
import time
from typing import Dict, Optional, Sequence

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


EXPRESSIONS = {
    'neutral': 1,
    'happy': 2,
    'sad': 3,
    'scared': 4,
    'surprised': 5,
    'angry': 6,
    'disgusted': 7,
    'sleepy': 8,
}

DIRECTIONS = {
    'center': 0,
    'n': 1,
    'ne': 2,
    'e': 3,
    'se': 4,
    's': 5,
    'sw': 6,
    'w': 7,
    'nw': 8,
    'n+': 9,
    'ne+': 10,
    'e+': 11,
    'se+': 12,
    's+': 13,
    'sw+': 14,
    'w+': 15,
    'nw+': 16,
}

COLOR_PATTERN = re.compile(r'^#[0-9a-fA-F]{6}$')


def parse_color(value: str) -> str:
    """Validate and normalize a six-digit HTML color."""
    if not COLOR_PATTERN.fullmatch(value):
        raise argparse.ArgumentTypeError(
            'color must use the #RRGGBB format.'
        )
    return value.lower()


def build_face_state(
    expression: str,
    direction: str,
    talking: bool = False,
    blink: bool = False,
    color: str = '#ffffff',
    pause_look: bool = False,
    pause_blink: bool = False,
) -> Dict[str, object]:
    """Build the JSON object consumed by the Rive face application."""
    return {
        'talking': talking,
        'dir': DIRECTIONS[direction.lower()],
        'blink': blink,
        'exp': EXPRESSIONS[expression.lower()],
        'color': parse_color(color),
        'pauseLook': pause_look,
        'pauseBlink': pause_blink,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Publish one validated state to the RoboDC face.'
    )
    parser.add_argument(
        '--expression', choices=EXPRESSIONS, default='neutral'
    )
    parser.add_argument(
        '--direction', choices=DIRECTIONS, default='center'
    )
    parser.add_argument('--talking', action='store_true')
    parser.add_argument('--blink', action='store_true')
    parser.add_argument('--color', type=parse_color, default='#ffffff')
    parser.add_argument('--pause-look', action='store_true')
    parser.add_argument('--pause-blink', action='store_true')
    parser.add_argument('--topic', default='/robot/face_state')
    parser.add_argument('--repetitions', type=int, default=3)
    parser.add_argument('--interval', type=float, default=0.2)
    parser.add_argument('--subscriber-wait', type=float, default=3.0)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.repetitions <= 0:
        raise SystemExit('--repetitions must be greater than zero.')
    if args.interval < 0.0 or args.subscriber_wait < 0.0:
        raise SystemExit('--interval and --subscriber-wait cannot be negative.')

    state = build_face_state(
        expression=args.expression,
        direction=args.direction,
        talking=args.talking,
        blink=args.blink,
        color=args.color,
        pause_look=args.pause_look,
        pause_blink=args.pause_blink,
    )
    payload = json.dumps(state, separators=(',', ':'))

    rclpy.init(args=None)
    node = Node('modubot_face_command')
    publisher = node.create_publisher(String, args.topic, 10)
    try:
        deadline = time.monotonic() + args.subscriber_wait
        while (
            publisher.get_subscription_count() == 0
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.05)

        subscriber_count = publisher.get_subscription_count()
        if subscriber_count == 0:
            node.get_logger().warning(
                'No face subscriber detected; publishing anyway.'
            )

        message = String(data=payload)
        for index in range(args.repetitions):
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.05)
            if index + 1 < args.repetitions and args.interval > 0.0:
                time.sleep(args.interval)
        node.get_logger().info(
            f'Published to {args.topic}: {payload}'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
