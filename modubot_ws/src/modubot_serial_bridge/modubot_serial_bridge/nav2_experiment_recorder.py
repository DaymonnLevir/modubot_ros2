#!/usr/bin/env python3
"""Record repeatable Nav2 missions with operator markers and battery data."""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

from .experiment_support import (
    SummaryTable,
    battery_statistics,
    execution_run_id,
)


SCRIPT_VERSION = '1.0.0'
DEFAULT_TOPICS = [
    '/experiment/marker',
    '/cmd_vel_nav',
    '/cmd_vel',
    '/cmd_vel_safe',
    '/odom',
    '/amcl_pose',
    '/particle_cloud',
    '/scan_filtered',
    '/map',
    '/map_updates',
    '/plan',
    '/local_plan',
    '/goal_pose',
    '/initialpose',
    '/tf',
    '/tf_static',
    '/diagnostics',
    '/modubot/serial_rx',
    '/battery/voltage',
    '/battery/percentage',
    '/battery_state',
    '/navigate_to_pose/_action/feedback',
    '/navigate_to_pose/_action/status',
    '/local_costmap/published_footprint',
    '/global_costmap/published_footprint',
    '/collision_monitor/emergency_stop_zone',
    '/collision_monitor/slowdown_zone',
]
SUMMARY_FIELDS = [
    'run_id',
    'planned_run_id',
    'attempt',
    'status',
    'scenario',
    'control_condition',
    'repetition',
    'started_at_utc',
    'stopped_at_utc',
    'duration_s',
    'battery_samples',
    'battery_start_v',
    'battery_end_v',
    'battery_mean_v',
    'battery_min_v',
    'battery_max_v',
    'battery_drop_v',
    'bag_directory',
    'operator_notes',
]


def default_output_dir() -> Path:
    configured = os.environ.get('MODUBOT_NAV2_EXPERIMENT_DIR')
    if configured:
        return Path(configured).expanduser()
    workspace = Path('/workspace/modubot_ws')
    if workspace.is_dir():
        return workspace / 'nav2_experiment_data'
    return Path.home() / 'modubot_nav2_experiment_data'


def parse_topics(value: str) -> List[str]:
    topics = [token.strip() for token in value.split(',') if token.strip()]
    if not topics:
        raise argparse.ArgumentTypeError('At least one topic is required.')
    return topics


class ExperimentMonitor(Node):
    """Keep the latest battery value and publish synchronization markers."""

    def __init__(self) -> None:
        super().__init__('nav2_experiment_recorder')
        self._lock = threading.Lock()
        self._recording = False
        self._battery_values: List[float] = []
        self.create_subscription(
            Float32, '/battery/voltage', self._on_battery, 20
        )
        self.marker_publisher = self.create_publisher(
            String, '/experiment/marker', 10
        )

    def _on_battery(self, message: Float32) -> None:
        with self._lock:
            if self._recording:
                self._battery_values.append(float(message.data))

    def start_run(self, run_id: str) -> None:
        with self._lock:
            self._battery_values = []
            self._recording = True
        self.publish_marker(f'START {run_id}')

    def stop_run(self, run_id: str, status: str) -> List[float]:
        self.publish_marker(f'STOP {run_id} {status}')
        with self._lock:
            self._recording = False
            return list(self._battery_values)

    def publish_marker(self, text: str) -> None:
        message = String()
        message.data = text
        self.marker_publisher.publish(message)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Record manual Nav2 experiment missions into rosbag2.'
    )
    parser.add_argument('--scenario', choices=['free', 'obstacles'], required=True)
    parser.add_argument(
        '--control-condition', choices=['baseline', 'pi'], required=True
    )
    parser.add_argument('--repetitions', type=int, default=10)
    parser.add_argument('--campaign-name', default='nav2_article')
    parser.add_argument('--output-dir', type=Path, default=default_output_dir())
    parser.add_argument(
        '--topics', type=parse_topics, default=list(DEFAULT_TOPICS)
    )
    parser.add_argument('--notes', default='')
    parser.add_argument('--topic-wait', type=float, default=5.0)
    parser.add_argument('--dry-run', action='store_true')
    return parser


def create_campaign_dir(args: argparse.Namespace) -> Path:
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    path = args.output_dir.expanduser() / (
        f'{args.campaign_name}_{args.scenario}_'
        f'{args.control_condition}_{stamp}'
    )
    path.mkdir(parents=True, exist_ok=False)
    (path / 'bags').mkdir()
    return path


def start_bag(path: Path, topics: Sequence[str]) -> subprocess.Popen:
    command = [
        'ros2',
        'bag',
        'record',
        '--include-hidden-topics',
        '-o',
        str(path),
        *topics,
    ]
    return subprocess.Popen(command)


def stop_bag(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5.0)


def missing_topics(node: Node, requested: Sequence[str]) -> List[str]:
    available = {name for name, _types in node.get_topic_names_and_types()}
    return [topic for topic in requested if topic not in available]


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.repetitions <= 0:
        parser.error('repetitions must be greater than zero.')
    if args.topic_wait <= 0.0:
        parser.error('topic-wait must be greater than zero.')

    planned_ids = [
        f'NAV2_{args.scenario.upper()}_{args.control_condition.upper()}_'
        f'R{repetition:02d}'
        for repetition in range(1, args.repetitions + 1)
    ]
    print('\nPlanned missions:')
    for run_id in planned_ids:
        print(f'  {run_id}')
    print('\nTopics recorded:')
    for topic in args.topics:
        print(f'  {topic}')
    if args.dry_run:
        return 0

    answer = input(
        'Confirm that Nav2 is active, localization is valid, and the arena is '
        'clear. Type INICIAR: '
    ).strip().upper()
    if answer != 'INICIAR':
        return 1

    output_dir = create_campaign_dir(args)
    summary = SummaryTable(output_dir / 'summary.csv', SUMMARY_FIELDS)
    metadata = {
        'script_version': SCRIPT_VERSION,
        'created_at_utc': datetime.now(timezone.utc).isoformat(),
        'arguments': {
            key: str(value) if isinstance(value, Path) else value
            for key, value in vars(args).items()
        },
        'planned_run_ids': planned_ids,
        'method': (
            'The operator sets the initial pose and sends the fixed Nav2 goal. '
            'Each mission is stored in an independent rosbag with synchronized '
            'START/STOP markers.'
        ),
    }
    (output_dir / 'metadata.json').write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False), encoding='utf-8'
    )

    rclpy.init(args=None)
    monitor = ExperimentMonitor()
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(monitor,), daemon=True
    )
    spin_thread.start()
    deadline = time.monotonic() + args.topic_wait
    while time.monotonic() < deadline:
        if '/battery/voltage' not in missing_topics(
            monitor, ['/battery/voltage']
        ):
            break
        time.sleep(0.1)
    absent = missing_topics(monitor, args.topics)
    if '/battery/voltage' in absent:
        print(
            'ERROR: /battery/voltage is unavailable. Start the base with the '
            'battery monitor before recording.',
            file=sys.stderr,
        )
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        print(f'Data directory: {output_dir}')
        return 2
    if absent:
        print('\nWARNING: these requested topics are not currently available:')
        for topic in absent:
            print(f'  {topic}')
        print('The bag will still start and record them if they appear later.')

    return_code = 0
    active_bag: Optional[subprocess.Popen] = None
    try:
        repetition = 1
        attempts: Dict[str, int] = {}
        while repetition <= args.repetitions:
            planned_id = planned_ids[repetition - 1]
            attempt = attempts.get(planned_id, 0) + 1
            attempts[planned_id] = attempt
            run_id = execution_run_id(planned_id, attempt)
            action = input(
                f'Position the robot for {run_id}, set 2D Pose Estimate, and '
                'prepare the fixed goal. ENTER=start bag, q=finish: '
            ).strip().lower()
            if action == 'q':
                break

            bag_path = output_dir / 'bags' / run_id
            bag_process = start_bag(bag_path, args.topics)
            active_bag = bag_process
            time.sleep(2.0)
            if bag_process.poll() is not None:
                raise RuntimeError(
                    f'ros2 bag record exited before {run_id} started.'
                )
            started_utc = datetime.now(timezone.utc).isoformat()
            started = time.monotonic()
            monitor.start_run(run_id)
            print(
                f'RECORDING {run_id}. Send the Nav2 goal now. '
                'Keep this terminal available.'
            )
            result = input(
                'After the mission: ENTER=success, f=failed, '
                'r=preserve and repeat, q=stop campaign: '
            ).strip().lower()
            status = {
                'f': 'failed',
                'r': 'repeated_by_operator',
                'q': 'aborted_by_operator',
            }.get(result, 'completed')
            battery_values = monitor.stop_run(run_id, status)
            time.sleep(0.1)
            stopped_utc = datetime.now(timezone.utc).isoformat()
            duration = time.monotonic() - started
            stop_bag(bag_process)
            active_bag = None
            note = input('Optional note for this mission (ENTER=none): ').strip()
            summary.append(
                {
                    'run_id': run_id,
                    'planned_run_id': planned_id,
                    'attempt': attempt,
                    'status': status,
                    'scenario': args.scenario,
                    'control_condition': args.control_condition,
                    'repetition': repetition,
                    'started_at_utc': started_utc,
                    'stopped_at_utc': stopped_utc,
                    'duration_s': duration,
                    **battery_statistics(battery_values),
                    'bag_directory': str(bag_path.relative_to(output_dir)),
                    'operator_notes': note,
                }
            )
            if result == 'r':
                continue
            if result == 'q':
                break
            repetition += 1
    except KeyboardInterrupt:
        print('\nRecorder interrupted.')
        return_code = 130
    except (OSError, RuntimeError) as exc:
        print(f'Recorder aborted: {exc}', file=sys.stderr)
        return_code = 2
    finally:
        if active_bag is not None:
            stop_bag(active_bag)
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        print(f'Data directory: {output_dir}')
    return return_code


if __name__ == '__main__':
    raise SystemExit(main())
