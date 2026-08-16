"""Run an odometry-stopped trajectory experiment in Gazebo Classic."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    gazebo_share = get_package_share_directory('modubot_gazebo')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'use_gazebo_gui': LaunchConfiguration('use_gazebo_gui'),
            'paused': 'false',
            'world': LaunchConfiguration('world'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )

    experiment = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'modubot_serial_bridge',
            'odometry_trajectory_experiment',
            '--backend',
            'ros',
            '--cmd-vel-topic',
            '/cmd_vel_safe',
            '--odom-topic',
            '/odom',
            '--trajectories',
            LaunchConfiguration('trajectory'),
            '--repetitions',
            LaunchConfiguration('repetitions'),
            '--distance',
            LaunchConfiguration('distance'),
            '--arc-length',
            LaunchConfiguration('arc_length'),
            '--arc-radius',
            LaunchConfiguration('arc_radius'),
            '--figure-eight-radius',
            LaunchConfiguration('figure_eight_radius'),
            '--figure-eight-cycles',
            LaunchConfiguration('figure_eight_cycles'),
            '--figure-eight-start-direction',
            LaunchConfiguration('figure_eight_start_direction'),
            '--linear-speed',
            LaunchConfiguration('linear_speed'),
            '--rotation-angle-deg',
            LaunchConfiguration('rotation_angle_deg'),
            '--angular-speed',
            LaunchConfiguration('angular_speed'),
            '--preflight-timeout',
            LaunchConfiguration('preflight_timeout'),
            '--max-duration',
            LaunchConfiguration('max_duration'),
            '--pre-time',
            LaunchConfiguration('pre_time'),
            '--post-time',
            LaunchConfiguration('post_time'),
            '--countdown',
            LaunchConfiguration('countdown'),
            '--inter-run-wait',
            '0',
            '--surface',
            'gazebo',
            '--output-dir',
            LaunchConfiguration('output_dir'),
            '--campaign-name',
            LaunchConfiguration('campaign_name'),
            '--automatic',
        ],
        output='screen',
        additional_env={'PYTHONUNBUFFERED': '1'},
    )

    stop_launch_when_done = RegisterEventHandler(
        OnProcessExit(
            target_action=experiment,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason='Simulation trajectory experiment completed.'
                    )
                )
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('trajectory', default_value='figure_eight'),
        DeclareLaunchArgument('repetitions', default_value='1'),
        DeclareLaunchArgument('distance', default_value='1.5'),
        DeclareLaunchArgument('arc_length', default_value='0.8'),
        DeclareLaunchArgument('arc_radius', default_value='0.8'),
        DeclareLaunchArgument('figure_eight_radius', default_value='0.35'),
        DeclareLaunchArgument('figure_eight_cycles', default_value='1'),
        DeclareLaunchArgument(
            'figure_eight_start_direction', default_value='left'
        ),
        DeclareLaunchArgument('linear_speed', default_value='0.15'),
        DeclareLaunchArgument('rotation_angle_deg', default_value='90.0'),
        DeclareLaunchArgument('angular_speed', default_value='0.4'),
        DeclareLaunchArgument('preflight_timeout', default_value='120.0'),
        DeclareLaunchArgument('max_duration', default_value='120.0'),
        DeclareLaunchArgument('pre_time', default_value='1.0'),
        DeclareLaunchArgument('post_time', default_value='1.0'),
        DeclareLaunchArgument('countdown', default_value='3'),
        DeclareLaunchArgument(
            'output_dir',
            default_value=os.path.expanduser('~/modubot_trajectory_data'),
        ),
        DeclareLaunchArgument(
            'campaign_name', default_value='sim_figure_eight'
        ),
        DeclareLaunchArgument('use_gazebo_gui', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                gazebo_share, 'worlds', 'DC_FirstFloor.world'
            ),
        ),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.10'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        gazebo,
        experiment,
        stop_launch_when_done,
    ])
