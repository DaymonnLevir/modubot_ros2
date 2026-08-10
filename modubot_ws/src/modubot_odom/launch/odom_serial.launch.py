from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='modubot_odom',
            executable='serial_odom',
            name='serial_odom',
            output='screen',
            parameters=[
                {
                    'serial_rx_topic': '/modubot/serial_rx',
                    'ticks_per_rev_left': 91.0,
                    'ticks_per_rev_right': 91.0,
                    'wheel_radius': 0.078,
                    'wheel_separation': 0.207,
                    'frame_id': 'odom',
                    'child_frame_id': 'base_link',
                    'debug': False,
                }
            ]
        )
    ])
