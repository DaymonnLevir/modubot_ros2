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
                    'port': '/dev/ttyUSB0',
                    'baud': 115200,
                    'ticks_per_rev_left': 91.0,   # AJUSTA AQUI
                    'ticks_per_rev_right': 91.0,  # AJUSTA AQUI
                    'wheel_radius': 0.078,         # AJUSTA AQUI
                    'wheel_separation': 0.225,     # AJUSTA AQUI
                    'frame_id': 'odom',
                    'child_frame_id': 'base_link',
                    'debug': False,
                }
            ]
        )
    ])

