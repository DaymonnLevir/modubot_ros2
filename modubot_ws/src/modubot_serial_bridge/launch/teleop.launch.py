from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#test  

def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    baudrate    = LaunchConfiguration('baudrate')
    cmd_vel     = LaunchConfiguration('cmd_vel_topic')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),

        # Teclado -> /cmd_vel
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'speed': 0.10,   # m/s
                'turn': 0.50,    # rad/s
            }],
        ),

        # /cmd_vel -> Serial (ESP32)
        Node(
            package='modubot_teleop',
            executable='cmdvel_to_serial',
            name='cmdvel_to_serial',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'serial_port': serial_port,
                'baudrate': baudrate,
                'cmd_vel_topic': cmd_vel,
            }],
        ),
    ])
