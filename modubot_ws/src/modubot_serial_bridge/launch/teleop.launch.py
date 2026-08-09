from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    baudrate    = LaunchConfiguration('baudrate')
    start_base = LaunchConfiguration('start_base')
    closed_loop = LaunchConfiguration('closed_loop')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('start_base', default_value='true'),
        DeclareLaunchArgument('closed_loop', default_value='true'),

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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('modubot_serial_bridge'),
                'launch',
                'base.launch.py',
            ])),
            condition=IfCondition(start_base),
            launch_arguments={
                'port': serial_port,
                'baud': baudrate,
                'closed_loop': closed_loop,
            }.items(),
        ),
    ])
