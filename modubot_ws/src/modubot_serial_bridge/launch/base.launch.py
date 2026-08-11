from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare('modubot_serial_bridge'),
        'config',
        'modubot_params.yaml',
    ])

    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')
    closed_loop = LaunchConfiguration('closed_loop')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    params_file = LaunchConfiguration('params_file')

    bridge = Node(
        package='modubot_serial_bridge',
        executable='cmdvel_to_serial',
        name='cmdvel_to_serial',
        output='screen',
        respawn=True,
        respawn_delay=1.0,
        parameters=[
            params_file,
            {
                'port': port,
                'baud': ParameterValue(baud, value_type=int),
                'closed_loop': ParameterValue(closed_loop, value_type=bool),
                'cmd_vel_topic': cmd_vel_topic,
            },
        ],
    )

    odometry = Node(
        package='modubot_odom',
        executable='serial_odom',
        name='serial_odom',
        output='screen',
        respawn=True,
        respawn_delay=1.0,
        parameters=[params_file],
    )

    battery_monitor = Node(
        package='modubot_battery_monitor',
        executable='battery_monitor',
        name='battery_monitor',
        output='screen',
        respawn=True,
        respawn_delay=1.0,
        parameters=[params_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('closed_loop', default_value='true'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        bridge,
        odometry,
        battery_monitor,
    ])
