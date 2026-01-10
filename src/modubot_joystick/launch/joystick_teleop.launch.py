from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    #
    # Args opcionais (podem ser sobrescritos na linha de comando)
    #
    joy_dev_arg   = DeclareLaunchArgument('joy_dev',   default_value='/dev/input/js0')
    joy_deadzone  = DeclareLaunchArgument('joy_deadzone', default_value='0.05')
    joy_autorp    = DeclareLaunchArgument('joy_autorepeat', default_value='20.0')

    port_arg      = DeclareLaunchArgument('port',  default_value='/dev/ttyUSB0')
    baud_arg      = DeclareLaunchArgument('baud',  default_value='115200')
    vmax_arg      = DeclareLaunchArgument('v_wheel_max', default_value='0.6')
    dbg_arg       = DeclareLaunchArgument('debug', default_value='true')

    #
    # YAML do teleop (mantém seus mapeamentos/ganhos)
    #
    teleop_yaml = PathJoinSubstitution([
        FindPackageShare('modubot_joystick'),
        'config',
        'teleop.yaml'
    ])

    #
    # 1) joy_node (lê o controle)
    #
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': LaunchConfiguration('joy_dev'),
            'deadzone': LaunchConfiguration('joy_deadzone'),
            'autorepeat_rate': LaunchConfiguration('joy_autorepeat'),
        }]
    )

    #
    # 2) teleop_twist_joy (gera /modubot/cmd_vel a partir do joystick)
    #
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[teleop_yaml],
        remappings=[('/cmd_vel', '/modubot/cmd_vel')]
    )

    #
    # 3) bridge /modubot/cmd_vel -> serial (ESP32)
    #
    bridge_node = Node(
        package='modubot_teleop',
        executable='cmdvel_to_serial',
        name='cmdvel_to_serial',
        output='screen',
        parameters=[{
            'port':        LaunchConfiguration('port'),
            'baud':        LaunchConfiguration('baud'),
            'v_wheel_max': LaunchConfiguration('v_wheel_max'),
            'debug':       LaunchConfiguration('debug'),
        }],
        remappings=[('/cmd_vel', '/modubot/cmd_vel')]
    )

    return LaunchDescription([
        # argumentos
        joy_dev_arg, joy_deadzone, joy_autorp,
        port_arg, baud_arg, vmax_arg, dbg_arg,
        # nós
        joy_node,
        teleop_node,
        bridge_node,
    ])

