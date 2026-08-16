import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gazebo_share = get_package_share_directory('modubot_gazebo')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    model_share = get_package_share_directory('modubot_model_description')

    default_world = os.path.join(
        gazebo_share, 'worlds', 'DC_FirstFloor.world')
    xacro_file = os.path.join(
        gazebo_share, 'urdf', 'modubot_sim.xacro')
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' mesh_prefix:=',
            'file://' + os.path.join(model_share, 'meshes'),
            ' cmd_vel_topic:=', LaunchConfiguration('cmd_vel_topic'),
        ]),
        value_type=str,
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'pause': LaunchConfiguration('paused'),
        }.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(use_gazebo_gui),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'modubot',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
            '-timeout', '60',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_gazebo_gui', default_value='true'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='/cmd_vel_safe'),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.10'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_robot,
    ])
