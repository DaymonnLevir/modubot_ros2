"""Launch the DC first-floor Gazebo simulation with the real Nav2 stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    gazebo_share = get_package_share_directory('modubot_gazebo')
    nav2_share = get_package_share_directory('modubot_nav2')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(
        nav2_share, 'maps', 'PisoInferiorDC.yaml')
    default_params = os.path.join(
        nav2_share, 'config', 'nav2_params.yaml')
    default_rviz = os.path.join(nav2_share, 'rviz', 'nav2.rviz')
    default_nav_bt = os.path.join(
        nav2_share,
        'behavior_trees',
        'navigate_to_pose_realtime.xml',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    configured_nav2_params = RewrittenYaml(
        source_file=LaunchConfiguration('nav2_params_file'),
        param_rewrites={
            'use_sim_time': use_sim_time,
            'default_nav_to_pose_bt_xml': default_nav_bt,
        },
        convert_types=True,
    )
    configured_parameter_file = ParameterFile(
        configured_nav2_params,
        allow_substs=True,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_gazebo_gui': LaunchConfiguration('use_gazebo_gui'),
            'paused': LaunchConfiguration('paused'),
            'world': LaunchConfiguration('world'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )

    scan_filter = Node(
        package='modubot_model_description',
        executable='simple_filter.py',
        name='modubot_filter',
        parameters=[{
            'use_sim_time': use_sim_time,
            'minimum_range': 0.15,
            'blocked_sector_enabled': True,
            'blocked_sector_center': 0.0,
            'blocked_sector_half_width': 0.261799,
        }],
        output='screen',
    )

    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        parameters=[configured_parameter_file],
        output='screen',
    )
    collision_monitor_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision_monitor',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['collision_monitor'],
        }],
        output='screen',
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_bringup_share,
                'launch',
                'bringup_launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'map': LaunchConfiguration('map'),
            'params_file': configured_nav2_params,
            'use_robot_state_pub': 'false',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', LaunchConfiguration('rviz_config'),
            '-f', 'map',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_gazebo_gui', default_value='true'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                gazebo_share, 'worlds', 'DC_FirstFloor.world'),
        ),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('nav2_params_file', default_value=default_params),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.10'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        gazebo,
        scan_filter,
        TimerAction(
            period=20.0,
            actions=[collision_monitor, collision_monitor_lifecycle],
        ),
        TimerAction(period=22.0, actions=[nav2]),
        TimerAction(period=30.0, actions=[rviz]),
    ])
