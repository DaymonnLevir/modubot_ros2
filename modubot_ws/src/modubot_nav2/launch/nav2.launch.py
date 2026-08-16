import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    UnsetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
import xacro


def generate_launch_description():
    # 1. Caminhos e Configurações
    pkg_description = 'modubot_model_description'
    pkg_nav2 = 'modubot_nav2'

    # Processamento do Xacro
    xacro_file = os.path.join(
        get_package_share_directory(pkg_description),
        'urdf/modubot_model.xacro',
    )
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Configurações do Nav2
    nav2_share = get_package_share_directory(pkg_nav2)
    default_params = os.path.join(nav2_share, "config", "nav2_params.yaml")
    default_base_params = os.path.join(
        get_package_share_directory("modubot_serial_bridge"),
        "config",
        "modubot_params.yaml",
    )
    default_map = os.path.join(nav2_share, "maps", "PisoInferiorDC.yaml")
    default_rviz = os.path.join(nav2_share, "rviz", "nav2.rviz")
    default_nav_bt = os.path.join(
        nav2_share,
        "behavior_trees",
        "navigate_to_pose_realtime.xml",
    )
    default_nav_through_poses_bt = os.path.join(
        nav2_share,
        "behavior_trees",
        "navigate_through_poses_realtime.xml",
    )
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    closed_loop = LaunchConfiguration("closed_loop", default="true")
    use_rviz = LaunchConfiguration("use_rviz")
    configured_nav2_params = RewrittenYaml(
        source_file=LaunchConfiguration("nav2_params_file"),
        param_rewrites={
            "default_nav_to_pose_bt_xml": default_nav_bt,
            "default_nav_through_poses_bt_xml": (
                default_nav_through_poses_bt
            ),
        },
        convert_types=True,
    )
    configured_nav2_parameter_file = ParameterFile(
        configured_nav2_params,
        allow_substs=True,
    )

    # --- A EQUIPE DE LANÇAMENTO ---

    # A. O Lidar (Atenção à porta USB)
    node_rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('lidar_port'),
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
        output='screen',
        respawn=True,
        respawn_delay=5.0,
    )

    # B. Base móvel: uma única ponte serial + odometria
    launch_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('modubot_serial_bridge'),
                'launch',
                'base.launch.py',
            )
        ),
        launch_arguments={
            'port': LaunchConfiguration('base_port'),
            'closed_loop': closed_loop,
            'cmd_vel_topic': '/cmd_vel_safe',
            'params_file': LaunchConfiguration('base_params_file'),
        }.items(),
    )

    # C. A "Alma" do Robô (Estado e Juntas)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': use_sim_time,
        }]
    )

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # D. Filtro do setor traseiro bloqueado pela estrutura
    node_script_filter = Node(
        package=pkg_description,
        executable='simple_filter.py',
        name='modubot_filter',
        parameters=[{
            'use_sim_time': use_sim_time,
            'minimum_range': ParameterValue(
                LaunchConfiguration('lidar_minimum_range'),
                value_type=float,
            ),
            'blocked_sector_enabled': ParameterValue(
                LaunchConfiguration('lidar_blocked_sector_enabled'),
                value_type=bool,
            ),
            'blocked_sector_center': ParameterValue(
                LaunchConfiguration('lidar_blocked_sector_center'),
                value_type=float,
            ),
            'blocked_sector_half_width': ParameterValue(
                LaunchConfiguration('lidar_blocked_sector_half_width'),
                value_type=float,
            ),
        }],
        output='screen',
    )

    # E. Nav2 Bringup (Mapa, AMCL, Planejador)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "bringup_launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": "true",
            "map": LaunchConfiguration("map"),
            "params_file": configured_nav2_params,
            "use_robot_state_pub": "false",
        }.items(),
    )
    delayed_nav2_bringup = TimerAction(
        period=3.0,
        actions=[nav2_bringup],
    )

    node_collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        parameters=[configured_nav2_parameter_file],
        output='screen',
    )
    node_collision_monitor_lifecycle = Node(
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
    delayed_collision_monitor = TimerAction(
        period=2.0,
        actions=[
            node_collision_monitor,
            node_collision_monitor_lifecycle,
        ],
    )

    # F. Rosbridge usado pelo aplicativo e pela face
    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml',
            )
        ),
        launch_arguments={
            'port': LaunchConfiguration('rosbridge_port'),
        }.items(),
    )

    # G. Interface Visual (RViz)
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config"), "-f", "map"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(use_rviz),
    )
    delayed_rviz = TimerAction(
        period=12.0,
        actions=[node_rviz],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("closed_loop", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("rosbridge_port", default_value="9090"),
        DeclareLaunchArgument("base_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB1"),
        DeclareLaunchArgument("lidar_minimum_range", default_value="0.15"),
        DeclareLaunchArgument(
            "lidar_blocked_sector_enabled", default_value="true"),
        DeclareLaunchArgument(
            "lidar_blocked_sector_center", default_value="0.0"),
        DeclareLaunchArgument(
            "lidar_blocked_sector_half_width", default_value="0.261799"),
        DeclareLaunchArgument(
            "base_params_file", default_value=default_base_params),
        DeclareLaunchArgument(
            "nav2_params_file", default_value=default_params),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),
        DeclareLaunchArgument("map", default_value=default_map),

        UnsetEnvironmentVariable("ROS_DISCOVERY_SERVER"),
        UnsetEnvironmentVariable("FASTDDS_DEFAULT_PROFILES_FILE"),
        UnsetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE"),
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "0"),

        node_rplidar,
        launch_base,
        node_robot_state_publisher,
        node_joint_state_publisher,
        node_script_filter,
        rosbridge,
        delayed_collision_monitor,
        delayed_nav2_bringup,
        delayed_rviz
    ])
