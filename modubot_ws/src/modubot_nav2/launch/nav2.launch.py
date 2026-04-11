import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    rviz_config = LaunchConfiguration("rviz_config")
    use_static_tf_laser = LaunchConfiguration("use_static_tf_laser")

    # Ajuste aqui para o seu pacote/arquivo do modelo:
    description_pkg = "modubot_model_description"
    xacro_file = "urdf/modubot.urdf.xacro"

    description_share = get_package_share_directory(description_pkg)
    xacro_path = os.path.join(description_share, xacro_file)

    nav2_share = get_package_share_directory("modubot_nav2")
    default_params = os.path.join(nav2_share, "config", "nav2_params.yaml")
    default_rviz = os.path.join(nav2_share, "rviz", "nav2.rviz")
    default_map = os.path.join(nav2_share, "maps", "piso01.yaml")

    # Robot State Publisher (carrega o CAD/URDF no RViz)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": Command(["xacro ", xacro_path]),
        }],
    )

    # Joint State Publisher (ok para RViz; no robô real pode ser opcional se você já publica /joint_states)
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Static TF do LiDAR (use apenas se o seu URDF NÃO publicar base_link->laser)
    static_tf_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0.20", "0.0", "0.15", "0", "0", "0", "base_link", "laser"],
        condition=IfCondition(use_static_tf_laser),
    )

    # Nav2 bringup (AMCL + map_server + planner + controller + costmaps + BT)
    # Importante: passar "map" e "autostart" para o lifecycle manager configurar/ativar sozinho.
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
            "autostart": autostart,
            "map": map_yaml,
            "params_file": params_file,
            # Não prejudica; útil se você decidir usar bringup para abrir RViz no futuro
            "rviz_config": rviz_config,
            # Em muitos setups ajuda a receber /map de forma "latched" (transient local)
            "map_subscribe_transient_local": "true",
        }.items(),
    )

    # RViz2 (você abre manualmente; ok)
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically configure/activate Nav2 lifecycle nodes",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Full path to the ROS2 parameters file for Nav2",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz,
            description="Full path to the RViz2 config file",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=default_map,
            description="Full path to the map yaml file",
        ),
        DeclareLaunchArgument(
            "use_static_tf_laser",
            default_value="true",
            description="Publish static TF base_link->laser if URDF does not provide it",
        ),

        robot_state_publisher,
        joint_state_publisher,
        static_tf_laser,

        nav2_bringup,
        rviz,
    ])

