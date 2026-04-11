import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    rviz_config = LaunchConfiguration("rviz_config")

    # Ajuste aqui para o seu pacote/arquivo do modelo:
    description_pkg = "modubot_model_description"
    xacro_file = "urdf/modubot.urdf.xacro"

    description_share = get_package_share_directory(description_pkg)
    xacro_path = os.path.join(description_share, xacro_file)

    nav2_share = get_package_share_directory("modubot_nav2")
    default_params = os.path.join(nav2_share, "config", "nav2_params.yaml")
    default_rviz = os.path.join(nav2_share, "rviz", "nav2.rviz")
    default_map = os.path.join(nav2_share, "maps", "piso01.yaml")  # você vai criar/salvar esse mapa

    # Robot State Publisher (carrega o CAD/URDF no RViz)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": Command(["xacro ", xacro_path])
        }],
    )

    # Joint State Publisher (para rodas/articulações ficarem “ok” no RViz)
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Static TF do LiDAR (use apenas se o seu URDF NÃO publicar base_link->laser)
    # Ajuste xyz/rpy conforme montagem real
    static_tf_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0.20", "0.0", "0.15", "0", "0", "0", "base_link", "laser"],
    )

    # Nav2 bringup (AMCL + map_server + planner + controller + costmaps + BT)
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
        }.items(),
    )

    # RViz2
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),
        DeclareLaunchArgument("map", default_value=default_map),

        robot_state_publisher,
        joint_state_publisher,

        # Se o seu URDF já define o frame do laser com joint fixo, você pode comentar essa linha:
        static_tf_laser,

        nav2_bringup,
        rviz,
    ])
