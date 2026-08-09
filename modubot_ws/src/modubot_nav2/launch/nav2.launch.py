import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import xacro

def generate_launch_description():
    # 1. Caminhos e Configurações
    pkg_description = 'modubot_model_description'
    pkg_nav2 = 'modubot_nav2'
    pkg_rplidar = 'rplidar_ros'
    
    # Processamento do Xacro
    xacro_file = os.path.join(get_package_share_directory(pkg_description), 'urdf/modubot_model.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Configurações do Nav2
    nav2_share = get_package_share_directory(pkg_nav2)
    default_params = os.path.join(nav2_share, "config", "nav2_params.yaml")
    default_map = os.path.join(nav2_share, "maps", "piso01.yaml")
    default_rviz = os.path.join(nav2_share, "rviz", "nav2.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    closed_loop = LaunchConfiguration("closed_loop", default="true")

    # --- A EQUIPE DE LANÇAMENTO ---

    # A. O Lidar (Atenção à porta USB)
    launch_rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg_rplidar), 'launch', 'rplidar_a1_launch.py')
        ),
        launch_arguments={'serial_port': '/dev/ttyUSB1'}.items() # <-- VERIFIQUE SE É ESTA PORTA
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
        launch_arguments={'closed_loop': closed_loop}.items(),
    )

    # C. A "Alma" do Robô (Estado e Juntas)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': use_sim_time}]
    )

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # O Filtro de Laser (Correção: Usando ExecuteProcess)
    node_script_filter = ExecuteProcess(
        cmd=['python3', '/workspace/modubot_ws/src/modubot_model_description/scripts/simple_filter.py'],
        output='screen'
    )

    # E. Nav2 Bringup (Mapa, AMCL, Planejador)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("nav2_bringup"), "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": "true",
            "map": LaunchConfiguration("map"),
            "params_file": LaunchConfiguration("params_file"),
            "use_robot_state_pub": "false",
        }.items(),
    )

    # F. Interface Visual (RViz)
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config"), "-f", "map"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen"
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("closed_loop", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),
        DeclareLaunchArgument("map", default_value=default_map),

        launch_rplidar,
        launch_base,
        node_robot_state_publisher,
        node_joint_state_publisher,
        node_script_filter,
        nav2_bringup,
        node_rviz
    ])
