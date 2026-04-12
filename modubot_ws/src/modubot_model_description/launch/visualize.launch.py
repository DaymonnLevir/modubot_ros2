import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Nome do pacote e arquivo
    pkg_name = 'modubot_model_description'
    file_subpath = 'urdf/modubot_model.xacro'

    # Caminho completo do arquivo xacro
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Nó: Robot State Publisher (Publica as TFs)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )

    # Nó: Joint State Publisher GUI (Abre janelinha para girar as rodas)
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    node_script_filter = Node(
        # Remova o 'package' ou deixe-o apenas se o executable for um binário do ROS
        executable='python3',
        arguments=['/workspace/modubot_ws/src/modubot_model_description/scripts/simple_filter.py'],
        output='screen'
    )
    
    # Nó: RViz2
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-f', 'base_link']
    )


    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz,
        node_script_filter
    ])

