import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('blackpearls_nav2_puzzlebot')
    # Ruta al archivo del mundo de Gazebo 
    world_file = os.path.join(
        pkg_share,
        'worlds',
        'modulo3_world'
    )

    # Lanzar Gazebo con el mundo definido
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )


    # Lanzar RViz para mapeo
    rviz_mapping = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_mapping',
        arguments=['-d', mapping_rviz_config],
        output='screen'
    )

    # Lanzar RViz para navegación
    rviz_navigation = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_navigation',
        arguments=['-d', navigation_rviz_config],
        output='screen'
    )


# Get the path to the package directory
    urdf_file_name = 'puzzlebot.urdf'
    urdf = os.path.join(
        get_package_share_directory('puzzlebot_sim'),
        'urdf',
        urdf_file_name)
    
   
#############################################
#############################################
    ld = LaunchDescription([
        
        # Gazebo y Rviz
        LogInfo(msg=['Iniciando Gazebo con: ', world_file]),
        gazebo,
        LogInfo(msg=['Iniciando RViz (mapeo): ', mapping_rviz_config]),
        rviz_mapping,
        LogInfo(msg=['Iniciando RViz (navegación): ', navigation_rviz_config]),
        rviz_navigation,
        
       
        
    ])
    return ld
