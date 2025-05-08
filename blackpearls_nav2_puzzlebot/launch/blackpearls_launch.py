#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import LogInfo, IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('blackpearls_nav2_puzzlebot')

    # Argumento 'mode' para seleccionar perfil: 'sim', 'map', 'nav' o 'all'
    mode = LaunchConfiguration('mode')

    # Ruta al world y a configuraciones RViz
    world_file = os.path.join(pkg_share, 'worlds', 'world.world')
    mapping_rviz_config    = os.path.join(pkg_share, 'rviz', 'map.rviz')
    navigation_rviz_config = os.path.join(pkg_share, 'rviz', 'nav.rviz')

    # Rutas a lanzadores auxiliares de mapeo y navegación
    map_launch  = os.path.join(pkg_share, 'launch', 'blackpearls_map_launch.py')
    rviz_launch = os.path.join(pkg_share, 'launch', 'blackpearls_rviz_launch.py')

    # Nodos RViz condicionales según 'mode'
    rviz_mapping = Node(
        package='rviz2', executable='rviz2', name='rviz_mapping',
        arguments=['-d', mapping_rviz_config], output='screen',
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'map' or '", mode, "' == 'all'"
        ]))
    )
    rviz_navigation = Node(
        package='rviz2', executable='rviz2', name='rviz_navigation',
        arguments=['-d', navigation_rviz_config], output='screen',
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'nav' or '", mode, "' == 'all'"
        ]))
    )

    ld = LaunchDescription([

        # Parámetro 'mode'
        DeclareLaunchArgument(
            'mode', default_value='all',
            description="Modo de lanzamiento: 'sim', 'map', 'nav' o 'all'"
        ),

        # Lanzar Gazebo Garden (modo 'sim' o 'all')
        LogInfo(msg=['[MAIN] Launching Gazebo Garden with world: ', world_file]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch', 'gz_sim.launch.py'
                )
            ),
            launch_arguments={'gz_args': world_file}.items(),
            condition=IfCondition(PythonExpression([
                "'", mode, "' in ['sim','all']"
            ]))
        ),

        # Lanzar perfil de mapeo (modo 'map' o 'all')
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(map_launch),
            condition=IfCondition(PythonExpression([
                "'", mode, "' in ['map','all']"
            ]))
        ),

        # Lanzar perfil de RViz completo (modo 'nav' o 'all')
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_launch),
            condition=IfCondition(PythonExpression([
                "'", mode, "' in ['nav','all']"
            ]))
        ),

        #rviz_mapping,
        #rviz_navigation,

        LogInfo(msg=["[MAIN] RViz mode: ", mode]),

    ])

    return ld
