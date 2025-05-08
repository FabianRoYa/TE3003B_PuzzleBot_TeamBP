#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('blackpearls_nav2_puzzlebot')
    launch_dir = os.path.join(pkg_share, 'launch')

    # Argumento 'mode': sim | map | nav | all
    mode = LaunchConfiguration('mode')

    # Rutas a los child-launchfiles
    sim_launch  = os.path.join(launch_dir, 'blackpearls_sim_launch.py')
    map_launch  = os.path.join(launch_dir, 'blackpearls_map_launch.py')
    rviz_launch = os.path.join(launch_dir, 'blackpearls_rviz_launch.py')

    return LaunchDescription([
        # 1) Declarar parámetro 'mode'
        DeclareLaunchArgument(
            'mode', default_value='all',
            description="Modo de lanzamiento: 'sim', 'map', 'nav' o 'all'"
        ),

        # 2) Mensaje de arranque
        LogInfo(msg=['[MAIN] Launch mode: ', mode]),

        # 3) Incluir simulación (Gazebo + robot + nodos) si mode sim o all
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            condition=IfCondition(PythonExpression([
                "'", mode, "' in ['sim','all']"
            ]))
        ),

        # 4) Incluir mapeo si mode map o all
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(map_launch),
            condition=IfCondition(PythonExpression([
                "'", mode, "' in ['map','all']"
            ]))
        ),

        # 5) Incluir RViz si mode nav or all
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_launch),
            condition=IfCondition(PythonExpression([
                "'", mode, "' in ['nav','all']"
            ]))
        ),
    ])
