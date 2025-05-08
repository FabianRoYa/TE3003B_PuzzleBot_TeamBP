#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('blackpearls_nav2_puzzlebot')

    # 0) Ajustar recurso para mallas de Ignition
    set_ign_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.path.dirname(pkg_share)
    )

    # 1) Ruta a tu world.world
    world_path = os.path.join(pkg_share, 'worlds', 'world.world')

    # 2) Lanzar Gazebo Garden con ese world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': world_path}.items()
    )

    return LaunchDescription([
        set_ign_path,
        gazebo_launch,
    ])
