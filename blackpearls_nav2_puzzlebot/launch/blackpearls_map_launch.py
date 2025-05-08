#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Path al world de tu paquete
    world_path = PathJoinSubstitution([
        FindPackageShare('blackpearls_nav2_puzzlebot'),
        'worlds',
        'world.world'
    ])

    return LaunchDescription([
        # Launch de Gazebo Garden (Ignition) vía ros_gz_sim
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
                ])
            ]),
            launch_arguments={
                'gz_args': world_path
            }.items()
        )
    ])
