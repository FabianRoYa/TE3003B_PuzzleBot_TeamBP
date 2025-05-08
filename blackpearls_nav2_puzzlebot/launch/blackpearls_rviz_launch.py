#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_name = 'blackpearls_nav2_puzzlebot'

    # Argumentos de lanzamiento
    mapping_arg = DeclareLaunchArgument(
        'mapping',
        default_value='true',
        description='Habilita RVIZ para mapeo'
    )
    navigation_arg = DeclareLaunchArgument(
        'navigation',
        default_value='true',
        description='Habilita RVIZ para navegación'
    )

    # Paths a configuraciones RViz y al mundo
    rviz_mapping_config = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'rviz', 'mapping', 'mapping_config.rviz'
    ])
    rviz_nav_config = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'rviz', 'navigation', 'navigation_config.rviz'
    ])
    world_path = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'worlds', 'world.world'
    ])

    return LaunchDescription([
        mapping_arg,
        navigation_arg,

        # Lanzar Gazebo Garden (Ignition) usando ros_gz_sim
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
        ),

        # Nodo RViz para mapeo (si mapping=='true')
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_mapping',
            output='screen',
            arguments=['-d', rviz_mapping_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('mapping'))
        ),

        # Nodo RViz para navegación (si navigation=='true')
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_navigation',
            output='screen',
            arguments=['-d', rviz_nav_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('navigation'))
        ),
    ])
