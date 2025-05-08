#!/usr/bin/env python3
# En este launchfile solo arrancamos RViz
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_name = 'blackpearls_nav2_puzzlebot'

    # Argumentos para habilitar mapeo / navegación en RViz
    mapping_arg = DeclareLaunchArgument(
        'mapping',
        default_value='true',
        description='Habilita RViz para mapeo'
    )
    navigation_arg = DeclareLaunchArgument(
        'navigation',
        default_value='true',
        description='Habilita RViz para navegación'
    )

    # Configs de RViz
    rviz_mapping_config = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'rviz', 'mapping', 'mapping_config.rviz'
    ])
    rviz_nav_config = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'rviz', 'navigation', 'navigation_config.rviz'
    ])

    return LaunchDescription([
        mapping_arg,
        navigation_arg,

        # Nodo RViz para mapeo (solo si mapping=='true')
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_mapping',
            output='screen',
            arguments=['-d', rviz_mapping_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('mapping'))
        ),

        # Nodo RViz para navegación (solo si navigation=='true')
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
