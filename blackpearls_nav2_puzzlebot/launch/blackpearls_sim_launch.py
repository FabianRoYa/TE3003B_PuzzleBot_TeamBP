#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg       = 'blackpearls_nav2_puzzlebot'
    pkg_share = get_package_share_directory(pkg)

    # 1) Leer URDF y preparar robot_description
    urdf_path = os.path.join(pkg_share, 'urdf', 'puzzlebot.urdf')
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    # 2) Publicador de TF a partir de URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # 3) Publicador de joint_states 
    joint_state_publisher = Node(
        package=pkg,
        executable='joint_state_publisher',
        output='screen'
    )

    # 4) Spawn del robot en el mundo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name',  'puzzlebot',
            '-x',     '0.3',
            '-y',     '0.3',
            '-z',     '0.0',
            '-roll',  '0.0',
            '-pitch', '0.0',
            '-yaw',   '1.0'
        ]
    )

    # 5) Nodos de simulación y control
    puzzlebot_sim = Node(
        package=pkg,
        executable='puzzlebot_sim',
        output='screen'
    )
    localisation_node = Node(
        package=pkg,
        executable='localisation',
        output='screen'
    )
    point_stab_node = Node(
        package=pkg,
        executable='point_stabilisation_controller',
        output='screen'
    )
    shape_drawer = Node(
        package=pkg,
        executable='shapeDrawer',
        parameters=[{'shape': 'square'}, {'size': 1.0}],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,
        puzzlebot_sim,
        localisation_node,
        point_stab_node,
        shape_drawer,
    ])
