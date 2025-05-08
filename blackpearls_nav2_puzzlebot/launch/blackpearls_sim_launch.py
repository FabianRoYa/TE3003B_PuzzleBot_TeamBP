#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = 'blackpearls_nav2_puzzlebot'
    pkg_share = get_package_share_directory(pkg)

    # 0) Ajustar recurso para mallas: apunta al directorio 'share' padre
    share_dir = os.path.dirname(pkg_share)
    set_ign_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=share_dir
    )

    # Rutas
    world_path = os.path.join(pkg_share, 'worlds', 'world.world')
    urdf_path = os.path.join(pkg_share, 'urdf', 'puzzlebot.urdf')
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    # 1) Gazebo Garden
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': world_path}.items()
    )

    # 2) URDF → TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # 3) Joint States
    joint_state_publisher = Node(
        package=pkg,
        executable='joint_state_publisher',
        output='screen'
    )

    # 4) Spawn robot
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

    # 5) Otros nodos
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
    point_stabilisation_node = Node(
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

    ld = LaunchDescription([
        set_ign_path,
        gazebo_launch,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,
        puzzlebot_sim,
        localisation_node,
        point_stabilisation_node,
        shape_drawer,
    ])

    return ld
