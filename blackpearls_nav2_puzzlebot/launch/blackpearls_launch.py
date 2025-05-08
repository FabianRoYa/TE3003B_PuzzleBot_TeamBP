import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import LogInfo, IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('blackpearls_nav2_puzzlebot')

    # Argumento 'mode' para seleccionar perfil: 'sim', 'map' o 'nav'
    mode = LaunchConfiguration('mode')

    # Rutas a archivos y configs
    world_file = os.path.join(pkg_share, 'worlds', 'world.world')
    mapping_rviz_config    = os.path.join(pkg_share, 'rviz', 'map.rviz')
    navigation_rviz_config = os.path.join(pkg_share, 'rviz', 'nav.rviz')

    # Rutas a lanzadores auxiliares
    sim_launch  = os.path.join(pkg_share, 'launch', 'blackpearls_sim_launch.py')
    map_launch  = os.path.join(pkg_share, 'launch', 'blackpearls_map_launch.py')
    rviz_launch = os.path.join(pkg_share, 'launch', 'blackpearls_rviz_launch.py')

    # Include para Gazebo usando el launch oficial de gazebo_ros
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true'
        }.items(),
    )

    # Nodos RViz condicionales según 'mode'
    rviz_mapping = Node(
        package='rviz2', executable='rviz2', name='rviz_mapping',
        arguments=['-d', mapping_rviz_config], output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'map' or '", mode, "' == 'all' "]))
    )
    rviz_navigation = Node(
        package='rviz2', executable='rviz2', name='rviz_navigation',
        arguments=['-d', navigation_rviz_config], output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'nav' or '", mode, "' == 'all' "]))
    )

    ld = LaunchDescription([
        # Definición del parámetro 'mode'
        DeclareLaunchArgument(
            'mode', default_value='all',
            description="Mode for launch: 'sim', 'map', 'nav' or 'all'"
        ),

        # Lanzar Gazebo (modo 'sim' o 'all')
        LogInfo(msg=['[MAIN] Launching Gazebo with world: ', world_file]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            condition=IfCondition(PythonExpression(["'", mode, "' in ['sim','all']"]))
        ),

        # Lanzar perfil de mapeo (modo 'map' o 'all')
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(map_launch),
            condition=IfCondition(PythonExpression(["'", mode, "' in ['map','all']"]))
        ),

        # Lanzar perfil de RViz completo (modo 'nav' o 'all')
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_launch),
            condition=IfCondition(PythonExpression(["'", mode, "' in ['nav','all']"]))
        ),

        # Alternativamente, lanzar Nodes RViz manueales según mode
        rviz_mapping,
        rviz_navigation,

        LogInfo(msg=["[MAIN] RViz mode: ", mode]),
    ])

    return ld
