import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import LogInfo, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():
    pkg_share = get_package_share_directory('blackpearls_nav2_puzzlebot')
    
    # Argumento 'mode' (opcional, sólo si quieres condicionar)
    mode = LaunchConfiguration('mode', default='all')
    # Puedes usar valores: 'all', 'sim', 'map', 'rviz'

    # Rutas a archivos y configs
    world_file = os.path.join(pkg_share, 'worlds', 'modulo3_world.world')
    mapping_rviz_config    = os.path.join(pkg_share, 'rviz', 'map.rviz')
    navigation_rviz_config = os.path.join(pkg_share, 'rviz', 'nav.rviz')

    # Launch files auxiliares
    sim_launch  = os.path.join(pkg_share, 'launch', 'blackpearls_sim_launch.py')
    map_launch  = os.path.join(pkg_share, 'launch', 'blackpearls_map_launch.py')
    rviz_launch = os.path.join(pkg_share, 'launch', 'blackpearls_rviz_launch.py')

    # 1) Incluimos el launch oficial de gazebo_ros con tu world
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

    # 2) Definimos nodos de RViz “manuales” (si aún los quieres usar)
    rviz_mapping = Node(
        package='rviz2', executable='rviz2', name='rviz_mapping',
        arguments=['-d', mapping_rviz_config], output='screen'
    )
    rviz_navigation = Node(
        package='rviz2', executable='rviz2', name='rviz_navigation',
        arguments=['-d', navigation_rviz_config], output='screen'
    )

    # 3) Construimos la descripción de lanzamiento
    ld = LaunchDescription([
        # (Opcional) argumento para condicionar
        DeclareLaunchArgument(
            'mode', default_value='all',
            description="Choose 'all', 'sim', 'map' or 'rviz'"
        ),

        # --- Gazebo ---
        LogInfo(msg=['[MAIN] Launching Gazebo with world: ', world_file]),
        gazebo_launch,

        # --- Incluye cada sub-launch ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            condition=IfCondition(PythonExpression(["'", mode, "' in ['all','sim']"]))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(map_launch),
            condition=IfCondition(PythonExpression(["'", mode, "' in ['all','map']"]))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_launch),
            condition=IfCondition(PythonExpression(["'", mode, "' in ['all','rviz']"]))
        ),

        # --- (Opcional) tus nodos RViz “manuales” si prefieres sobre el rviz_launch.py ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_launch),
            condition=IfCondition(PythonExpression(["'", mode, "' == 'all'"]))
        ),
    ])

    return ld
