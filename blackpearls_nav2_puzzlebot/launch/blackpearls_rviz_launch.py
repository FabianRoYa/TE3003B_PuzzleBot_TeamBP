from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition  # Importación crítica faltante

def generate_launch_description():
    pkg_name = 'blackpearls_nav2_puzzlebot'
    
    # Declarar argumentos de lanzamiento
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

    # Configurar paths
    rviz_mapping_config = PathJoinSubstitution([FindPackageShare(pkg_name), 'rviz', 'mapping', 'mapping_config.rviz'])
    rviz_nav_config = PathJoinSubstitution([FindPackageShare(pkg_name), 'rviz', 'navigation', 'navigation_config.rviz'])

    return LaunchDescription([
        mapping_arg,
        navigation_arg,

        # Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ]),
            launch_arguments={
                'world': PathJoinSubstitution([
                    FindPackageShare('blackpearls_nav2_puzzlebot'),  # Reemplaza con tu paquete
                    'worlds',
                    'world.world'     # Nombre de tu archivo de mundo
                ]),
                'verbose': 'true'
                }.items()
            )
        ,

        # RVIZ Mapping
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_mapping',
            output='screen',
            arguments=['-d', rviz_mapping_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('mapping'))  # Condición corregida
        ),

        # RVIZ Navigation
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_navigation',
            output='screen',
            arguments=['-d', rviz_nav_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('navigation'))  # Condición corregida
        )
    ])