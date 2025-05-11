from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Incluir el launch file por defecto de Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ]),
            # Especificar parámetros del mundo de Gazebo
            launch_arguments={
                'world': PathJoinSubstitution([
                    FindPackageShare('blackpearls_nav2_puzzlebot'),  # Reemplaza con tu paquete
                    'worlds',
                    'world.world'     # Nombre de tu archivo de mundo
                ]),
                'verbose': 'true'
            }.items()
        )
    ])