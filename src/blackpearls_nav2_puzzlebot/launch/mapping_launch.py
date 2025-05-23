from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    map_name = LaunchConfiguration('map_name').perform(context)
    base_path = get_package_share_directory('blackpearls_nav2_puzzlebot')
    rviz_file = os.path.join(base_path, 'rviz', "mapping.rviz")
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    maps_dir = os.path.join(base_path, 'maps')
    
    
    # Crear directorio de mapas si no existe
    os.makedirs(maps_dir, exist_ok=True)

    args = {
        'use_sim_time': LaunchConfiguration('use_sim_time')
    }
    
    if map_name.lower() == 'puzzlebot':
        args['slam_params_file'] = LaunchConfiguration('slam_params_file')

    return [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(base_path, 'param', 'nav2_mapping.yaml'),
            description='Full path to Slam Toolbox params file'
        ),
        
        # Nav2 Stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
        ),
        
        # SLAM Toolbox - Mapping
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
            ),
            launch_arguments=args.items()
        ),
        
        # Map Saver (se ejecuta 10 segundos después del inicio)
        TimerAction(
            period=25.0,
            actions=[
                Node(
                    package='nav2_map_server',
                    executable='map_saver_cli',
                    name='map_saver',
                    output='screen',
                    arguments=['-t', 'map', '-f', os.path.join(maps_dir, map_name)],
                    parameters=[{'save_map_timeout': 10000}]
                )
            ]
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name',
            default_value='mi_mapa',
            description='Nombre del mapa a guardar'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Usar tiempo de simulación'
        ),
        # Don't move this -_-
        Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf2',
        arguments=[
            '0', '0', '0',  # x, y, z
            '0', '0', '0',  # roll, pitch, yaw
            'world',    # parent frame
            'map' # child frame
        ],),
        OpaqueFunction(function=launch_setup)
    ])