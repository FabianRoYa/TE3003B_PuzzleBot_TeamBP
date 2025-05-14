# navigation_launch.py (corrected imports)
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction  # CORRECTED IMPORT
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


# -----------------------------------------------------------------------------
#                          NAVIGATION LAUNCH
# -----------------------------------------------------------------------------

def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    base_path = get_package_share_directory('blackpearls_nav2_puzzlebot')
    rviz_file = os.path.join(base_path, 'rviz', "nav2_navigating.rviz")
    map_value = LaunchConfiguration('map_name').perform(context)

    # Cmabiar el valor de map para que aparezca el luga correcto con el robot
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            base_path,
            'map',
            #f"{map_value}_puzzlebot.yaml"))
            "prueba4_puzzlebot.yaml"))

    param_file_name = 'puzzlebot.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            base_path,
            'param',
            param_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    return [
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file],
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }]
        ),
    ]

def generate_launch_description():
    return LaunchDescription([
        # Don't move this -_-
        Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf2',
        arguments=[
            '0', '0', '0',  # x, y, z
            '0', '0', '0',  # roll, pitch, yaw
            'map',    # parent frame
            'odom' # child frame
        ],),
        OpaqueFunction(
            function=launch_setup
        ),
    ])
    