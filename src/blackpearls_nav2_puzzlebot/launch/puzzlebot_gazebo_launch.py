import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Enable use of simulated clock (for ROS time sync)'
    )
    # -----------------------------------------------------------------------------
    #                         I NEED THIS TF
    # -----------------------------------------------------------------------------
    
   
    
    
    # -----------------------------------------------------------------------------
    #                          SIMULATION CONFIGURATION
    # -----------------------------------------------------------------------------
    
    # Name of the Gazebo world to load
    world = 'maze.world'

    # General Gazebo settings
    pause = 'false'           # Start Gazebo in paused state, world tf is not generated until Gazebo starts
    verbosity = '4'           # Gazebo log verbosity level
    use_sim_time = LaunchConfiguration('use_sim_time')     # Enable use of simulated clock (for ROS time sync)

    # mode_rviz = DeclareLaunchArgument(
    #     'mode',
    #     default_value='nav',
    #     description='Mode to load RVIZ configuration'
    # )
    
    # Robot configurations (can be extended or loaded from a JSON file in future)
    robot_config_list = [
        {
            'name': '',
            'type': 'puzzlebot_jetson_lidar_ed',
            'x': 0.3, 'y': 2.66, 'yaw': 0.0,
            'lidar_frame': 'laser_frame',
            'camera_frame': 'camera_link_optical',
            'tof_frame': 'tof_link'
        }
    ]

    # -----------------------------------------------------------------------------
    #                         LOAD GAZEBO WORLD
    # -----------------------------------------------------------------------------

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('blackpearls_nav2_puzzlebot'),
                'launch',
                'gazebo_world_launch.py'
            )
        ),
        launch_arguments={
            'world': world,
            'pause': pause,
            'verbosity': verbosity,
            'use_sim_time': use_sim_time
        }.items()
    )
    # -----------------------------------------------------------------------------
    #                       SPAWN EACH ROBOT DYNAMICALLY
    # -----------------------------------------------------------------------------
    robot_launches = []
    for robot in robot_config_list:
        robot_name   = robot['name']
        robot_type   = robot['type']
        x            = str(robot.get('x', 0.0))
        y            = str(robot.get('y', 0.0))
        yaw          = str(robot.get('yaw', 0.0))
        lidar_frame  = robot.get('lidar_frame', 'laser_frame')
        camera_frame = robot.get('camera_frame', 'camera_link_optical')
        tof_frame    = robot.get('tof_frame', 'tof_link')
        prefix = f'{robot_name}/' if robot_name != '' else ''

        # Each robot is launched using the shared puzzlebot launch file
        robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('blackpearls_nav2_puzzlebot'),
                    'launch',
                    'gazebo_puzzlebot_launch.py'
                )
            ),
            launch_arguments={
                'robot': robot_type,
                'robot_name': robot_name,
                'x': x,
                'y': y,
                'yaw': yaw,
                'prefix': prefix,
                'lidar_frame': lidar_frame,
                'camera_frame': camera_frame,
                'tof_frame': tof_frame,
                'use_sim_time': use_sim_time
            }.items()
        )

        robot_launches.append(robot_launch)
    
        
    # -----------------------------------------------------------------------------
    #                         ROBOT CONTROL NODES
    # -----------------------------------------------------------------------------
        controller_node = Node(
            package='blackpearls_nav2_puzzlebot',
            executable='point_stabilisation_controller',
            name='point_stabilisation_controller',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }]
        )
        # robot_launches.append(controller_node)
        
    # -----------------------------------------------------------------------------
    #                         ROBOT LOCALIZATION NODES
    # -----------------------------------------------------------------------------
    
    ### This node should gave the robot's covariance matrix 
    ### but it doesn't work, when ever I try to run it covariance matrix 
    ### is not published or is only zeros
    
        # localisation_node=Node(
        #     package='blackpearls_nav2_puzzlebot',
        #     executable='localisation',
        #     name='localisation',
        #     output='screen',
        #     parameters=[{
        #         'wr': 'VelocityEncR',
        #         'wl': 'VelocityEncL',
        #         'initialPose':[x, y, yaw], 
        #     }]
        # )
        # robot_launches.append(localisation_node)
    # =============================================================================
    #                         MAP or NAV LAUNCH CONFIGURATION
    # =============================================================================
    map_mode_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('blackpearls_nav2_puzzlebot'),
                'launch/mapping_launch.py'
            )
        ),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'map'"])
        )
    )

    nav_mode_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('blackpearls_nav2_puzzlebot'),
                'launch/navigation_launch.py'
            )
        ),
        launch_arguments={'map_name': LaunchConfiguration('map_name')}.items(),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'nav'"])
        )
    )

    # =============================================================================
    #                         COMPOSE FINAL LAUNCH DESCRIPTION
    # =============================================================================
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='nav',
            description='Operation mode (nav/map)'
        ),
        DeclareLaunchArgument(
            'map_name',
            default_value='map',
            description='Name of the map to load'
        ),
        
        declare_use_sim_time,
        
        # Mode-specific launches
        map_mode_launch,
        nav_mode_launch,
        
        # Core launches
        gazebo_launch,
        *robot_launches,
    ])
    
