import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import math


def generate_launch_description():
    
    # -----------------------------------------------------------------------------
    #                          SIMULATION CONFIGURATION
    # -----------------------------------------------------------------------------
    
    # Name of the Gazebo world to load
    world = 'maze_aruco.world'

    # General Gazebo settings
    pause = 'false'           # Start Gazebo in paused state, world tf is not generated until Gazebo starts
    verbosity = '1'           # Gazebo log verbosity level
    use_sim_time = 'True'     # Enable use of simulated clock (for ROS time sync)

    
    # Robot configurations (can be extended or loaded from a JSON file in future)
    robot_config_list = [
        {
            'name': '',
            'type': 'puzzlebot_jetson_lidar_ed',
            'x': 0.2, 'y': 2.7, 'yaw': 0.0,
            'lidar_frame': 'laser_frame',
            'camera_frame': 'camera_link_optical',
            'tof_frame': 'tof_link'
        }
    ]
    # Posiciones conocidas de los marcadores
    marker_positions = {
        0: [0.0, 0.0, 0.0],    # Marcador 0 en origen
        1: [1.5, 0.5, 1.57],    # Marcador 1 en x=1.5m, y=0.5m, orientación 90°
        2: [2.0, 1.0, 0.0]      # Marcador 2 en x=2.0m, y=1.0m
    }

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
            'verbosity': verbosity
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
                'kp_linear': 0.2,
                'kp_angular': 0.1,
                'max_linear_speed': 0.4,
                'max_angular_speed': 0.3,
                'goal_tolerance': 0.1,
                'angular_tolerance': math.radians(5)  # 5 grados en radianes
            }]
        )
        # robot_launches.append(controller_node)

        bug_algorithm_node = Node(
            package='blackpearls_nav2_puzzlebot',
            executable='bug_algorithm',
            name='bug_algorithm',
            output='screen',
            parameters=[{
                'mode': 'bug2',# or 'bug2'
                'goal_tolerance': 0.01,
                'angular_tolerance': math.radians(5),  # 5 grados en radianes
                
                # Goal pose & safe distances
                'goal_pose': [0.3, 0.1], # Goal position [x, y] in meters
                'safe_distance': 0.3,   # Safe distance from obstacles [meters]
                'safe_wall': 0.25,       # Safe distance from walls [meters]
                # Controller parameters
                'kp_angular': 0.15,
            }]
        )
        robot_launches.append(bug_algorithm_node)
        
    # -----------------------------------------------------------------------------
    #                         ROBOT VISION NODES
    # -----------------------------------------------------------------------------

        vision_node = Node(
            package='blackpearls_nav2_puzzlebot',
            executable='vision',
            name='vision',
            output='screen',
            parameters=[{'use_sim_time': True}], # KEEP IT TRUE
            
        )
        robot_launches.append(vision_node)

    # -----------------------------------------------------------------------------
    #                         ROBOT LOCALIZATION NODES
    # -----------------------------------------------------------------------------
    
    ### This node should gave the robot's covariance matrix 
    ### but it doesn't work, when ever I try to run it covariance matrix 
    ### is not published or is only zeros
    
        localisation_node=Node(
            package='blackpearls_nav2_puzzlebot',
            executable='localisation',
            name='localisation',
            output='screen',
            parameters=[{
                'wr': 'VelocityEncR',
                'wl': 'VelocityEncL',
                'initialPose':[float(x), float(y), float(yaw)],
                
            }]
        )
        robot_launches.append(localisation_node)
    # -----------------------------------------------------------------------------
    #                         RVIZ2 NODE
    # -----------------------------------------------------------------------------
    
    ### Rviz needs to have to modes, mapping and navigation
    ### The mapping mode is used to create the map and the navigation mode
    ### is used to navigate the robot using the map created in the mapping mode
    ### BUT I can't make it work, neither of them.

    rviz2_pub_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    )
    # -----------------------------------------------------------------------------
    #                         COMPOSE FINAL LAUNCH DESCRIPTION
    # -----------------------------------------------------------------------------
    ld = LaunchDescription([
        
        rviz2_pub_node,
        gazebo_launch,
        *robot_launches,
    ])
    
    return ld