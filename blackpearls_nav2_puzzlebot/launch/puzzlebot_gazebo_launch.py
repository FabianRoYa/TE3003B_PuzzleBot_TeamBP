import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression, TextSubstitution, Command
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import math


def generate_launch_description():
    # -----------------------------------------------------------------------------
    #                          DECLARE LAUNCH ARGUMENTS
    # -----------------------------------------------------------------------------
    
    goal_x_arg = DeclareLaunchArgument(
        'goal_x',
        default_value='0.5',
        description='X coordinate of the goal position'
    )
    goal_y_arg = DeclareLaunchArgument(
        'goal_y',
        default_value='0.2',
        description='Y coordinate of the goal position'
    )
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='bug0',
        description='Navigation mode: bug0 or bug2'
    )
    
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    mode = LaunchConfiguration('mode')
    
    # -----------------------------------------------------------------------------
    #                          SIMULATION CONFIGURATION
    # -----------------------------------------------------------------------------
    
    # Name of the Gazebo world to load
    # world = 'puzzlebot_final_world.world' # Paolo's world [Final world]
    world = 'maze_aruco.world'  # Gadi's world for testing
    rviz_config_path = os.path.join(get_package_share_directory('blackpearls_nav2_puzzlebot'), 'rviz/conf.rviz')

    
    
    
    # General Gazebo settings
    pause = 'false'           # Start Gazebo in paused state, world tf is not generated until Gazebo starts
    verbosity = '1'           # Gazebo log verbosity level
    use_sim_time = 'False'     # Enable use of simulated clock (for ROS time sync)

    
    # Robot configurations (can be extended or loaded from a JSON file in future)
    robot_config_list = [
        {
            'name': '',
            'type': 'puzzlebot_jetson_lidar_ed',
            'x': 2.5, 'y': 2.5, 'yaw': 0.0,
            'lidar_frame': 'laser_frame',
            'camera_frame': 'camera_link_optical',
            'tof_frame': 'tof_link'
        }
    ]
    # -----------------------------------------------------------------------------
    #                         STATIC TRANSFORM PUBLISHER
    # -----------------------------------------------------------------------------
    
    # The goal frame, for better understanding of the robot's goal position
    static_tf3 = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                        goal_x, goal_y  , '0',  # X, Y, Z coordinates of the goal frame
                       '0', '0', '0', 'world', 'goal_frame'],
        )


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
        }.items(),
        condition = IfCondition(PythonExpression(["'", use_sim_time, "' == 'True'"])),
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
                # 'use_sim_time': use_sim_time
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
                'x': float(x),
                'y': float(y),

                'goal_x': goal_x,
                'goal_y': goal_y,

                # 'use_sim_time': bool(use_sim_time),
                'mode': mode,
                
                'Kp_linear': 0.5,  # Proportional gain for linear velocity
                'Kp_angular': 0.15, # Proportional gain for angular velocity
                
                'max_linear_speed': 0.5,  # Maximum linear speed
                'max_angular_speed': 0.5, # Maximum angular speed
                
                'follow_distance': 0.8,  # Distance to maintain from the goal
                'stop_d': 0.3,  # Distance to stop before reaching the goal [The less that the lidar can handle is 0.15]
                'goal_tolerance_distance': 0.05,  # Tolerance distance to consider goal reached
                'turning_d_deg': 5.0  # Turning distance in degrees
            }]
        )
        robot_launches.append(controller_node)

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
                'wr_topic': 'VelocityEncR',
                'wl_topic': 'VelocityEncL',
                'initial_pose':[float(x), float(y), float(yaw)],
                # 'use_sim_time': bool(use_sim_time),
                'world_frame':'world'
            }]
        )
        robot_launches.append(localisation_node)

        joint_state_publisher_node= Node(
            package='blackpearls_nav2_puzzlebot',
            executable='joint_state_publisher',
            name= 'joint_state_publisher',
            output='screen',
            parameters=[{
                # 'use_sim_time': False,
                'initial_pose': [0.0, 0.0, 0.0],
                'odometry_frame': 'odom',
            }],
            # condition = IfCondition(PythonExpression(["'", use_sim_time, "' == 'False'"]))
        )
        robot_launches.append(joint_state_publisher_node)


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
    arguments=['-d', rviz_config_path],
    # parameters=[{'use_sim_time': bool(use_sim_time)}],
    output='screen',
    )
    # -----------------------------------------------------------------------------
    #                         COMPOSE FINAL LAUNCH DESCRIPTION
    # -----------------------------------------------------------------------------
    ld = LaunchDescription([
        
        goal_x_arg,
        goal_y_arg,
        mode_arg,

        static_tf3,
        
        
        rviz2_pub_node,
        # gazebo_launch,
        *robot_launches,
    ])
    
    return ld