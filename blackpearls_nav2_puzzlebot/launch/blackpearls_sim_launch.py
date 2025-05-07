import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

# /home/tony/PerlasNegras/src/blackpearls_nav2_puzzlebot/urdf

def generate_launch_description():
    urdf_file_name = 'puzzlebot.urdf'
    urdf = os.path.join(
        get_package_share_directory('blackpearls_nav2_puzzlebot'),
        'urdf',
        urdf_file_name)
    
    with open(urdf, 'r') as infp:
        robot_description = infp.read()
        print(robot_description)

    # Create the robot_state_publisher node
    robot_state_pub_node= Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description
        }],
        arguments=[urdf],
        output='screen'
    )

    # Create the joint_state_publisher node
    joint_state_publisher_node = Node(
        package='blackpearls_nav2_puzzlebot',
        executable='joint_state_publisher',
        name='JointStatePublisher',
        output='screen',
    )   
    puzzlebot_sim = Node(
        package='blackpearls_nav2_puzzlebot',
        executable='puzzlebot_sim',
        name='puzzlebot_sim',
        output='screen',
    )
    
    point_stabilisation_node = Node(
        package='blackpearls_nav2_puzzlebot',
        executable='point_stabilisation_controller',
        name='PointStabilisationController',
        output='screen',
    )
    
    # /home/tony/PerlasNegras/src/blackpearls_nav2_puzzlebot/blackpearls_nav2_puzzlebot/localisation.py
    # Create the localization node
    localisation_node = Node(
        package='blackpearls_nav2_puzzlebot',
        executable='localisation',
        name='Localisation',
        output='screen',
    )
    
    rviz2_pub_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # arguments=['-d', os.path.join(get_package_share_directory('puzzlebot_sim'), 'rviz', 'puzzlebot.rviz')],
        output='screen',
    )
    rqt_tf_tree_node = Node(
        package='rqt_tf_tree',
        executable='rqt_tf_tree',
        name='rqt_tf_tree',
        output='screen',
    )
    
    shape_drawer = Node(
        package='blackpearls_nav2_puzzlebot',
        executable='shapeDrawer',
        name='ShapeDrawer',
        parameters=[
            {'shape': 'square'},
            {'size': 1.0}
        ]
    )

    rqt_graph_node = Node(  
        package='rqt_graph',
        executable='rqt_graph',
        name='rqt_graph',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }],
        remappings=[
            ('cmd_vel', '/puzzlebot/cmd_vel')
        ]
    )
    
    ld = LaunchDescription([
         # Modelo Matematico y Odometria
        puzzlebot_sim,
        localisation_node,
        
        # Simulacion
        robot_state_pub_node,
        joint_state_publisher_node,
        rviz2_pub_node,
        
        # Debug
        rqt_tf_tree_node,
        rqt_graph_node,
    
        # Control y Rutinas de movimiento
        point_stabilisation_node,
        shape_drawer,
    
    ])
    return ld