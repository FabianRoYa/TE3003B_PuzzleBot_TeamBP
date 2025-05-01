import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('blackpearls_nav2_puzzlebot')
    # Ruta al archivo del mundo de Gazebo 
    world_file = os.path.join(
        pkg_share,
        'worlds',
        'modulo3_world'
    )

    # Lanzar Gazebo con el mundo definido
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )


    # Lanzar RViz para mapeo
    rviz_mapping = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_mapping',
        arguments=['-d', mapping_rviz_config],
        output='screen'
    )

    # Lanzar RViz para navegación
    rviz_navigation = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_navigation',
        arguments=['-d', navigation_rviz_config],
        output='screen'
    )


# Get the path to the package directory
    urdf_file_name = 'puzzlebot.urdf'
    urdf = os.path.join(
        get_package_share_directory('puzzlebot_sim'),
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
        package='puzzlebot_sim',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )   
    puzzlebot_sim = Node(
        package='puzzlebot_sim',
        executable='puzzlebot_sim',
        name='puzzlebot_sim',
        output='screen',
    )
    point_stabilisation_node = Node(
        package='puzzlebot_sim',
        executable='point_stabilisation_controller',
        name='point_stabilisation_controller',
        output='screen',
    )
    # Create the localization node
    localisation_node = Node(
        package='puzzlebot_sim',
        executable='localisation',
        name='localisation',
        output='screen',
    )
    
    rviz2_pub_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('puzzlebot_sim'), 'rviz', 'puzzlebot.rviz')],
        output='screen',
    )
    rqt_tf_tree_node = Node(
        package='rqt_tf_tree',
        executable='rqt_tf_tree',
        name='rqt_tf_tree',
        output='screen',
    )
    
    shape_drawer = Node(
        package='puzzlebot_sim',
        executable='shapeDrawer',
        name='shape_drawer',
        parameters=[
            {'shape': 'square'},
            {'size': 1.0}
        ]
    )
    
    # Teleop keyboard
    teleop_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }],
        remappings=[
            ('cmd_vel', '/puzzlebot/cmd_vel')
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
#############################################
#############################################
    ld = LaunchDescription([
        
        # Gazebo y Rviz
        LogInfo(msg=['Iniciando Gazebo con: ', world_file]),
        gazebo,
        LogInfo(msg=['Iniciando RViz (mapeo): ', mapping_rviz_config]),
        rviz_mapping,
        LogInfo(msg=['Iniciando RViz (navegación): ', navigation_rviz_config]),
        rviz_navigation,
        
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
        # teleop_keyboard_node
        
    ])
    return ld
