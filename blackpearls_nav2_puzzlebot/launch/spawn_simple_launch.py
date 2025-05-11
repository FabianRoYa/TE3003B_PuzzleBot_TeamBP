import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    package_name = 'blackpearls_nav2_puzzlebot'
    rviz_config_file_path = 'rviz/urdf_gazebo_config.rviz' 
    urdf_file_path = 'urdf/puzzlebot.urdf'
    pkg_share = get_package_share_directory('blackpearls_nav2_puzzlebot')
    world_path = os.path.join(pkg_share, 'worlds', 'world.world')
    urdf_path = os.path.join(pkg_share, urdf_file_path)
    rviz_config_path = os.path.join(pkg_share, rviz_config_file_path)
    set_ign_path = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=os.path.dirname(pkg_share))
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': world_path}.items()
    )
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name',  'puzzlebot',
            '-x',     '0.3',
            '-y',     '0.3',
            '-z',     '0.0',
            '-roll',  '0.0',
            '-pitch', '0.0',
            '-yaw',   '1.0'
        ]
    )
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('slam_toolbox'),
            '/launch/online_async_launch.py'
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('nav2_bringup'),
            '/launch/navigation_launch.py'
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    rviz_p = ExecuteProcess(
        cmd=[
            'rviz2', '-d',
            '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz'
        ],
        output='screen'
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_path]), 'use_sim_time': True}]
    )
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    )
    static_tf1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    frame_node = Node(
        name="frame",
        package=package_name,
        executable='frame',
    )
    localization_node = Node(
        name="localization",
        package=package_name,
        executable='localization',
    )
    controller_node = Node(
        name="cobtroller",
        package=package_name,
        executable='controller',
    )
    point_generator_node = Node(
        name="point_generator",
        package=package_name,
        executable='point_generator',
    )
    publish_wr_wl = Node(
        package=package_name,
        executable='publish_wr_wl',
        parameters=[{'use_sim_time': True}],
        emulate_tty=True,
        output='screen'
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    return LaunchDescription([
        set_ign_path,
        gazebo_launch,
        spawn_robot,
        slam_launch,
        nav2_launch,
        rviz_p,
        robot_state_publisher,
        static_tf,
        static_tf1,
        frame_node,
        localization_node,
        controller_node,
        point_generator_node,
        publish_wr_wl,
    ])
