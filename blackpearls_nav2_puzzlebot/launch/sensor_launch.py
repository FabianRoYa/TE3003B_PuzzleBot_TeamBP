from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess, LogInfo
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution, Command, PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    package_name = 'blackpearls_nav2_puzzlebot'
    robot = 'puzzlebot_jetson_lidar_ed'
    rviz_config_file_path = 'rviz/urdf_gazebo_config.rviz' 
    pos_x = '0.3'
    pos_y = '2.7'
    pos_th = '0.0'
    sim_time = 'true'
    pause_gazebo = 'false'
    camera_frame = 'camera_link_optical'
    lidar_frame = 'laser_frame'
    tof_frame = 'tof_link'
    gazebo_verbosity = 4
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_share = get_package_share_directory('blackpearls_nav2_puzzlebot')
    robot_path = os.path.join(pkg_share, 'urdf', 'mcr2_robots',  f"{robot}.xacro")
    world_path = os.path.join(pkg_share, 'worlds', 'world.world')
    gazebo_models_path = os.path.join(pkg_share, 'models')
    gazebo_plugins_path = os.path.join(pkg_share,'plugins')
    gazebo_media_path = os.path.join(pkg_share, 'models', 'media', 'materials')
    default_ros_gz_bridge_config_file_path = os.path.join(pkg_share, 'config', f"{robot}.yaml")
    rviz_config_path = os.path.join(pkg_share, rviz_config_file_path)
    declare_x_arg = DeclareLaunchArgument('x', default_value=pos_x, description='X position of the robot')
    declare_y_arg = DeclareLaunchArgument('y', default_value=pos_y, description='Y position of the robot')
    declare_th_arg = DeclareLaunchArgument('yaw', default_value=pos_th, description='angle of the robot')
    declare_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=sim_time, description='Use simulated time')
    declare_pause_arg = DeclareLaunchArgument('pause', default_value=pause_gazebo, description='Start Gazebo paused')
    declare_camera_frame_arg = DeclareLaunchArgument('camera_frame', default_value=camera_frame, description='Camera frame')
    declare_tof_frame_arg = DeclareLaunchArgument('tof_frame', default_value=tof_frame, description='TOF sensor frame')
    declare_lidar_frame_arg = DeclareLaunchArgument('lidar_frame', default_value=lidar_frame, description='Lidar sensor frame')
    declare_mode = DeclareLaunchArgument('mode', default_value='')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')
    use_sim_time = LaunchConfiguration('use_sim_time')
    pause = LaunchConfiguration('pause')
    camera_frame_name = LaunchConfiguration('camera_frame')
    tof_frame_name = LaunchConfiguration('tof_frame')
    lidar_frame_name = LaunchConfiguration('lidar_frame')
    mode = LaunchConfiguration('mode')
    set_gazebo_resources = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{gazebo_models_path}:{gazebo_media_path}"
    )
    set_gazebo_plugins = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=f"{gazebo_plugins_path}"
    )
    robot_description = Command([
        'xacro ', str(robot_path),
        ' camera_frame:=', camera_frame_name,
        ' tof_frame:=', tof_frame_name,
        ' lidar_frame:=', lidar_frame_name,
    ])
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    start_gazebo_server_run = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args':  ['-r' f'-v {gazebo_verbosity} ', world_path],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=UnlessCondition(pause)
    )
    start_gazebo_server_paused = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args':  [f'-v {gazebo_verbosity} ', world_path],
            'on_exit_shutdown': 'true',
            'pause': 'true',
        }.items(),
        condition=IfCondition(pause)
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": ParameterValue(robot_description, value_type=str),
            "use_sim_time": use_sim_time,
        }],
    )
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "puzzlebot",
            "-topic", "robot_description",
            "-x", x, "-y", y, "-Y", yaw,
        ],
        output="screen",
    )
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': default_ros_gz_bridge_config_file_path,
        }],
        output='screen'
    )
    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[PathJoinSubstitution([TextSubstitution(text='camera')])]
    )
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('slam_toolbox'),
            '/launch/online_async_launch.py'
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition = IfCondition(PythonExpression(["'", mode, "' == 'map'"]))
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
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'world']
    )
    static_tf2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom']
    )
    static_tf3 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.3', '0.5', '0', '0', '0', '0', 'world', 'goal']
    )
    frame_node = Node(
        name="frame",
        package=package_name,
        executable='frame',
        parameters=[{'use_sim_time': True},]
    )
    localization_node = Node(
        name="localization",
        package=package_name,
        executable='localization',
    )
    controller_node = Node(
        name="controller",
        package=package_name,
        executable='controller',
        parameters=[{'use_sim_time': True},],
        condition = IfCondition(PythonExpression(["'", mode, "' == 'nav'"]))
    )
    l_d = [
        declare_x_arg,
        declare_y_arg,
        declare_th_arg,
        declare_sim_time_arg,
        declare_pause_arg,
        declare_camera_frame_arg, 
        declare_tof_frame_arg,
        declare_lidar_frame_arg,
        declare_mode,
        set_gazebo_resources,
        set_gazebo_plugins, 
        robot_state_publisher_node,
        start_gazebo_server_run,
        start_gazebo_server_paused,
        spawn_robot,
        start_gazebo_ros_bridge_cmd,
        start_gazebo_ros_image_bridge_cmd,
        nav2_launch,
        rviz,
        static_tf,
        static_tf2,
        static_tf3,
        frame_node,
        #localization_node,
        ]
    l_d.extend([
        slam_launch,
        controller_node,
    ])
    return LaunchDescription(l_d)
