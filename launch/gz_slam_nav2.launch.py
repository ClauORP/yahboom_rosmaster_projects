#!/usr/bin/env python3
"""
Gazebo + ROS 2 Control + slam_toolbox + Nav2

slam_toolbox publishes /map and TF map->odom.
Nav2 uses map->odom from slam_toolbox (so AMCL MUST NOT run).
Frontier explorer selects frontier goals and sends NavigateToPose goals to Nav2.
"""

import os

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Packages
    package_name_gazebo = 'yahboom_rosmaster_gazebo'
    package_name_description = 'yahboom_rosmaster_description'
    package_name_bringup = 'yahboom_rosmaster_bringup'
    package_name_navigation = 'yahboom_rosmaster_navigation'
    package_name_worlds = 'yahboom_rosmaster_projects'

    default_robot_name = 'rosmaster_x3'
    default_world_file = 'rosmaster_home.world'

    gazebo_models_rel = 'models'
    gazebo_worlds_rel = 'worlds'
    ros_gz_bridge_rel = 'config/ros_gz_bridge.yaml'

    # One YAML containing BOTH slam_toolbox + nav2 params
    default_params_rel = 'config/rosmaster_x3_nav2_default_params.yaml'

    # Locate package shares
    pkg_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    pkg_share_gazebo = FindPackageShare(package=package_name_gazebo).find(package_name_gazebo)
    pkg_share_worlds = FindPackageShare(package=package_name_worlds).find(package_name_worlds)
    pkg_share_description = FindPackageShare(package=package_name_description).find(package_name_description)
    pkg_share_bringup = FindPackageShare(package=package_name_bringup).find(package_name_bringup)
    pkg_share_navigation = FindPackageShare(package=package_name_navigation).find(package_name_navigation)

    gazebo_models_path = os.path.join(pkg_share_worlds, gazebo_models_rel)
    default_ros_gz_bridge_config = os.path.join(pkg_share_gazebo, ros_gz_bridge_rel)
    default_params_file = os.path.join(pkg_share_navigation, default_params_rel)

    # Launch configurations
    enable_odom_tf = LaunchConfiguration('enable_odom_tf')
    headless = LaunchConfiguration('headless')
    jsp_gui = LaunchConfiguration('jsp_gui')
    load_controllers = LaunchConfiguration('load_controllers')
    robot_name = LaunchConfiguration('robot_name')
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world_file')

    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    # World path: <worlds_pkg>/worlds/<world_file>
    world_path = PathJoinSubstitution([pkg_share_worlds, gazebo_worlds_rel, world_file])

    # Spawn pose
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    # Args
    declare_enable_odom_tf_cmd = DeclareLaunchArgument(
        'enable_odom_tf', default_value='true', choices=['true', 'false'],
        description='Enable odom TF via ros2_control/robot_state_publisher pipeline')

    declare_headless_cmd = DeclareLaunchArgument(
        'headless', default_value='False',
        description='If true, do not start gzclient')

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name', default_value=default_robot_name,
        description='Robot name in Gazebo')

    declare_load_controllers_cmd = DeclareLaunchArgument(
        'load_controllers', default_value='true',
        description='Load ros2_control controllers')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub', default_value='true',
        description='Run robot_state_publisher')

    declare_jsp_gui_cmd = DeclareLaunchArgument(
        'jsp_gui', default_value='false',
        description='Enable joint_state_publisher_gui')

    declare_use_gazebo_cmd = DeclareLaunchArgument(
        'use_gazebo', default_value='true',
        description='Enable Gazebo')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use /clock from Gazebo')

    declare_world_cmd = DeclareLaunchArgument(
        'world_file', default_value=default_world_file,
        description='World file name under yahboom_rosmaster_projects/worlds')

    declare_params_cmd = DeclareLaunchArgument(
        'params_file', default_value=default_params_file,
        description='Single YAML containing BOTH Nav2 + slam_toolbox parameters'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Autostart Nav2 lifecycle nodes')

    declare_x_cmd = DeclareLaunchArgument('x', default_value='1.0')
    declare_y_cmd = DeclareLaunchArgument('y', default_value='0.0')
    declare_z_cmd = DeclareLaunchArgument('z', default_value='0.05')
    declare_roll_cmd = DeclareLaunchArgument('roll', default_value='0.0')
    declare_pitch_cmd = DeclareLaunchArgument('pitch', default_value='0.0')
    declare_yaw_cmd = DeclareLaunchArgument('yaw', default_value='0.0')

    # robot_state_publisher (no RViz)
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share_description, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={
            'enable_odom_tf': enable_odom_tf,
            'jsp_gui': jsp_gui,
            'use_rviz': 'false',
            'use_gazebo': use_gazebo,
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(use_robot_state_pub)
    )

    # Controllers
    load_controllers_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share_bringup, 'launch', 'load_ros2_controllers.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(load_controllers)
    )

    # Gazebo resources
    set_env_vars_resources = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gazebo_models_path)

    # Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments=[('gz_args', [' -r -s -v 4 ', world_path])]
    )

    # Gazebo client
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-g ']}.items(),
        condition=IfCondition(PythonExpression(['not ', headless]))
    )

    # Bridges
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': default_ros_gz_bridge_config}],
        output='screen'
    )

    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/cam_1/image'],
        remappings=[('/cam_1/image', '/cam_1/color/image_raw')]
    )

    # Spawn robot
    spawn_robot_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', robot_name,
            '-allow_renaming', 'true',
            '-x', x, '-y', y, '-z', z,
            '-R', roll, '-P', pitch, '-Y', yaw
        ]
    )

    # -----------------------------
    # SLAM (slam_toolbox)
    # -----------------------------
    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('slam_toolbox').find('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            # NOTE: slam_toolbox launch expects 'params_file' on newer distros;
            # some older examples use 'slam_params_file'.
            # Jazzy's online_async_launch.py uses 'params_file'.
            'params_file': params_file
        }.items()
    )

    # -----------------------------
    # Nav2 (NO AMCL)
    # -----------------------------
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('nav2_bringup').find('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart
        }.items()
    )

    # -----------------------------
    # cmd_vel relay (Twist -> TwistStamped)
    # -----------------------------
    cmd_vel_relay_cmd = Node(
        package='yahboom_rosmaster_navigation',
        executable='cmd_vel_relay',
        name='cmd_vel_relay',
        output='screen',
        parameters=[{
            'in_topic': '/cmd_vel',  # what Nav2 controller server publishes
            'out_topic': '/mecanum_drive_controller/cmd_vel',  # what ros2_control consumes
            'frame_id': 'base_link',
            'use_sim_time': use_sim_time,
        }]
    )

    # Build launch description
    ld = LaunchDescription()

    # Declare args
    ld.add_action(declare_enable_odom_tf_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_jsp_gui_cmd)
    ld.add_action(declare_load_controllers_cmd)
    ld.add_action(declare_use_gazebo_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_params_cmd)
    ld.add_action(declare_autostart_cmd)

    # Pose args
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_z_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)

    # Start simulation
    ld.add_action(set_env_vars_resources)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    # Core TF/robot_description
    ld.add_action(robot_state_publisher_cmd)

    # Bridges
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(start_gazebo_ros_image_bridge_cmd)

    # Spawn
    ld.add_action(spawn_robot_cmd)

    # Controllers after a bit
    ld.add_action(TimerAction(period=3.0, actions=[load_controllers_cmd]))

    # SLAM after TF + /scan stable
    ld.add_action(TimerAction(period=6.0, actions=[slam_cmd]))

    # Nav2 after slam_toolbox is publishing map->odom reliably
    ld.add_action(TimerAction(period=12.0, actions=[nav2_cmd]))

    # cmd_vel relay after Nav2 is up
    ld.add_action(TimerAction(period=14.0, actions=[cmd_vel_relay_cmd]))

    return ld