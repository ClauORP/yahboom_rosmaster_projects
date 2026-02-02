#!/usr/bin/env python3
"""
Launch Gazebo simulation with the rosmasterx3.
RViz is not included.
"""
# Standard library: used mainly to join file paths
import os

# Core launch API (build a LaunchDescription, add actions to it)
from launch import LaunchDescription

# Common launch "actions":
# - DeclareLaunchArgument: let the user configure things from the command line
# - IncludeLaunchDescription: include another launch file (like "sub-launch")
# - AppendEnvironmentVariable: add to env vars for the launched processes
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription

# "IfCondition" lets us only run an action if a condition is true
from launch.conditions import IfCondition

# Needed to include another Python launch file
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Substitutions are "values resolved at launch time" (not at Python import time)
# - LaunchConfiguration('x') reads launch args / parameters
# - PathJoinSubstitution builds paths safely
# - PythonExpression lets us do small boolean logic like "not headless"
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

# ROS-specific actions (launch a ROS node)
from launch_ros.actions import Node

# FindPackageShare finds the "share/" folder of a ROS package (installed or in workspace)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Constants for paths to different files and folders
    package_name_gazebo = 'yahboom_rosmaster_gazebo'
    package_name_description = 'yahboom_rosmaster_description'
    package_name_bringup = 'yahboom_rosmaster_bringup'
    package_name_worlds = 'yahboom_rosmaster_projects'

    # Default name for the spawned robot entity in Gazebo
    default_robot_name = 'rosmaster_x3'
    
    # Folders/filenames inside package share directories
    gazebo_models_path = 'models'
    default_world_file = 'rosmaster_home.world'
    gazebo_worlds_path = 'worlds'
    ros_gz_bridge_config_file_path = 'config/ros_gz_bridge.yaml'

    # Set the path to different files and folders
    pkg_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    pkg_share_gazebo = FindPackageShare(package=package_name_gazebo).find(package_name_gazebo)
    pkg_share_worlds = FindPackageShare(package=package_name_worlds).find(package_name_worlds)
    pkg_share_description = FindPackageShare(package=package_name_description).find(package_name_description)
    pkg_share_bringup = FindPackageShare(package=package_name_bringup).find(package_name_bringup)

    # Build absolute paths to files we need (bridge config, Gazebo models folder)
    default_ros_gz_bridge_config_file_path = os.path.join(pkg_share_gazebo, ros_gz_bridge_config_file_path)
    gazebo_models_path = os.path.join(pkg_share_worlds, gazebo_models_path)

    # Launch configuration variables
    # LaunchConfiguration('something') reads the value of the launch argument "something"
    enable_odom_tf = LaunchConfiguration('enable_odom_tf')
    headless = LaunchConfiguration('headless')
    jsp_gui = LaunchConfiguration('jsp_gui')
    load_controllers = LaunchConfiguration('load_controllers')
    robot_name = LaunchConfiguration('robot_name')
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world_file')

    # Build the world file absolute path:
    #   <pkg_share_worlds>/worlds/<world_file>
    world_path = PathJoinSubstitution([
        pkg_share_worlds,
        gazebo_worlds_path,
        world_file
    ])

    # Set the pose configuration variables
    # These are the position/orientation where the robot will appear in the Gazebo world
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    # Declare the launch arguments (what users can change on the CLI)
    declare_enable_odom_tf_cmd = DeclareLaunchArgument(
        name='enable_odom_tf',
        default_value='true',
        choices=['true', 'false'],
        description='Whether to enable odometry transform broadcasting via ROS 2 Control')

    declare_headless_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient (visualization)')

    declare_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value=default_robot_name,
        description='The name for the robot')

    declare_load_controllers_cmd = DeclareLaunchArgument(
        name='load_controllers',
        default_value='true',
        description='Flag to enable loading of ROS 2 controllers')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='true',
        description='Flag to enable robot state publisher')

    declare_jsp_gui_cmd = DeclareLaunchArgument(
        name='jsp_gui',
        default_value='false',
        description='Flag to enable joint_state_publisher_gui')

    declare_use_gazebo_cmd = DeclareLaunchArgument(
        name='use_gazebo',
        default_value='true',
        description='Flag to enable Gazebo')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_world_cmd = DeclareLaunchArgument(
        name='world_file',
        default_value=default_world_file,
        description='World file name (e.g., empty.world, house.world, pick_and_place_demo.world)')

    declare_x_cmd = DeclareLaunchArgument(
        name='x',
        default_value='1.0',
        description='x component of initial position, meters')

    declare_y_cmd = DeclareLaunchArgument(
        name='y',
        default_value='0.0',
        description='y component of initial position, meters')

    declare_z_cmd = DeclareLaunchArgument(
        name='z',
        default_value='0.05',
        description='z component of initial position, meters')

    declare_roll_cmd = DeclareLaunchArgument(
        name='roll',
        default_value='0.0',
        description='roll angle of initial orientation, radians')

    declare_pitch_cmd = DeclareLaunchArgument(
        name='pitch',
        default_value='0.0',
        description='pitch angle of initial orientation, radians')

    declare_yaw_cmd = DeclareLaunchArgument(
        name='yaw',
        default_value='0.0',
        description='yaw angle of initial orientation, radians')

    # robot_state_publisher publishes the TF tree and /robot_description (URDF)
    # This is required so that the Gazebo spawner can read /robot_description
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share_description, 'launch', 'robot_state_publisher.launch.py')
        ]),
        launch_arguments={
            'enable_odom_tf': enable_odom_tf,
            'jsp_gui': jsp_gui,
            'use_rviz': 'false',
            'use_gazebo': use_gazebo,
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(use_robot_state_pub))

    # Loads ros2_control controllers (mecanum drive controller, joint_state_broadcaster, etc.)
    load_controllers_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share_bringup, 'launch', 'load_ros2_controllers.launch.py')
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(load_controllers))

    # Environment variables for Gazebo resource lookup
    # Gazebo needs to find models/materials. Setting GZ_SIM_RESOURCE_PATH helps it locate them
    # If models are not found, you'll see missing meshes/textures in Gazebo
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        gazebo_models_path)

    # Start Gazebo server (physics): runs the simulation and loads the selected world.
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments=[('gz_args', [' -r -s -v 4 ', world_path])])

    # Start Gazebo client (GUI): the GUI window
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-g ']}.items(),
        condition=IfCondition(PythonExpression(['not ', headless])))

    # Bridge ROS topics and Gazebo messages for establishing communication
    # This is where /clock, /cmd_vel, lidar topics, etc. are typically bridged
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': default_ros_gz_bridge_config_file_path,}],
        output='screen')

    # It bridges a Gazebo camera topic to a ROS image topic
    # Includes optimizations to minimize latency and bandwidth when streaming image data
    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/cam_1/image'],
        remappings=[('/cam_1/image', '/cam_1/color/image_raw')])

    # Spawn the robot entity into Gazebo
    # This reads the URDF from /robot_description and creates the model in Gazebo
    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', robot_name,
            '-allow_renaming', 'true',
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw])
    
    # Bridges Gazebo dynamic pose info to a ROS PoseArray.
    gz_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/p2_world/dynamic_pose/info'
            '@geometry_msgs/msg/PoseArray'
            '[gz.msgs.Pose_V'],
        output='screen')
    
    # Ground-truth pose pipeline (Gazebo -> ROS)
    # Custom node that listens to the PoseArray and outputs ONLY this robot's pose.
    ground_truth_extractor = Node(
        package='yahboom_rosmaster_projects',
        executable='ground_truth_extractor.py',
        name='ground_truth_extractor',
        output='screen',
        parameters=[{
            'in_topic': '/world/p2_world/dynamic_pose/info',
            'out_topic': '/ground_truth_pose',
            'frame_id': 'world',}])
    
    # Build the LaunchDescription and add actions in order
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_enable_odom_tf_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_jsp_gui_cmd)
    ld.add_action(declare_load_controllers_cmd)
    ld.add_action(declare_use_gazebo_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)

    # Add pose arguments
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_z_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)

    # Add the actions to the launch description
    # Start the simulation + robot system
    ld.add_action(set_env_vars_resources)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(load_controllers_cmd)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(start_gazebo_ros_image_bridge_cmd)
    ld.add_action(start_gazebo_ros_spawner_cmd)

    ld.add_action(gz_pose_bridge)
    ld.add_action(ground_truth_extractor)
    
    # Launch system returns this description to ros2 launch
    return ld
