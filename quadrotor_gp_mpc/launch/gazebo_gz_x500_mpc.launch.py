#!/usr/bin/env python3
"""
Gazebo simulation with gz_x500 drone and MPC/GP controllers.

This launch file:
1. Starts Gazebo with gz_x500 quadrotor model
2. Launches MPC controller node
3. Launches Gaussian Process node
4. Provides bridge between Gazebo and ROS2
5. Displays real-time control performance metrics
"""

import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the launch description for Gazebo with MPC and GP controllers."""
    
    # Get package directories
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    pkg_quadrotor_gp_mpc = FindPackageShare(package='quadrotor_gp_mpc').find('quadrotor_gp_mpc')
    
    # Get paths
    urdf_file = os.path.join(pkg_quadrotor_gp_mpc, 'urdf', 'quadrotor.urdf')
    worlds_path = os.path.join(pkg_quadrotor_gp_mpc, 'worlds')
    
    # Gazebo world file for x500
    gazebo_world = os.path.join(worlds_path, 'gz_x500_world.sdf')
    if not os.path.exists(gazebo_world):
        # Fallback to default empty world
        gazebo_world = os.path.join(worlds_path, 'empty.sdf')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    headless = LaunchConfiguration('headless', default='false')
    verbose = LaunchConfiguration('verbose', default='false')
    
    # Initial drone position
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.5')
    
    # Control parameters
    mpc_enabled = LaunchConfiguration('mpc_enabled', default='true')
    gp_enabled = LaunchConfiguration('gp_enabled', default='true')
    control_rate = LaunchConfiguration('control_rate', default='100')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time')
    
    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode (no GUI)')
    
    declare_verbose_cmd = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Enable verbose output')
    
    declare_x_pose_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Initial X position of the drone')
    
    declare_y_pose_cmd = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Initial Y position of the drone')
    
    declare_z_pose_cmd = DeclareLaunchArgument(
        'z_pose',
        default_value='0.5',
        description='Initial Z position of the drone')
    
    declare_mpc_enabled_cmd = DeclareLaunchArgument(
        'mpc_enabled',
        default_value='true',
        description='Enable MPC controller')
    
    declare_gp_enabled_cmd = DeclareLaunchArgument(
        'gp_enabled',
        default_value='true',
        description='Enable Gaussian Process learning')
    
    declare_control_rate_cmd = DeclareLaunchArgument(
        'control_rate',
        default_value='100',
        description='Control loop rate in Hz')
    
    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': gazebo_world,
            'verbose': verbose
        }.items()
    )
    
    # Start Gazebo client (GUI)
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=launch.conditions.UnlessCondition(headless)
    )
    
    # Robot State Publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf_file])
        }]
    )
    
    # Spawn x500 drone in Gazebo
    spawn_x500_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'gz_x500',
            '-file', urdf_file,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', '0.0'
        ],
        output='screen'
    )
    
    # MPC Controller Node
    mpc_controller_cmd = Node(
        package='quadrotor_gp_mpc',
        executable='mpc_controller_node',
        name='mpc_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'control_rate': control_rate,
            'enabled': mpc_enabled,
            'horizon': 20,
            'dt': 0.01,
            'max_thrust': 2.0,
            'max_torque': 0.1
        }],
        remappings=[
            ('/state', '/gz_x500/state'),
            ('/control_input', '/gz_x500/control_input'),
            ('/reference_trajectory', '/reference_trajectory')
        ]
    )
    
    # Gaussian Process Node
    gp_node_cmd = Node(
        package='quadrotor_gp_mpc',
        executable='gaussian_process_node',
        name='gaussian_process',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'enabled': gp_enabled,
            'kernel': 'rbf',
            'length_scale': 0.5,
            'sigma': 0.1,
            'noise_variance': 0.01
        }],
        remappings=[
            ('/training_data', '/gz_x500/training_data'),
            ('/predictions', '/gz_x500/gp_predictions')
        ]
    )
    
    # Gazebo Bridge (connects ROS2 to Gazebo)
    gazebo_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            # Subscribe to Gazebo topics and publish to ROS2
            '/gz_x500/state@std_msgs/String@gz.msgs.StringMsg',
            '/gz_x500/control_input@std_msgs/Float64MultiArray@gz.msgs.Float64_V',
            '/clock@rosgraph_msgs/Clock[gz.msgs.Clock'
        ]
    )
    
    # Performance metrics node (optional)
    metrics_node_cmd = Node(
        package='quadrotor_gp_mpc',
        executable='performance_metrics_node',
        name='performance_metrics',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'log_interval': 1.0,
            'save_results': True
        }]
    )
    
    # Reference trajectory generator (optional)
    reference_trajectory_cmd = Node(
        package='quadrotor_gp_mpc',
        executable='reference_trajectory_node',
        name='reference_trajectory_generator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'trajectory_type': 'setpoint',  # or 'circular', 'figure8', etc.
            'setpoint_x': 0.0,
            'setpoint_y': 0.0,
            'setpoint_z': 1.0,
            'reference_rate': 50
        }]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add declare actions
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_verbose_cmd)
    ld.add_action(declare_x_pose_cmd)
    ld.add_action(declare_y_pose_cmd)
    ld.add_action(declare_z_pose_cmd)
    ld.add_action(declare_mpc_enabled_cmd)
    ld.add_action(declare_gp_enabled_cmd)
    ld.add_action(declare_control_rate_cmd)
    
    # Add Gazebo actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    
    # Add ROS2 nodes
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_x500_cmd)
    
    # Add controller nodes
    ld.add_action(mpc_controller_cmd)
    ld.add_action(gp_node_cmd)
    
    # Add bridge and optional nodes
    ld.add_action(gazebo_bridge_cmd)
    ld.add_action(metrics_node_cmd)
    ld.add_action(reference_trajectory_cmd)
    
    return ld
