#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Package Directory
    pkg_quadrotor_gp_mpc = FindPackageShare(package='quadrotor_gp_mpc').find('quadrotor_gp_mpc')
    
    # Path to URDF file
    urdf_file = os.path.join(pkg_quadrotor_gp_mpc, 'urdf', 'quadrotor.urdf')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # Robot State Publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['cat ', urdf_file])
        }])

    # RViz for visualization
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen')

    # TF publisher to connect our dynamics to the robot model
    tf_publisher_cmd = Node(
        package='quadrotor_gp_mpc',
        executable='tf_publisher',
        name='tf_publisher',
        output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)

    # Add the actions
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(tf_publisher_cmd)

    return ld