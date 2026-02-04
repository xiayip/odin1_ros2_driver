#!/usr/bin/env python3
"""
Launch file for Odin1 ROS2 Driver

This launch file starts the Odin1 ROS2 driver node with configurable parameters
for controlling various sensor outputs (RGB, IMU, odometry, point clouds, etc.).
"""

import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    # Package directory
    pkg_share = FindPackageShare('odin1_ros2_driver')

    config_file_path = PathJoinSubstitution([pkg_share, 'config', 'control_command.yaml'])
    # Odin1 ROS2 Driver Node
    odin1_driver_node = Node(
        package='odin1_ros2_driver',
        executable='odin1_ros2_driver_node',
        name='odin1_ros2_driver',
        parameters=[config_file_path],
        output='screen',
    )
    
    # Get the URDF file path
    urdf_file_path = PathJoinSubstitution(
        [pkg_share, 'description', 'odin1_description.urdf']
    )
    robot_description_content = Command(['cat ', urdf_file_path])
    robot_description = {"robot_description": robot_description_content}
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    # run rviz2 if needed
    rviz_arg = LaunchConfiguration('rviz', default='false')
    rviz_config_file = PathJoinSubstitution(
        [pkg_share, 'rviz', 'odin_ros2.rviz']
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[robot_description],
        condition=IfCondition(rviz_arg),
    )

    return LaunchDescription([
        odin1_driver_node,
        robot_state_publisher_node,
        rviz_node,
    ])
