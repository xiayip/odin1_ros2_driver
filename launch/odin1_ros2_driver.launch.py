#!/usr/bin/env python3
"""
Launch file for Odin1 ROS2 Driver

This launch file starts the Odin1 ROS2 driver node with configurable parameters
for controlling various sensor outputs (RGB, IMU, odometry, point clouds, etc.).
"""

import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


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

    return LaunchDescription([
        odin1_driver_node,
    ])
