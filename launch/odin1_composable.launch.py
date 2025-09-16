#!/usr/bin/env python3
"""
Composable node launch file for Odin1 ROS2 Driver

This launch file runs the Odin1 driver as a composable node for better performance
by reducing inter-process communication overhead.
"""

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    pkg_share = FindPackageShare('odin1_ros2_driver')

    config_file_path = PathJoinSubstitution([pkg_share, 'config', 'control_command.yaml'])
    
    # Composable node container
    container = ComposableNodeContainer(
        name='odin1_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='odin1_ros2_driver',
                plugin='odin1_ros2_driver::Odin1Driver',
                name='odin1_ros2_driver',
                parameters=[config_file_path],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        container,
    ])
