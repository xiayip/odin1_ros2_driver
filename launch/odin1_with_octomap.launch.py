#!/usr/bin/env python3
"""
Launch file for Odin1 ROS2 Driver with OctoMap Server

This launch file starts the Odin1 driver and OctoMap server for 3D occupancy mapping.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    pkg_share = FindPackageShare('odin1_ros2_driver')
    
    # Include Odin1 driver launch
    odin1_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'odin1_ros2_driver.launch.py'])
        ]),
    )
    
    # Declare octomap parameters
    resolution_arg = DeclareLaunchArgument(
        'resolution', 
        default_value='0.05',
        description='Resolution of the OctoMap in meters'
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id', 
        default_value='map',
        description='The fixed frame for the OctoMap'
    )
    cloud_topic_arg = DeclareLaunchArgument(
        'cloud_topic', 
        default_value='/odin1/cloud_raw',
        description='Input point cloud topic'
    )
    max_range_arg = DeclareLaunchArgument(
        'max_range', 
        default_value='10.0',
        description='Maximum range for sensor readings'
    )
    
    # OctoMap server node
    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        parameters=[{
            'resolution': LaunchConfiguration('resolution'),
            'frame_id': LaunchConfiguration('frame_id'),
            'sensor_model.max_range': LaunchConfiguration('max_range'),
        }],
        remappings=[
            ('cloud_in', LaunchConfiguration('cloud_topic')),
        ],
        output='screen',
    )

    return LaunchDescription([
        # Launch arguments
        resolution_arg,
        frame_id_arg,
        cloud_topic_arg,
        max_range_arg,
        # Odin1 driver
        odin1_driver_launch,
        # OctoMap server
        octomap_server_node,
    ])
