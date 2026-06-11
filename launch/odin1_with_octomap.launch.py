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
    base_frame_id_arg = DeclareLaunchArgument(
        'base_frame_id',
        default_value='odin1_base_link',
        description='Robot base frame used by octomap (must exist in the TF tree)'
    )
    queue_size_arg = DeclareLaunchArgument(
        'message_filter_queue_size',
        default_value='20',
        description='TF message filter queue size for incoming clouds'
    )
    cloud_topic_arg = DeclareLaunchArgument(
        'cloud_topic', 
        default_value='/odin1/cloud_raw',
        description='Input point cloud topic'
    )
    max_range_arg = DeclareLaunchArgument(
        'max_range',
        default_value='5.0',
        description='Maximum range for sensor readings (smaller = faster raycasting)'
    )
    occupancy_min_z_arg = DeclareLaunchArgument(
        'occupancy_min_z',
        default_value='0.1',
        description='Minimum height for occupancy grid projection'
    )
    occupancy_max_z_arg = DeclareLaunchArgument(
        'occupancy_max_z',
        default_value='1.7',
        description='Maximum height for occupancy grid projection'
    )
    
    # OctoMap server node
    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        parameters=[{
            'resolution': LaunchConfiguration('resolution'),
            'frame_id': LaunchConfiguration('frame_id'),
            'base_frame_id': LaunchConfiguration('base_frame_id'),
            'sensor_model.max_range': LaunchConfiguration('max_range'),
            'occupancy_min_z': LaunchConfiguration('occupancy_min_z'),
            'occupancy_max_z': LaunchConfiguration('occupancy_max_z'),
            'message_filter_queue_size': LaunchConfiguration('message_filter_queue_size'),
        }],
        remappings=[
            ('cloud_in', LaunchConfiguration('cloud_topic')),
            ('projected_map', 'map')
        ],
        output='screen',
    )

    return LaunchDescription([
        # Launch arguments
        resolution_arg,
        frame_id_arg,
        base_frame_id_arg,
        queue_size_arg,
        cloud_topic_arg,
        max_range_arg,
        occupancy_min_z_arg,
        occupancy_max_z_arg,
        # Odin1 driver
        odin1_driver_launch,
        # OctoMap server
        octomap_server_node,
    ])
