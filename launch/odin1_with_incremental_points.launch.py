#!/usr/bin/env python3
"""
Launch file for Odin1 driver with incremental map builder

This composable launch runs both the Odin1 driver and incremental map node
in the same container for efficient intra-process communication.
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    pkg_share = FindPackageShare('odin1_ros2_driver')
    config_file_path = PathJoinSubstitution([pkg_share, 'config', 'control_command.yaml'])
    
    # Composable node container with both nodes
    container = ComposableNodeContainer(
        name='odin1_mapping_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # Odin1 driver
            ComposableNode(
                package='odin1_ros2_driver',
                plugin='odin1_ros2_driver::Odin1Driver',
                name='odin1_ros2_driver',
                parameters=[config_file_path],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # Incremental map builder
            ComposableNode(
                package='odin1_ros2_driver',
                plugin='odin1_ros2_driver::IncrementalMapNode',
                name='incremental_map_node',
                parameters=[{
                    'voxel_leaf_size': 0.01,          # 1cm voxel grid
                    'octree_resolution': 0.02,          # 2cm octree resolution
                    'novelty_threshold': 0.02,         # 2cm radius for novelty check
                    'max_accumulated_points': 1000000, # 1M point limit
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )
    
    # Get the URDF file path
    rviz_arg = LaunchConfiguration('rviz', default='false')
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
        condition=IfCondition(rviz_arg),
    )
    
    # run rviz2 if needed
    rviz_config_file = PathJoinSubstitution(
        [pkg_share, 'rviz', 'odin_ros2_Increment_pl.rviz']
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
        container,
        robot_state_publisher_node,
        rviz_node,
    ])
