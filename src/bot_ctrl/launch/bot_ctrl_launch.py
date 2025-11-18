#!/usr/bin/env python3
"""
Launch file for bot_ctrl package
Starts all nodes: path_smoother, trajectory_generator, trajectory_tracker
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Generate launch description for bot_ctrl package
    
    Returns:
        LaunchDescription with all nodes
    """
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    # Get launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Path Smoother Node
    path_smoother_node = Node(
        package='bot_ctrl',
        executable='path_smoother',
        name='path_smoother',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            # Add any necessary topic remappings here
        ]
    )
    
    # Trajectory Generator Node
    trajectory_generator_node = Node(
        package='bot_ctrl',
        executable='trajectory_generator',
        name='trajectory_generator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            # Add any necessary topic remappings here
        ]
    )
    
    # Trajectory Tracker Node
    trajectory_tracker_node = Node(
        package='bot_ctrl',
        executable='trajectory_tracker',
        name='trajectory_tracker',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            # Add any necessary topic remappings here
        ]
    )
    
    # Path Visualizer Node
    path_visualizer_node = Node(
        package='bot_ctrl',
        executable='path_visualizer',
        name='path_visualizer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            # Add any necessary topic remappings here
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        path_smoother_node,
        trajectory_generator_node,
        trajectory_tracker_node,
        path_visualizer_node,
    ])