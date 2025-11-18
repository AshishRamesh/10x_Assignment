#!/usr/bin/env python3
"""
Visualization-only launch file for bot_ctrl package
Starts only the path visualization node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Generate launch description for visualization only
    
    Returns:
        LaunchDescription with visualization node
    """
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    # Get launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    
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
        path_visualizer_node,
    ])