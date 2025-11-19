#!/usr/bin/env python3
"""
Launch file for hardcoded waypoints navigation system
Launches nodes in specific order: planner -> controller -> visualizer -> smoother
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for hardcoded waypoints navigation"""
    
    return LaunchDescription([
        # 1. Planner Node - generates trajectory from smooth path
        Node(
            package='bot_ctrl',
            executable='planner',
            name='planner',
            output='screen',
            parameters=[],
        ),
        
        # 2. Controller Node - tracks trajectory and sends cmd_vel
        Node(
            package='bot_ctrl',
            executable='controller',
            name='controller',
            output='screen',
            parameters=[],
        ),
        
        # 3. Visualizer Node - displays path planning pipeline
        Node(
            package='bot_ctrl',
            executable='visualizer',
            name='visualizer',
            output='screen',
            parameters=[],
        ),
        
        # 4. Smoother Clicked Node - generates smooth path from clicked waypoints
        Node(
            package='bot_ctrl',
            executable='smoother_clicked',
            name='smoother_clicked',
            output='screen',
            parameters=[],
        ),
    ])
