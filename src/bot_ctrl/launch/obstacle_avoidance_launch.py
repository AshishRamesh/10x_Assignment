#!/usr/bin/env python3
"""
Launch file for obstacle avoidance navigation system
Launches nodes in specific order: obstacle_avoid -> smoother_obstacle -> planner -> controller -> visualizer
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for obstacle avoidance navigation"""
    
    return LaunchDescription([
        # 1. Obstacle Avoid Node - A* pathfinding with obstacle avoidance
        Node(
            package='bot_ctrl',
            executable='obstacle_avoid',
            name='obstacle_avoid',
            output='screen',
            parameters=[],
        ),
        
        # 2. Smoother Obstacle Node - smooths A* paths for navigation
        Node(
            package='bot_ctrl',
            executable='smoother_obstacle',
            name='smoother_obstacle',
            output='screen',
            parameters=[],
        ),
        
        # 3. Planner Node - generates trajectory from smooth path
        Node(
            package='bot_ctrl',
            executable='planner',
            name='planner',
            output='screen',
            parameters=[],
        ),
        
        # 4. Controller Node - tracks trajectory and sends cmd_vel
        Node(
            package='bot_ctrl',
            executable='controller',
            name='controller',
            output='screen',
            parameters=[],
        ),
        
        # 5. Visualizer Node - displays path planning pipeline
        Node(
            package='bot_ctrl',
            executable='visualizer',
            name='visualizer',
            output='screen',
            parameters=[],
        ),
    ])
