#!/usr/bin/env python3
"""
Path Visualization Node for ROS2
Collects and visualizes the entire path-planning and execution pipeline in live windows
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.gridspec import GridSpec
from matplotlib.animation import FuncAnimation
import math
from typing import List, Tuple, Optional
import threading

# Set matplotlib to use interactive backend
plt.ion()


class PathVisualizer(Node):
    """
    ROS2 Node for visualizing path planning and execution pipeline in live windows
    """
    
    def __init__(self):
        super().__init__('path_visualizer')
        
        # Data storage - start with empty data
        self.original_waypoints = []  # Will be extracted from smooth path data
        self.smooth_path_data = []
        self.trajectory_data = []
        self.odometry_data = []
        self.cmd_vel_data = []
        
        # Timing data
        self.start_time = None
        
        # Threading lock for data access
        self.data_lock = threading.Lock()
        
        # Initialize matplotlib figures
        self.setup_plots()
        
        # Subscribers
        self.smooth_path_sub = self.create_subscription(
            Path,
            '/smooth_path',
            self.smooth_path_callback,
            10
        )
        
        self.trajectory_sub = self.create_subscription(
            Path,
            '/trajectory',
            self.trajectory_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Update timer for live plotting - 10 Hz update rate
        self.plot_timer = self.create_timer(0.1, self.update_plots)
        
        self.get_logger().info('Path Visualizer Node initialized with live plotting')
        self.get_logger().info('Close plot windows to shutdown the node')
    
    def setup_plots(self):
        """Initialize matplotlib figures and axes for live plotting"""
        # Combined path plot - single window
        self.fig1, self.ax1 = plt.subplots(figsize=(12, 8))
        self.fig1.suptitle('Path Planning and Execution Pipeline Overview', fontsize=14, fontweight='bold')
        
        # Show the figure
        plt.show(block=False)
        
        # Initialize empty line objects for efficient updating
        self.init_plot_objects()
    
    def init_plot_objects(self):
        """Initialize plot objects for efficient updating"""
        # Combined plot objects
        self.waypoint_scatter = None
        self.waypoint_line = None
        self.smooth_line = None
        self.traj_line = None
        self.tracked_line = None
        self.start_marker = None
        self.current_marker = None
        self.arrows = []
    
    def quaternion_to_yaw(self, qx: float, qy: float, qz: float, qw: float) -> float:
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def smooth_path_callback(self, msg: Path):
        """Callback for smooth path data"""
        with self.data_lock:
            if self.start_time is None:
                self.start_time = self.get_clock().now()
            
            current_time = self.get_clock().now()
            timestamp = (current_time - self.start_time).nanoseconds / 1e9
            
            path_points = []
            for pose in msg.poses:
                x = pose.pose.position.x
                y = pose.pose.position.y
                yaw = self.quaternion_to_yaw(
                    pose.pose.orientation.x,
                    pose.pose.orientation.y,
                    pose.pose.orientation.z,
                    pose.pose.orientation.w
                )
                path_points.append((x, y, yaw))
            
            self.smooth_path_data = path_points
            
            # Extract approximate waypoints from smooth path (first time only)
            if not self.original_waypoints and len(path_points) > 10:
                # Take evenly spaced points to approximate original waypoints
                step = len(path_points) // 5  # Assuming 5 original waypoints
                waypoint_indices = [0, step, 2*step, 3*step, len(path_points)-1]
                self.original_waypoints = [(path_points[i][0], path_points[i][1]) 
                                         for i in waypoint_indices if i < len(path_points)]
                self.get_logger().info(f'Extracted {len(self.original_waypoints)} waypoints from smooth path')
    
    def trajectory_callback(self, msg: Path):
        """Callback for trajectory data"""
        with self.data_lock:
            if self.start_time is None:
                self.start_time = self.get_clock().now()
            
            current_time = self.get_clock().now()
            timestamp = (current_time - self.start_time).nanoseconds / 1e9
            
            trajectory_points = []
            for pose in msg.poses:
                x = pose.pose.position.x
                y = pose.pose.position.y
                yaw = self.quaternion_to_yaw(
                    pose.pose.orientation.x,
                    pose.pose.orientation.y,
                    pose.pose.orientation.z,
                    pose.pose.orientation.w
                )
                trajectory_points.append((x, y, yaw, timestamp))
            
            self.trajectory_data = trajectory_points
    
    def odom_callback(self, msg: Odometry):
        """Callback for odometry data"""
        with self.data_lock:
            if self.start_time is None:
                self.start_time = self.get_clock().now()
            
            current_time = self.get_clock().now()
            timestamp = (current_time - self.start_time).nanoseconds / 1e9
            
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = self.quaternion_to_yaw(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            )
            
            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            wz = msg.twist.twist.angular.z
            
            self.odometry_data.append((timestamp, x, y, yaw, vx, vy, wz))
    
    def cmd_vel_callback(self, msg: Twist):
        """Callback for command velocity data"""
        with self.data_lock:
            if self.start_time is None:
                self.start_time = self.get_clock().now()
            
            current_time = self.get_clock().now()
            timestamp = (current_time - self.start_time).nanoseconds / 1e9
            
            vx = msg.linear.x
            wz = msg.angular.z
            
            self.cmd_vel_data.append((timestamp, vx, wz))
    
    def update_combined_plot(self):
        """Update the combined plot with latest data"""
        with self.data_lock:
            self.ax1.clear()
            
            # Check if we have any data to plot
            has_data = (self.original_waypoints or self.smooth_path_data or 
                       self.trajectory_data or self.odometry_data)
            
            if not has_data:
                # Show waiting message when no data is available
                self.ax1.text(0.5, 0.5, 'Waiting for data...\nStart the path planning nodes to see visualization', 
                            transform=self.ax1.transAxes, fontsize=14, ha='center', va='center',
                            bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.8))
                self.ax1.set_xlim(-1, 4)
                self.ax1.set_ylim(-1, 2)
            else:
                # Plot 1: Original waypoints (only if extracted from smooth path)
                if self.original_waypoints:
                    wp_x = [wp[0] for wp in self.original_waypoints]
                    wp_y = [wp[1] for wp in self.original_waypoints]
                    self.ax1.scatter(wp_x, wp_y, c='red', s=100, marker='o', 
                                  label='Original Waypoints', zorder=5, edgecolors='black')
                    self.ax1.plot(wp_x, wp_y, 'r--', alpha=0.5, linewidth=1)
                
                # Plot 2: Smoothed path
                if self.smooth_path_data:
                    smooth_x = [pt[0] for pt in self.smooth_path_data]
                    smooth_y = [pt[1] for pt in self.smooth_path_data]
                    self.ax1.plot(smooth_x, smooth_y, 'blue', linewidth=2, 
                               label='Smoothed Path (SciPy Splines)', alpha=0.7)
                
                # Plot 3: Generated trajectory
                if self.trajectory_data:
                    traj_x = [pt[0] for pt in self.trajectory_data]
                    traj_y = [pt[1] for pt in self.trajectory_data]
                    self.ax1.plot(traj_x, traj_y, 'green', linewidth=3, 
                               label='Generated Trajectory', alpha=0.8)
                    
                    # Add direction arrows for trajectory
                    if len(self.trajectory_data) > 10:
                        step = len(self.trajectory_data) // 10
                        for i in range(0, len(self.trajectory_data), step):
                            x, y, yaw, _ = self.trajectory_data[i]
                            dx = 0.1 * math.cos(yaw)
                            dy = 0.1 * math.sin(yaw)
                            self.ax1.arrow(x, y, dx, dy, head_width=0.05, head_length=0.03, 
                                        fc='green', ec='green', alpha=0.6)
                
                # Plot 4: Tracked trajectory (actual robot path)
                if len(self.odometry_data) > 1:
                    odom_x = [pt[1] for pt in self.odometry_data]
                    odom_y = [pt[2] for pt in self.odometry_data]
                    self.ax1.plot(odom_x, odom_y, 'magenta', linewidth=2, 
                               label='Tracked Trajectory (Actual)', linestyle='-')
                    
                    # Mark start and end positions
                    if self.odometry_data:
                        start_x, start_y = self.odometry_data[0][1], self.odometry_data[0][2]
                        end_x, end_y = self.odometry_data[-1][1], self.odometry_data[-1][2]
                        
                        self.ax1.scatter(start_x, start_y, c='green', s=150, marker='^', 
                                      label='Start Position', zorder=6, edgecolors='black')
                        self.ax1.scatter(end_x, end_y, c='red', s=150, marker='s', 
                                      label='Current Position', zorder=6, edgecolors='black')
                
                # Add statistics text box (only when we have data)
                stats_text = f"Data Points:\n"
                stats_text += f"• Waypoints: {len(self.original_waypoints)}\n"
                stats_text += f"• Smooth Path: {len(self.smooth_path_data)}\n"
                stats_text += f"• Trajectory: {len(self.trajectory_data)}\n"
                stats_text += f"• Odometry: {len(self.odometry_data)}"
                
                self.ax1.text(0.02, 0.98, stats_text, transform=self.ax1.transAxes, fontsize=9,
                            verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
            
            # Styling (always applied)
            self.ax1.set_xlabel('X Position (m)', fontsize=12)
            self.ax1.set_ylabel('Y Position (m)', fontsize=12)
            if has_data:
                self.ax1.legend(loc='best', fontsize=10)
                self.ax1.set_aspect('equal', adjustable='box')
            self.ax1.grid(True, alpha=0.3)
    

    

    
    def update_plots(self):
        """Update the live visualization plot"""
        try:
            # Check if figure is still open
            if not plt.fignum_exists(self.fig1.number):
                self.get_logger().warn('Plot window closed, shutting down...')
                rclpy.shutdown()
                return
            
            # Update the combined plot
            self.update_combined_plot()
            
            # Refresh display
            self.fig1.canvas.draw()
            self.fig1.canvas.flush_events()
            
            plt.pause(0.001)  # Small pause to allow GUI updates
                
        except Exception as e:
            self.get_logger().error(f'Error updating plot: {str(e)}')


def main(args=None):
    """Main function to run the path visualizer node"""
    rclpy.init(args=args)
    
    try:
        visualizer = PathVisualizer()
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()