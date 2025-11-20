#!/usr/bin/env python3
"""
Path Visualization Node for ROS2
Collects and visualizes the entire path-planning and execution pipeline in live windows
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
import numpy as np
import matplotlib.pyplot as plt
import math
import os
from datetime import datetime
import threading

plt.ion()


class PathVisualizer(Node):
    """ROS2 Node for visualizing path planning and execution pipeline"""
    
    def __init__(self):
        super().__init__('path_visualizer')
        
        self.original_waypoints = []
        self.smooth_path_data = []
        self.trajectory_data = []
        self.odometry_data = []
        self.cmd_vel_data = []
        
        self.error_timestamps = []
        self.tracking_errors = []
        
        self.start_time = None
        self.data_lock = threading.Lock()
        
        self.setup_plots()
        
        self.smooth_path_sub = self.create_subscription(
            Path, '/smooth_path', self.smooth_path_callback, 10)
        
        self.trajectory_sub = self.create_subscription(
            Path, '/trajectory', self.trajectory_callback, 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.plot_timer = self.create_timer(0.1, self.update_plots)  # 10 Hz
        
        self.get_logger().info('Path Visualizer initialized - Close plot window to shutdown')
    
    def setup_plots(self):
        """Initialize matplotlib figures and axes"""
        # Main display figure
        self.fig1, self.ax1 = plt.subplots(1, 1, figsize=(10, 10))
        self.fig1.suptitle('Path Planning and Execution Pipeline', fontsize=14, fontweight='bold')
        self.ax1.set_title('Complete Path Visualization', fontsize=12, fontweight='bold')
        
        # Background error tracking figure (for saving only)
        self.fig_error = plt.figure(figsize=(10, 6))
        self.ax2 = self.fig_error.add_subplot(1, 1, 1)
        self.fig_error.suptitle('Tracking Error Analysis', fontsize=14, fontweight='bold')
        self.ax2.set_title('Tracking Error Over Time', fontsize=12, fontweight='bold')
        
        plt.close(self.fig_error)  # Keep hidden
        
        plt.figure(self.fig1.number)
        plt.show(block=False)
        
        self.init_plot_objects()
    
    def init_plot_objects(self):
        """Initialize plot objects for efficient updating"""
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
        """Store smooth path data and extract waypoints"""
        with self.data_lock:
            if self.start_time is None:
                self.start_time = self.get_clock().now()
            
            path_points = []
            for pose in msg.poses:
                x = pose.pose.position.x
                y = pose.pose.position.y
                yaw = self.quaternion_to_yaw(
                    pose.pose.orientation.x, pose.pose.orientation.y,
                    pose.pose.orientation.z, pose.pose.orientation.w)
                path_points.append((x, y, yaw))
            
            self.smooth_path_data = path_points
            
            # Extract waypoints from smooth path (first time only)
            if not self.original_waypoints and len(path_points) > 10:
                step = len(path_points) // 5  # Assume 5 original waypoints
                waypoint_indices = [0, step, 2*step, 3*step, len(path_points)-1]
                self.original_waypoints = [(path_points[i][0], path_points[i][1]) 
                                         for i in waypoint_indices if i < len(path_points)]
                self.get_logger().info(f'Extracted {len(self.original_waypoints)} waypoints')
    
    def trajectory_callback(self, msg: Path):
        """Store trajectory data"""
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
                    pose.pose.orientation.x, pose.pose.orientation.y,
                    pose.pose.orientation.z, pose.pose.orientation.w)
                trajectory_points.append((x, y, yaw, timestamp))
            
            self.trajectory_data = trajectory_points
    
    def odom_callback(self, msg: Odometry):
        """Store odometry data"""
        with self.data_lock:
            if self.start_time is None:
                self.start_time = self.get_clock().now()
            
            current_time = self.get_clock().now()
            timestamp = (current_time - self.start_time).nanoseconds / 1e9
            
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = self.quaternion_to_yaw(
                msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
            
            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            wz = msg.twist.twist.angular.z
            
            self.odometry_data.append((timestamp, x, y, yaw, vx, vy, wz))
    
    def cmd_vel_callback(self, msg: Twist):
        """Store command velocity data"""
        with self.data_lock:
            if self.start_time is None:
                self.start_time = self.get_clock().now()
            
            current_time = self.get_clock().now()
            timestamp = (current_time - self.start_time).nanoseconds / 1e9
            
            vx = msg.linear.x
            wz = msg.angular.z
            
            self.cmd_vel_data.append((timestamp, vx, wz))
    
    def update_combined_plot(self):
        """Update main visualization and error tracking"""
        with self.data_lock:
            self.update_main_plot()
            self.update_error_tracking()
    
    def update_main_plot(self):
        """Update main visualization plot"""
        self.ax1.clear()
        
        has_data = (self.original_waypoints or self.smooth_path_data or 
                   self.trajectory_data or self.odometry_data)
        
        if not has_data:
            self.ax1.text(0.5, 0.5, 'Waiting for data...\nStart nodes to see visualization', 
                        transform=self.ax1.transAxes, fontsize=14, ha='center', va='center',
                        bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.8))
            self.ax1.set_xlim(-1, 4)
            self.ax1.set_ylim(-1, 2)
        else:
            # Original waypoints
            if self.original_waypoints:
                wp_x = [wp[0] for wp in self.original_waypoints]
                wp_y = [wp[1] for wp in self.original_waypoints]
                self.ax1.scatter(wp_x, wp_y, c='red', s=100, marker='o', 
                              label='Original Waypoints', zorder=5, edgecolors='black')
                self.ax1.plot(wp_x, wp_y, 'r--', alpha=0.5, linewidth=1)
            
            # Smoothed path
            if self.smooth_path_data:
                smooth_x = [pt[0] for pt in self.smooth_path_data]
                smooth_y = [pt[1] for pt in self.smooth_path_data]
                self.ax1.plot(smooth_x, smooth_y, 'blue', linewidth=2, 
                           label='Smoothed Path', alpha=0.7)
            
            # Generated trajectory with direction arrows
            if self.trajectory_data:
                traj_x = [pt[0] for pt in self.trajectory_data]
                traj_y = [pt[1] for pt in self.trajectory_data]
                self.ax1.plot(traj_x, traj_y, 'green', linewidth=3, 
                           label='Generated Trajectory', alpha=0.8)
                
                if len(self.trajectory_data) > 10:
                    step = len(self.trajectory_data) // 10
                    for i in range(0, len(self.trajectory_data), step):
                        x, y, yaw, _ = self.trajectory_data[i]
                        dx = 0.1 * math.cos(yaw)
                        dy = 0.1 * math.sin(yaw)
                        self.ax1.arrow(x, y, dx, dy, head_width=0.05, head_length=0.03, 
                                    fc='green', ec='green', alpha=0.6)
            
            # Actual robot trajectory
            if len(self.odometry_data) > 1:
                odom_x = [pt[1] for pt in self.odometry_data]
                odom_y = [pt[2] for pt in self.odometry_data]
                self.ax1.plot(odom_x, odom_y, 'magenta', linewidth=2, 
                           label='Actual Trajectory')
                
                if self.odometry_data:
                    start_x, start_y = self.odometry_data[0][1], self.odometry_data[0][2]
                    end_x, end_y = self.odometry_data[-1][1], self.odometry_data[-1][2]
                    
                    self.ax1.scatter(start_x, start_y, c='green', s=150, marker='^', 
                                  label='Start Position', zorder=6, edgecolors='black')
                    self.ax1.scatter(end_x, end_y, c='blue', s=150, marker='s', 
                                  label='Current Position', zorder=6, edgecolors='black')
            
        self.ax1.set_xlabel('X Position (m)', fontsize=12)
        self.ax1.set_ylabel('Y Position (m)', fontsize=12)
        if has_data:
            self.ax1.legend(loc='upper right', fontsize=10)
        
        self.ax1.set_aspect('equal', adjustable='box')
        self.ax1.grid(True, alpha=0.3)
    
    def update_error_tracking(self):
        """Update error tracking data for background saving"""
        if self.trajectory_data and len(self.odometry_data) > 0:
            current_error = self.get_current_tracking_error()
            
            if self.start_time:
                current_time = self.get_clock().now()
                timestamp = (current_time - self.start_time).nanoseconds / 1e9
                self.error_timestamps.append(timestamp)
                self.tracking_errors.append(current_error)
                
                # Memory management: keep last 1000 points
                if len(self.tracking_errors) > 1000:
                    self.error_timestamps = self.error_timestamps[-1000:]
                    self.tracking_errors = self.tracking_errors[-1000:]
    
    def update_error_plot(self):
        """Update tracking error plot for background figure"""
        self.ax2.clear()
        
        if len(self.tracking_errors) < 2:
            self.ax2.text(0.5, 0.5, 'Collecting error data...\nGraph will appear as robot moves', 
                        transform=self.ax2.transAxes, fontsize=12, ha='center', va='center',
                        bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))
            self.ax2.set_xlim(0, 10)
            self.ax2.set_ylim(0, 1)
        else:
            self.ax2.plot(self.error_timestamps, self.tracking_errors, 'red', linewidth=2, 
                         label='Tracking Error', alpha=0.8)
            
            if self.tracking_errors:
                self.ax2.scatter(self.error_timestamps[-1], self.tracking_errors[-1], 
                               c='red', s=50, zorder=5, edgecolors='black')
            
            if self.tracking_errors:
                max_error = max(self.tracking_errors)
                self.ax2.set_ylim(0, max(max_error * 1.1, 0.1))
        
        self.ax2.set_xlabel('Time (s)', fontsize=12)
        self.ax2.set_ylabel('Tracking Error (m)', fontsize=12)
        self.ax2.set_title('Tracking Error Over Time', fontsize=12, fontweight='bold')
        self.ax2.grid(True, alpha=0.3)
        if len(self.tracking_errors) >= 2:
            self.ax2.legend(loc='best', fontsize=10)
    
    def get_current_tracking_error(self):
        """Calculate tracking error between robot and closest trajectory point"""
        if not self.trajectory_data or len(self.odometry_data) == 0:
            return 0.0
        
        current_odom = self.odometry_data[-1]
        current_x, current_y = current_odom[1], current_odom[2]
        
        traj_points = np.array([(pt[0], pt[1]) for pt in self.trajectory_data])
        distances = np.sqrt((traj_points[:, 0] - current_x)**2 + (traj_points[:, 1] - current_y)**2)
        
        return np.min(distances)

    def update_plots(self):
        """Update live visualization"""
        try:
            if not plt.fignum_exists(self.fig1.number):
                self.get_logger().info('Plot window closed, saving and shutting down...')
                self.save_graphs()
                rclpy.shutdown()
                return
            
            self.update_combined_plot()
            
            self.fig1.canvas.draw()
            self.fig1.canvas.flush_events()
            
            plt.pause(0.001)
                
        except Exception as e:
            self.get_logger().error(f'Plot update error: {str(e)}')
    
    def save_graphs(self):
        """Save visualization and error plots as images"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_dir = "/home/ashish/ros2/10x_Assignment/visualization_output"
            os.makedirs(output_dir, exist_ok=True)
            
            # Final update before saving
            with self.data_lock:
                self.update_main_plot()
                self.update_error_plot()
            
            # Save both plots
            main_filename = os.path.join(output_dir, f"path_visualization_{timestamp}.png")
            self.fig1.savefig(main_filename, dpi=300, bbox_inches='tight', 
                             facecolor='white', edgecolor='none')
            
            error_filename = os.path.join(output_dir, f"tracking_error_{timestamp}.png")
            self.fig_error.savefig(error_filename, dpi=300, bbox_inches='tight',
                                  facecolor='white', edgecolor='none')
            
            self.get_logger().info('Graphs saved:')
            self.get_logger().info(f'  Main: {main_filename}')
            self.get_logger().info(f'  Error: {error_filename}')
            
        except Exception as e:
            self.get_logger().error(f'Save error: {str(e)}')


def main(args=None):
    """Main function to run the path visualizer node"""
    rclpy.init(args=args)
    
    try:
        visualizer = PathVisualizer()
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()