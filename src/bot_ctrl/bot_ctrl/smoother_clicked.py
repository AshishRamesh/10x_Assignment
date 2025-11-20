#!/usr/bin/env python3
"""
Click-to-Navigate Waypoint Manager for ROS2
Implements FIFO waypoint queue with cubic spline path smoothing using SciPy
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PointStamped
import numpy as np
import math
from scipy.interpolate import CubicSpline, splprep, splev
from typing import List, Tuple
from collections import deque
from enum import Enum


class RobotState(Enum):
    """Robot navigation states"""
    IDLE = "idle"
    PLANNING = "planning"
    TRACKING = "tracking"


class WaypointManager(Node):
    """ROS2 Node for click-to-navigate waypoint management with path smoothing"""
    
    def __init__(self):
        super().__init__('waypoint_manager')
        
        self.waypoint_queue = deque()  # FIFO queue of (x, y) waypoints
        
        self.current_state = RobotState.IDLE
        self.current_pose = None
        self.active_waypoint = None
        
        self.num_points = 150  # Path interpolation density
        
        self.path_publisher = self.create_publisher(Path, '/smooth_path', 10)
        
        self.clicked_point_sub = self.create_subscription(
            PointStamped, '/clicked_point', self.clicked_point_callback, 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        self.state_timer = self.create_timer(0.1, self.state_machine_update)  # 10 Hz
        
        self.get_logger().info('Waypoint Manager initialized - Click points in RViz to navigate')
        self.get_logger().info(f'Path smoothing: {self.num_points} points | State: IDLE')
    
    def clicked_point_callback(self, msg: PointStamped):
        """Add clicked points from RViz to waypoint queue"""
        waypoint = (msg.point.x, msg.point.y)
        self.waypoint_queue.append(waypoint)
        
        self.get_logger().info(
            f'Added waypoint ({waypoint[0]:.2f}, {waypoint[1]:.2f}) | Queue: {len(self.waypoint_queue)}')
        
    def odom_callback(self, msg: Odometry):
        """Update current robot pose from odometry"""
        self.current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        
    def state_machine_update(self):
        """Main state machine update loop"""
        if self.current_state == RobotState.IDLE:
            if len(self.waypoint_queue) > 0 and self.current_pose is not None:
                self.active_waypoint = self.waypoint_queue.popleft()
                self.current_state = RobotState.PLANNING
                
                self.get_logger().info(
                    f'IDLE -> PLANNING: waypoint ({self.active_waypoint[0]:.2f}, {self.active_waypoint[1]:.2f})')
                
        elif self.current_state == RobotState.PLANNING:
            if self.current_pose is not None and self.active_waypoint is not None:
                self.plan_and_publish_path()
                self.current_state = RobotState.TRACKING
                
                self.get_logger().info(
                    f'PLANNING -> TRACKING: path to ({self.active_waypoint[0]:.2f}, {self.active_waypoint[1]:.2f})')
                
        elif self.current_state == RobotState.TRACKING:
            if self.waypoint_reached():
                self.active_waypoint = None
                self.current_state = RobotState.IDLE
                
                self.get_logger().info('TRACKING -> IDLE: waypoint reached!')
                
                if len(self.waypoint_queue) > 0:
                    self.get_logger().info(f'Next waypoint queued: {len(self.waypoint_queue)} remaining')
    
    def waypoint_reached(self) -> bool:
        """Check if robot reached active waypoint (within 0.2m)"""
        if self.current_pose is None or self.active_waypoint is None:
            return False
            
        dx = self.current_pose[0] - self.active_waypoint[0]
        dy = self.current_pose[1] - self.active_waypoint[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        return distance < 0.2
    
    def plan_and_publish_path(self):
        """Generate and publish smooth path from current pose to active waypoint"""
        if self.current_pose is None or self.active_waypoint is None:
            return
            
        waypoints = [self.current_pose, self.active_waypoint]
        
        try:
            smooth_points = self.smooth_path(waypoints)
            path_msg = self.create_path_message(smooth_points)
            self.path_publisher.publish(path_msg)
            
            self.get_logger().info(
                f'Published path: {len(smooth_points)} points from '
                f'({self.current_pose[0]:.2f}, {self.current_pose[1]:.2f}) to '
                f'({self.active_waypoint[0]:.2f}, {self.active_waypoint[1]:.2f})')
            
        except Exception as e:
            self.get_logger().error(f'Path planning error: {str(e)}')
    
    def smooth_path(self, waypoints: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Apply cubic spline smoothing using SciPy"""
        if len(waypoints) < 2:
            return waypoints
        
        # Add intermediate point for 2-point paths to create smooth curve
        if len(waypoints) == 2:
            start, end = waypoints
            mid_x = (start[0] + end[0]) / 2
            mid_y = (start[1] + end[1]) / 2
            waypoints = [start, (mid_x, mid_y + 0.1), end]  # Small curve offset
        
        x_points = np.array([wp[0] for wp in waypoints])
        y_points = np.array([wp[1] for wp in waypoints])
        
        # Primary method: parametric spline interpolation
        try:
            tck, u = splprep([x_points, y_points], s=0, k=min(3, len(waypoints)-1))
            u_new = np.linspace(0, 1, self.num_points)
            smooth_points = splev(u_new, tck)
            smooth_x, smooth_y = smooth_points
            return list(zip(smooth_x, smooth_y))
            
        except Exception as e:
            self.get_logger().warn(f'Parametric spline failed: {e}, using fallback')
            
            # Fallback: linear interpolation for 2-point paths
            if len(waypoints) == 2:
                start, end = waypoints
                t = np.linspace(0, 1, self.num_points)
                smooth_x = start[0] + t * (end[0] - start[0])
                smooth_y = start[1] + t * (end[1] - start[1])
                return list(zip(smooth_x, smooth_y))
            
            # Fallback: distance-based cubic splines
            distances = [0.0]
            for i in range(1, len(waypoints)):
                dx = x_points[i] - x_points[i-1]
                dy = y_points[i] - y_points[i-1]
                distances.append(distances[-1] + math.sqrt(dx*dx + dy*dy))
            
            distances = np.array(distances)
            spline_x = CubicSpline(distances, x_points, bc_type='natural')
            spline_y = CubicSpline(distances, y_points, bc_type='natural')
            
            sample_distances = np.linspace(0, distances[-1], self.num_points)
            smooth_x = spline_x(sample_distances)
            smooth_y = spline_y(sample_distances)
            
            return list(zip(smooth_x, smooth_y))
    
    def create_path_message(self, path_points: List[Tuple[float, float]]) -> Path:
        """Create nav_msgs/Path message from smoothed path points"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for x, y in path_points:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            
            pose_stamped.pose.position.x = float(x)
            pose_stamped.pose.position.y = float(y)
            pose_stamped.pose.position.z = 0.0
            
            # Default orientation (identity quaternion)
            pose_stamped.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose_stamped)
        
        return path_msg


def main(args=None):
    """Main function to run the waypoint manager node"""
    rclpy.init(args=args)
    
    try:
        waypoint_manager = WaypointManager()
        rclpy.spin(waypoint_manager)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
