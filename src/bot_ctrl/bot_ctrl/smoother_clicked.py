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
from typing import List, Tuple, Optional
from collections import deque
from enum import Enum


class RobotState(Enum):
    """Robot navigation states"""
    IDLE = "idle"
    PLANNING = "planning"
    TRACKING = "tracking"


class WaypointManager(Node):
    """
    ROS2 Node for click-to-navigate waypoint management with path smoothing
    """
    
    def __init__(self):
        super().__init__('waypoint_manager')
        
        # FIFO waypoint queue
        self.waypoint_queue = deque()  # Queue of (x, y) waypoints from clicks
        
        # Robot state management
        self.current_state = RobotState.IDLE
        self.current_pose = None  # Current robot pose from odometry
        self.active_waypoint = None  # Currently active waypoint being navigated to
        
        # Path smoothing parameters
        self.num_points = 150  # Dense sampling for smooth paths
        
        # Publishers
        self.path_publisher = self.create_publisher(
            Path,
            '/smooth_path',
            10
        )
        
        # Subscribers
        self.clicked_point_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Timer for state machine updates
        self.state_timer = self.create_timer(0.1, self.state_machine_update)  # 10 Hz update
        
        self.get_logger().info('Waypoint Manager Node initialized')
        self.get_logger().info('Ready to receive clicked waypoints from RViz')
        self.get_logger().info(f'Path smoothing with {self.num_points} interpolated points')
        self.get_logger().info('State: IDLE - Click points in RViz to start navigation')
    
    def clicked_point_callback(self, msg: PointStamped):
        """
        Callback for clicked points from RViz
        
        Args:
            msg: PointStamped message from /clicked_point topic
        """
        # Extract waypoint coordinates
        waypoint = (msg.point.x, msg.point.y)
        
        # Add to FIFO queue
        self.waypoint_queue.append(waypoint)
        
        self.get_logger().info(
            f'Added waypoint ({waypoint[0]:.2f}, {waypoint[1]:.2f}) to queue. '
            f'Queue size: {len(self.waypoint_queue)}'
        )
        
    def odom_callback(self, msg: Odometry):
        """
        Callback for odometry data to track current robot pose
        
        Args:
            msg: Odometry message from /odom topic
        """
        # Store current pose
        self.current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        
    def state_machine_update(self):
        """
        Main state machine update loop
        """
        if self.current_state == RobotState.IDLE:
            # Check if we have waypoints to process and current pose is available
            if len(self.waypoint_queue) > 0 and self.current_pose is not None:
                # Take first waypoint from queue
                self.active_waypoint = self.waypoint_queue.popleft()
                self.current_state = RobotState.PLANNING
                
                self.get_logger().info(
                    f'State: IDLE -> PLANNING. Processing waypoint '
                    f'({self.active_waypoint[0]:.2f}, {self.active_waypoint[1]:.2f})'
                )
                
        elif self.current_state == RobotState.PLANNING:
            if self.current_pose is not None and self.active_waypoint is not None:
                # Generate and publish smooth path from current pose to waypoint
                self.plan_and_publish_path()
                self.current_state = RobotState.TRACKING
                
                self.get_logger().info(
                    f'State: PLANNING -> TRACKING. Path published to '
                    f'({self.active_waypoint[0]:.2f}, {self.active_waypoint[1]:.2f})'
                )
                
        elif self.current_state == RobotState.TRACKING:
            # Check if robot reached the waypoint (simplified - in practice you'd 
            # subscribe to trajectory completion or check distance)
            if self.waypoint_reached():
                self.active_waypoint = None
                self.current_state = RobotState.IDLE
                
                self.get_logger().info('State: TRACKING -> IDLE. Waypoint reached!')
                
                # Check if more waypoints are queued
                if len(self.waypoint_queue) > 0:
                    self.get_logger().info(f'Processing next waypoint. Queue size: {len(self.waypoint_queue)}')
    
    def waypoint_reached(self) -> bool:
        """
        Check if the robot has reached the active waypoint
        
        Returns:
            True if waypoint is reached, False otherwise
        """
        if self.current_pose is None or self.active_waypoint is None:
            return False
            
        # Calculate distance to waypoint
        dx = self.current_pose[0] - self.active_waypoint[0]
        dy = self.current_pose[1] - self.active_waypoint[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Consider waypoint reached if within 0.2 meters
        return distance < 0.2
    
    def plan_and_publish_path(self):
        """
        Generate and publish a smooth path from current pose to active waypoint
        """
        if self.current_pose is None or self.active_waypoint is None:
            return
            
        # Create waypoints list: current pose -> target waypoint
        waypoints = [self.current_pose, self.active_waypoint]
        
        try:
            # Generate smooth path
            smooth_points = self.smooth_path(waypoints)
            
            # Create and publish path message
            path_msg = self.create_path_message(smooth_points)
            self.path_publisher.publish(path_msg)
            
            self.get_logger().info(
                f'Published smooth path with {len(smooth_points)} points from '
                f'({self.current_pose[0]:.2f}, {self.current_pose[1]:.2f}) to '
                f'({self.active_waypoint[0]:.2f}, {self.active_waypoint[1]:.2f})'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error in path planning: {str(e)}')
    
    def smooth_path(self, waypoints: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Apply cubic spline smoothing to the waypoints using SciPy
        
        Args:
            waypoints: List of (x, y) waypoint tuples
            
        Returns:
            List of smoothed path points
        """
        if len(waypoints) < 2:
            return waypoints
        
        # For 2-point paths (current pose -> target), add intermediate points for better smoothing
        if len(waypoints) == 2:
            start, end = waypoints
            # Add a slight curve by inserting intermediate point
            mid_x = (start[0] + end[0]) / 2
            mid_y = (start[1] + end[1]) / 2
            # Slightly offset the midpoint for natural curvature
            offset = 0.1  # Small offset for smooth curve
            waypoints = [start, (mid_x, mid_y + offset), end]
        
        # Extract x and y coordinates
        x_points = np.array([wp[0] for wp in waypoints])
        y_points = np.array([wp[1] for wp in waypoints])
        
        # Method 1: Use parametric spline interpolation with splprep/splev
        try:
            # Create parametric spline representation
            tck, u = splprep([x_points, y_points], s=0, k=min(3, len(waypoints)-1))
            
            # Generate new parameter values for dense sampling
            u_new = np.linspace(0, 1, self.num_points)
            
            # Evaluate spline at new parameter values
            smooth_points = splev(u_new, tck)
            smooth_x, smooth_y = smooth_points
            
            # Return as list of tuples
            return list(zip(smooth_x, smooth_y))
            
        except Exception as e:
            self.get_logger().warn(f'Parametric spline failed: {e}, using linear interpolation')
            
            # Fallback: Simple linear interpolation for 2-point paths
            if len(waypoints) == 2:
                start, end = waypoints
                t = np.linspace(0, 1, self.num_points)
                smooth_x = start[0] + t * (end[0] - start[0])
                smooth_y = start[1] + t * (end[1] - start[1])
                return list(zip(smooth_x, smooth_y))
            
            # For multiple points, use distance-based interpolation
            distances = [0.0]
            for i in range(1, len(waypoints)):
                dx = x_points[i] - x_points[i-1]
                dy = y_points[i] - y_points[i-1]
                dist = math.sqrt(dx*dx + dy*dy)
                distances.append(distances[-1] + dist)
            
            distances = np.array(distances)
            
            # Create cubic splines using SciPy
            spline_x = CubicSpline(distances, x_points, bc_type='natural')
            spline_y = CubicSpline(distances, y_points, bc_type='natural')
            
            # Generate dense sampling
            total_distance = distances[-1]
            sample_distances = np.linspace(0, total_distance, self.num_points)
            
            smooth_x = spline_x(sample_distances)
            smooth_y = spline_y(sample_distances)
            
            return list(zip(smooth_x, smooth_y))
    
    def create_path_message(self, path_points: List[Tuple[float, float]]) -> Path:
        """
        Create a nav_msgs/Path message from path points
        
        Args:
            path_points: List of (x, y) coordinate tuples
            
        Returns:
            nav_msgs/Path message
        """
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'  # Assuming map frame
        
        for x, y in path_points:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = path_msg.header.stamp
            pose_stamped.header.frame_id = path_msg.header.frame_id
            
            pose_stamped.pose.position.x = float(x)
            pose_stamped.pose.position.y = float(y)
            pose_stamped.pose.position.z = 0.0
            
            # Set orientation (assuming robot moves forward along path)
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = 0.0
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
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
