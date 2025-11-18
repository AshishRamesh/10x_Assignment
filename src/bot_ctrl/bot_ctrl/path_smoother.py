#!/usr/bin/env python3
"""
Path Smoother Node for ROS2
Implements cubic spline interpolation for path smoothing using SciPy
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
from scipy.interpolate import CubicSpline, splprep, splev
from typing import List, Tuple





class PathSmoother(Node):
    """
    ROS2 Node for path smoothing using cubic spline interpolation
    """
    
    def __init__(self):
        super().__init__('path_smoother')
        
        # Hardcoded waypoints
        self.waypoints = [
            (-2.75, -0.82),
            (-3.75, -0.1),
            (-3.80, 0.88),
            (-1.81, 0.88),
            (-1.71, -0.04),
            (-2.75, -0.82)
        ]
        
        # Number of interpolated points
        self.num_points = 200  # Dense sampling between 100-200 points
        
        # Publisher for smooth path
        self.path_publisher = self.create_publisher(
            Path,
            '/smooth_path',
            10
        )
        
        # Timer to publish path once at startup and then stop
        self.published = False
        self.timer = self.create_timer(2.0, self.publish_smooth_path)  # 2 second delay for startup
        
        self.get_logger().info('Path Smoother Node initialized with SciPy')
        self.get_logger().info(f'Waypoints: {self.waypoints}')
        self.get_logger().info(f'Number of interpolated points: {self.num_points}')
        self.get_logger().info('Using SciPy parametric splines (splprep/splev) with fallback to CubicSpline')
    
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
        
        # Extract x and y coordinates
        x_points = np.array([wp[0] for wp in waypoints])
        y_points = np.array([wp[1] for wp in waypoints])
        
        # Method 1: Use parametric spline interpolation with splprep/splev
        # This method is better for 2D paths as it treats the path as a parametric curve
        try:
            # Create parametric spline representation
            # s=0 forces the spline to pass through all points exactly
            # k=3 for cubic splines (default)
            tck, u = splprep([x_points, y_points], s=0, k=min(3, len(waypoints)-1))
            
            # Generate new parameter values for dense sampling
            u_new = np.linspace(0, 1, self.num_points)
            
            # Evaluate spline at new parameter values
            smooth_points = splev(u_new, tck)
            smooth_x, smooth_y = smooth_points
            
            # Return as list of tuples
            return list(zip(smooth_x, smooth_y))
            
        except Exception as e:
            self.get_logger().warn(f'Parametric spline failed: {e}, falling back to distance-based method')
            
            # Method 2: Fallback to distance-based cubic spline interpolation
            # Calculate cumulative distance for parameterization
            distances = [0.0]
            for i in range(1, len(waypoints)):
                dx = x_points[i] - x_points[i-1]
                dy = y_points[i] - y_points[i-1]
                dist = math.sqrt(dx*dx + dy*dy)
                distances.append(distances[-1] + dist)
            
            distances = np.array(distances)
            
            # Create cubic splines for x and y as functions of distance using SciPy
            spline_x = CubicSpline(distances, x_points, bc_type='natural')
            spline_y = CubicSpline(distances, y_points, bc_type='natural')
            
            # Generate dense sampling of the path
            total_distance = distances[-1]
            sample_distances = np.linspace(0, total_distance, self.num_points)
            
            # Interpolate smooth path
            smooth_x = spline_x(sample_distances)
            smooth_y = spline_y(sample_distances)
            
            # Return as list of tuples
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
    
    def publish_smooth_path(self):
        """Timer callback to publish the smoothed path (only once)"""
        if self.published:
            return  # Already published, don't publish again
            
        try:
            # Generate smooth path
            smooth_points = self.smooth_path(self.waypoints)
            
            # Create and publish path message
            path_msg = self.create_path_message(smooth_points)
            self.path_publisher.publish(path_msg)
            
            self.get_logger().info(f'Published smooth path with {len(smooth_points)} points (one-time)')
            
            # Mark as published and destroy timer
            self.published = True
            self.timer.cancel()
            
        except Exception as e:
            self.get_logger().error(f'Error in path smoothing: {str(e)}')


def main(args=None):
    """Main function to run the path smoother node"""
    rclpy.init(args=args)
    
    try:
        path_smoother = PathSmoother()
        rclpy.spin(path_smoother)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
