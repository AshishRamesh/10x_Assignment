#!/usr/bin/env python3
"""
Path Smoother Node for ROS2
Implements cubic spline interpolation for path smoothing
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
from typing import List, Tuple


class CubicSpline:
    """
    Simple cubic spline implementation for path smoothing
    Uses natural cubic spline interpolation
    """
    
    def __init__(self, x: List[float], y: List[float]):
        self.x = np.array(x)
        self.y = np.array(y)
        self.n = len(x)
        
        # Calculate spline coefficients
        self.a, self.b, self.c, self.d = self._calculate_coefficients()
    
    def _calculate_coefficients(self):
        """Calculate cubic spline coefficients using natural spline conditions"""
        n = self.n - 1
        h = np.diff(self.x)
        
        # Build the tridiagonal system for second derivatives
        A = np.zeros((n + 1, n + 1))
        B = np.zeros(n + 1)
        
        # Natural spline boundary conditions (second derivative = 0 at endpoints)
        A[0, 0] = 1
        A[n, n] = 1
        
        # Interior points
        for i in range(1, n):
            A[i, i-1] = h[i-1]
            A[i, i] = 2 * (h[i-1] + h[i])
            A[i, i+1] = h[i]
            B[i] = 3 * ((self.y[i+1] - self.y[i]) / h[i] - (self.y[i] - self.y[i-1]) / h[i-1])
        
        # Solve for second derivatives
        c = np.linalg.solve(A, B)
        
        # Calculate other coefficients
        a = self.y[:-1].copy()
        b = np.zeros(n)
        d = np.zeros(n)
        
        for i in range(n):
            b[i] = (self.y[i+1] - self.y[i]) / h[i] - h[i] * (2 * c[i] + c[i+1]) / 3
            d[i] = (c[i+1] - c[i]) / (3 * h[i])
        
        return a, b, c[:-1], d
    
    def interpolate(self, x_new: float) -> float:
        """Interpolate a single point"""
        # Find the appropriate segment
        i = np.searchsorted(self.x[1:], x_new)
        i = min(i, len(self.a) - 1)
        
        # Calculate the interpolated value
        dx = x_new - self.x[i]
        return self.a[i] + self.b[i] * dx + self.c[i] * dx**2 + self.d[i] * dx**3
    
    def interpolate_path(self, x_new: np.ndarray) -> np.ndarray:
        """Interpolate multiple points"""
        return np.array([self.interpolate(x) for x in x_new])


class PathSmoother(Node):
    """
    ROS2 Node for path smoothing using cubic spline interpolation
    """
    
    def __init__(self):
        super().__init__('path_smoother')
        
        # Hardcoded waypoints
        self.waypoints = [
            (0.0, 0.0),
            (1.0, 0.5),
            (2.0, 1.0),
            (2.5, 0.5),
            (3.0, 0.0)
        ]
        
        # Number of interpolated points
        self.num_points = 150  # Dense sampling between 100-200 points
        
        # Publisher for smooth path
        self.path_publisher = self.create_publisher(
            Path,
            '/smooth_path',
            10
        )
        
        # Timer to publish path periodically
        self.timer = self.create_timer(1.0, self.publish_smooth_path)
        
        self.get_logger().info('Path Smoother Node initialized')
        self.get_logger().info(f'Waypoints: {self.waypoints}')
        self.get_logger().info(f'Number of interpolated points: {self.num_points}')
    
    def smooth_path(self, waypoints: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Apply cubic spline smoothing to the waypoints
        
        Args:
            waypoints: List of (x, y) waypoint tuples
            
        Returns:
            List of smoothed path points
        """
        if len(waypoints) < 2:
            return waypoints
        
        # Extract x and y coordinates
        x_points = [wp[0] for wp in waypoints]
        y_points = [wp[1] for wp in waypoints]
        
        # Calculate cumulative distance for parameterization
        distances = [0.0]
        for i in range(1, len(waypoints)):
            dx = x_points[i] - x_points[i-1]
            dy = y_points[i] - y_points[i-1]
            dist = math.sqrt(dx*dx + dy*dy)
            distances.append(distances[-1] + dist)
        
        # Create cubic splines for x and y as functions of distance
        spline_x = CubicSpline(distances, x_points)
        spline_y = CubicSpline(distances, y_points)
        
        # Generate dense sampling of the path
        total_distance = distances[-1]
        sample_distances = np.linspace(0, total_distance, self.num_points)
        
        # Interpolate smooth path
        smooth_x = spline_x.interpolate_path(sample_distances)
        smooth_y = spline_y.interpolate_path(sample_distances)
        
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
        """Timer callback to publish the smoothed path"""
        try:
            # Generate smooth path
            smooth_points = self.smooth_path(self.waypoints)
            
            # Create and publish path message
            path_msg = self.create_path_message(smooth_points)
            self.path_publisher.publish(path_msg)
            
            self.get_logger().info(
                f'Published smooth path with {len(smooth_points)} points',
                throttle_duration_sec=5.0  # Log every 5 seconds
            )
            
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
