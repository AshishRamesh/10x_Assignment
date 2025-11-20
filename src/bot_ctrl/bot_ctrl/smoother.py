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
    """ROS2 Node for path smoothing using cubic spline interpolation"""
    
    def __init__(self):
        super().__init__('path_smoother')
        
        self.waypoints = [
            (-2.75, -0.82),
            (-3.75, -0.1),
            (-3.80, 0.88),
            (-1.81, 0.88),
            (-1.71, -0.04)
        ]
        
        self.num_points = 200
        
        self.path_publisher = self.create_publisher(Path, '/smooth_path', 10)
        
        self.published = False
        self.timer = self.create_timer(2.0, self.publish_smooth_path)
        
        self.get_logger().info('Path Smoother Node initialized')
        self.get_logger().info(f'Waypoints: {self.waypoints}')
        self.get_logger().info(f'Interpolated points: {self.num_points}')
    
    def smooth_path(self, waypoints: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Apply cubic spline smoothing using SciPy parametric splines"""
        if len(waypoints) < 2:
            return waypoints
        
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
            self.get_logger().warn(f'Parametric spline failed: {e}, using fallback method')
            
            # Fallback: distance-based cubic spline interpolation
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
    
    def publish_smooth_path(self):
        """One-time path publication callback"""
        if self.published:
            return
            
        try:
            smooth_points = self.smooth_path(self.waypoints)
            path_msg = self.create_path_message(smooth_points)
            self.path_publisher.publish(path_msg)
            
            self.get_logger().info(f'Published smooth path with {len(smooth_points)} points')
            
            self.published = True
            self.timer.cancel()
            
        except Exception as e:
            self.get_logger().error(f'Path smoothing error: {str(e)}')


def main(args=None):
    """Main function to run the path smoother node"""
    rclpy.init(args=args)
    
    try:
        path_smoother = PathSmoother()
        rclpy.spin(path_smoother)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
