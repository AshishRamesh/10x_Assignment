#!/usr/bin/env python3
"""
Obstacle-Aware Path Smoother Node for ROS2
Converts A* grid-based paths into smooth, continuous trajectories for robot control
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
from typing import List, Tuple, Optional
import numpy as np


class PathSmoother:
    """
    Path smoothing utility class using cubic spline interpolation and corner cutting
    """
    
    def __init__(self, num_output_points: int = 200):
        """
        Initialize path smoother
        
        Args:
            num_output_points: Target number of points in smoothed path
        """
        self.num_output_points = num_output_points
        self.min_input_points = 2  # Minimum points required for smoothing
    
    def extract_waypoints(self, path: Path) -> List[Tuple[float, float]]:
        """
        Extract (x, y) coordinates from nav_msgs/Path
        
        Args:
            path: Input nav_msgs/Path message
            
        Returns:
            List of (x, y) coordinate tuples
        """
        waypoints = []
        for pose_stamped in path.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            waypoints.append((x, y))
        
        return waypoints
    
    def calculate_path_length(self, waypoints: List[Tuple[float, float]]) -> float:
        """
        Calculate total path length
        
        Args:
            waypoints: List of (x, y) coordinates
            
        Returns:
            Total path length in meters
        """
        if len(waypoints) < 2:
            return 0.0
        
        total_length = 0.0
        for i in range(1, len(waypoints)):
            dx = waypoints[i][0] - waypoints[i-1][0]
            dy = waypoints[i][1] - waypoints[i-1][1]
            total_length += math.sqrt(dx*dx + dy*dy)
        
        return total_length
    
    def corner_cutting_smooth(self, waypoints: List[Tuple[float, float]], 
                             iterations: int = 3, alpha: float = 0.3) -> List[Tuple[float, float]]:
        """
        Apply corner cutting smoothing algorithm
        
        Args:
            waypoints: Input waypoints
            iterations: Number of smoothing iterations
            alpha: Smoothing factor (0-0.5, higher = more smoothing)
            
        Returns:
            Smoothed waypoints
        """
        if len(waypoints) < 3:
            return waypoints
        
        # Convert to numpy arrays for easier computation
        points = np.array(waypoints)
        
        # Apply corner cutting iterations
        for _ in range(iterations):
            new_points = np.copy(points)
            
            # Apply smoothing to interior points (keep start and end fixed)
            for i in range(1, len(points) - 1):
                # Corner cutting: move point towards average of neighbors
                neighbor_avg = (points[i-1] + points[i+1]) / 2.0
                new_points[i] = (1 - alpha) * points[i] + alpha * neighbor_avg
            
            points = new_points
        
        return [(float(p[0]), float(p[1])) for p in points]
    
    def interpolate_path(self, waypoints: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Interpolate path to achieve target number of points using linear interpolation
        
        Args:
            waypoints: Input waypoints
            
        Returns:
            Interpolated path with target number of points
        """
        if len(waypoints) < 2:
            return waypoints
        
        # Calculate cumulative distances
        distances = [0.0]
        for i in range(1, len(waypoints)):
            dx = waypoints[i][0] - waypoints[i-1][0]
            dy = waypoints[i][1] - waypoints[i-1][1]
            dist = math.sqrt(dx*dx + dy*dy)
            distances.append(distances[-1] + dist)
        
        total_distance = distances[-1]
        if total_distance == 0:
            return waypoints
        
        # Generate evenly spaced interpolation points
        interpolated_points = []
        target_distances = np.linspace(0, total_distance, self.num_output_points)
        
        for target_dist in target_distances:
            # Find segment containing target distance
            segment_idx = 0
            for i in range(len(distances) - 1):
                if distances[i] <= target_dist <= distances[i + 1]:
                    segment_idx = i
                    break
            
            # Linear interpolation within segment
            if segment_idx < len(waypoints) - 1:
                t = 0.0
                if distances[segment_idx + 1] != distances[segment_idx]:
                    t = (target_dist - distances[segment_idx]) / (distances[segment_idx + 1] - distances[segment_idx])
                
                x = waypoints[segment_idx][0] + t * (waypoints[segment_idx + 1][0] - waypoints[segment_idx][0])
                y = waypoints[segment_idx][1] + t * (waypoints[segment_idx + 1][1] - waypoints[segment_idx][1])
                
                interpolated_points.append((x, y))
            else:
                # Use last point if we're at the end
                interpolated_points.append(waypoints[-1])
        
        return interpolated_points
    
    def smooth_path(self, waypoints: List[Tuple[float, float]]) -> Optional[List[Tuple[float, float]]]:
        """
        Main smoothing function that combines corner cutting and interpolation
        
        Args:
            waypoints: Input waypoints from A* path
            
        Returns:
            Smoothed path points, None if smoothing fails
        """
        try:
            # Validate input
            if len(waypoints) < self.min_input_points:
                return None
            
            # Remove duplicate consecutive points
            cleaned_waypoints = []
            for i, point in enumerate(waypoints):
                if i == 0 or point != waypoints[i-1]:
                    cleaned_waypoints.append(point)
            
            if len(cleaned_waypoints) < self.min_input_points:
                return None
            
            # Apply corner cutting smoothing if we have enough points
            if len(cleaned_waypoints) >= 3:
                smoothed_waypoints = self.corner_cutting_smooth(cleaned_waypoints)
            else:
                smoothed_waypoints = cleaned_waypoints
            
            # Interpolate to target number of points
            final_waypoints = self.interpolate_path(smoothed_waypoints)
            
            # Validate output
            if len(final_waypoints) < 2:
                return None
            
            return final_waypoints
            
        except Exception:
            return None  # Return None on any smoothing failure


class SmootherObstacleNode(Node):
    """
    ROS2 Node for smoothing obstacle-aware A* paths into continuous trajectories
    """
    
    def __init__(self):
        super().__init__('smoother_obstacle')
        
        # Path smoother instance
        self.path_smoother = PathSmoother(num_output_points=200)
        
        # Publishers
        self.smooth_path_publisher = self.create_publisher(
            Path,
            '/smooth_path',
            10
        )
        
        # Subscribers
        self.astar_path_sub = self.create_subscription(
            Path,
            '/a_star_path',
            self.astar_path_callback,
            10
        )
        
        self.get_logger().info('Obstacle-Aware Path Smoother Node initialized')
        self.get_logger().info('Subscribing to /a_star_path and publishing to /smooth_path')
        self.get_logger().info('Ready to smooth A* paths into continuous trajectories')
    
    def astar_path_callback(self, msg: Path):
        """
        Callback for A* path messages
        
        Args:
            msg: nav_msgs/Path message from A* planner
        """
        self.get_logger().info(f'Received A* path with {len(msg.poses)} waypoints')
        
        # Validate input path
        if len(msg.poses) == 0:
            self.get_logger().warn('Received empty A* path! Publishing empty smooth path.')
            self.publish_empty_path(msg.header.frame_id)
            return
        
        if len(msg.poses) < 2:
            self.get_logger().warn('A* path too short for smoothing! Publishing empty smooth path.')
            self.publish_empty_path(msg.header.frame_id)
            return
        
        # Extract waypoints from path message
        try:
            waypoints = self.path_smoother.extract_waypoints(msg)
            path_length = self.path_smoother.calculate_path_length(waypoints)
            
            self.get_logger().info(
                f'Extracted {len(waypoints)} waypoints, total path length: {path_length:.2f}m'
            )
            
            # Apply path smoothing
            smoothed_waypoints = self.path_smoother.smooth_path(waypoints)
            
            if smoothed_waypoints is None:
                self.get_logger().warn('Path smoothing failed! Publishing empty smooth path.')
                self.publish_empty_path(msg.header.frame_id)
                return
            
            # Create and publish smooth path
            self.publish_smooth_path(smoothed_waypoints, msg.header.frame_id)
            
        except Exception as e:
            self.get_logger().error(f'Error processing A* path: {str(e)}')
            self.publish_empty_path(msg.header.frame_id)
    
    def publish_smooth_path(self, smoothed_waypoints: List[Tuple[float, float]], frame_id: str):
        """
        Create and publish smoothed path as nav_msgs/Path
        
        Args:
            smoothed_waypoints: List of smoothed (x, y) coordinates
            frame_id: Frame ID for the path (typically 'map')
        """
        try:
            # Create path message
            smooth_path = Path()
            smooth_path.header.stamp = self.get_clock().now().to_msg()
            smooth_path.header.frame_id = frame_id
            
            # Convert waypoints to PoseStamped messages
            for i, (x, y) in enumerate(smoothed_waypoints):
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = smooth_path.header.stamp
                pose_stamped.header.frame_id = frame_id
                
                # Set position
                pose_stamped.pose.position.x = float(x)
                pose_stamped.pose.position.y = float(y)
                pose_stamped.pose.position.z = 0.0
                
                # Calculate orientation based on path direction
                if i < len(smoothed_waypoints) - 1:
                    # Calculate yaw angle to next waypoint
                    dx = smoothed_waypoints[i + 1][0] - x
                    dy = smoothed_waypoints[i + 1][1] - y
                    yaw = math.atan2(dy, dx)
                    
                    # Convert yaw to quaternion
                    pose_stamped.pose.orientation.x = 0.0
                    pose_stamped.pose.orientation.y = 0.0
                    pose_stamped.pose.orientation.z = math.sin(yaw / 2.0)
                    pose_stamped.pose.orientation.w = math.cos(yaw / 2.0)
                else:
                    # Last point: use orientation from previous point
                    if len(smooth_path.poses) > 0:
                        prev_orientation = smooth_path.poses[-1].pose.orientation
                        pose_stamped.pose.orientation = prev_orientation
                    else:
                        # Fallback to neutral orientation
                        pose_stamped.pose.orientation.x = 0.0
                        pose_stamped.pose.orientation.y = 0.0
                        pose_stamped.pose.orientation.z = 0.0
                        pose_stamped.pose.orientation.w = 1.0
                
                smooth_path.poses.append(pose_stamped)
            
            # Publish the smoothed path
            self.smooth_path_publisher.publish(smooth_path)
            
            # Calculate smoothed path length for logging
            smooth_length = self.path_smoother.calculate_path_length(smoothed_waypoints)
            
            self.get_logger().info(
                f'Published smooth path with {len(smoothed_waypoints)} points, '
                f'length: {smooth_length:.2f}m'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error publishing smooth path: {str(e)}')
            self.publish_empty_path(frame_id)
    
    def publish_empty_path(self, frame_id: str = 'map'):
        """
        Publish an empty path message
        
        Args:
            frame_id: Frame ID for the empty path
        """
        empty_path = Path()
        empty_path.header.stamp = self.get_clock().now().to_msg()
        empty_path.header.frame_id = frame_id
        
        self.smooth_path_publisher.publish(empty_path)
        self.get_logger().info('Published empty smooth path')


def main(args=None):
    """Main function to run the smoother obstacle node"""
    rclpy.init(args=args)
    
    try:
        smoother_node = SmootherObstacleNode()
        rclpy.spin(smoother_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()