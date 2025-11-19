#!/usr/bin/env python3
"""
Trajectory Generator Node for ROS2
Subscribes to /smooth_path and generates time-parameterized trajectory with yaw
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
from typing import List, Tuple


class TrajectoryGenerator(Node):
    """
    ROS2 Node for generating time-parameterized trajectory from smooth path
    """
    
    def __init__(self):
        super().__init__('trajectory_generator')
        
        # Constant velocity for trajectory generation
        self.linear_velocity = 0.15  # m/s as specified
        
        # Subscriber for smooth path
        self.path_subscriber = self.create_subscription(
            Path,
            '/smooth_path',
            self.path_callback,
            10
        )
        
        # Publisher for trajectory
        self.trajectory_publisher = self.create_publisher(
            Path,
            '/trajectory',
            10
        )
        
        self.get_logger().info('Trajectory Generator Node initialized')
        self.get_logger().info(f'Using constant velocity: {self.linear_velocity} m/s')
    
    def calculate_yaw_from_points(self, x1: float, y1: float, x2: float, y2: float) -> float:
        """
        Calculate yaw angle from two successive points
        
        Args:
            x1, y1: First point coordinates
            x2, y2: Second point coordinates
            
        Returns:
            Yaw angle in radians
        """
        return math.atan2(y2 - y1, x2 - x1)
    
    def quaternion_from_yaw(self, yaw: float) -> Tuple[float, float, float, float]:
        """
        Convert yaw angle to quaternion
        
        Args:
            yaw: Yaw angle in radians
            
        Returns:
            Quaternion as (x, y, z, w) tuple
        """
        half_yaw = yaw * 0.5
        return (
            0.0,  # x
            0.0,  # y
            math.sin(half_yaw),  # z
            math.cos(half_yaw)   # w
        )
    
    def calculate_cumulative_distances(self, poses: List[PoseStamped]) -> List[float]:
        """
        Calculate cumulative distances along the path
        
        Args:
            poses: List of PoseStamped messages
            
        Returns:
            List of cumulative distances
        """
        distances = [0.0]
        
        for i in range(1, len(poses)):
            prev_pose = poses[i-1].pose.position
            curr_pose = poses[i].pose.position
            
            dx = curr_pose.x - prev_pose.x
            dy = curr_pose.y - prev_pose.y
            
            segment_distance = math.sqrt(dx*dx + dy*dy)
            distances.append(distances[-1] + segment_distance)
        
        return distances
    
    def generate_trajectory(self, path_msg: Path) -> Path:
        """
        Generate time-parameterized trajectory from smooth path
        
        Args:
            path_msg: Input smooth path
            
        Returns:
            Time-parameterized trajectory with yaw orientations
        """
        if len(path_msg.poses) < 2:
            self.get_logger().warn('Path has less than 2 points, cannot generate trajectory')
            return path_msg
        
        # Calculate cumulative distances
        distances = self.calculate_cumulative_distances(path_msg.poses)
        total_distance = distances[-1]
        
        if total_distance <= 0:
            self.get_logger().warn('Total path distance is zero')
            return path_msg
        
        # Calculate total travel time
        total_time = total_distance / self.linear_velocity
        
        # Create trajectory message
        trajectory_msg = Path()
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        trajectory_msg.header.frame_id = path_msg.header.frame_id
        
        # Generate time-parameterized trajectory points
        for i, pose in enumerate(path_msg.poses):
            # Create new pose with timestamp
            traj_pose = PoseStamped()
            traj_pose.header.frame_id = trajectory_msg.header.frame_id
            
            # Calculate timestamp for this point
            time_offset = distances[i] / self.linear_velocity
            timestamp_ns = trajectory_msg.header.stamp.sec * 1_000_000_000 + trajectory_msg.header.stamp.nanosec
            timestamp_ns += int(time_offset * 1_000_000_000)
            
            traj_pose.header.stamp.sec = int(timestamp_ns // 1_000_000_000)
            traj_pose.header.stamp.nanosec = int(timestamp_ns % 1_000_000_000)
            
            # Copy position
            traj_pose.pose.position.x = pose.pose.position.x
            traj_pose.pose.position.y = pose.pose.position.y
            traj_pose.pose.position.z = pose.pose.position.z
            
            # Calculate yaw from current and next point
            if i < len(path_msg.poses) - 1:
                # Use next point to calculate yaw
                curr_x = pose.pose.position.x
                curr_y = pose.pose.position.y
                next_x = path_msg.poses[i+1].pose.position.x
                next_y = path_msg.poses[i+1].pose.position.y
                
                yaw = self.calculate_yaw_from_points(curr_x, curr_y, next_x, next_y)
            else:
                # For last point, use previous yaw or calculate from previous point
                if i > 0:
                    prev_x = path_msg.poses[i-1].pose.position.x
                    prev_y = path_msg.poses[i-1].pose.position.y
                    curr_x = pose.pose.position.x
                    curr_y = pose.pose.position.y
                    
                    yaw = self.calculate_yaw_from_points(prev_x, prev_y, curr_x, curr_y)
                else:
                    yaw = 0.0  # Default yaw for single point
            
            # Convert yaw to quaternion
            qx, qy, qz, qw = self.quaternion_from_yaw(yaw)
            traj_pose.pose.orientation.x = qx
            traj_pose.pose.orientation.y = qy
            traj_pose.pose.orientation.z = qz
            traj_pose.pose.orientation.w = qw
            
            trajectory_msg.poses.append(traj_pose)
        
        return trajectory_msg
    
    def path_callback(self, msg: Path):
        """
        Callback function for smooth path subscription
        
        Args:
            msg: Received smooth path message
        """
        try:
            # Generate trajectory from smooth path
            trajectory = self.generate_trajectory(msg)
            
            # Publish trajectory
            self.trajectory_publisher.publish(trajectory)
            
            # Log success
            self.get_logger().info(
                f'Generated trajectory with {len(trajectory.poses)} points, '
                f'total time: {len(trajectory.poses) * self.linear_velocity:.2f}s',
                throttle_duration_sec=5.0
            )
            
        except Exception as e:
            self.get_logger().error(f'Error generating trajectory: {str(e)}')


def main(args=None):
    """Main function to run the trajectory generator node"""
    rclpy.init(args=args)
    
    try:
        trajectory_generator = TrajectoryGenerator()
        rclpy.spin(trajectory_generator)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()