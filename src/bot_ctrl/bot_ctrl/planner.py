#!/usr/bin/env python3
"""
Trajectory Generator Node for ROS2
Subscribes to /smooth_path and generates time-parameterized trajectory with yaw
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
from typing import List, Tuple, Optional


class TrajectoryGenerator(Node):
    """
    ROS2 Node for generating time-parameterized trajectory from smooth path
    """
    
    def __init__(self):
        super().__init__('trajectory_generator')
        
        # Robot state
        self.current_pose: Optional[Tuple[float, float, float]] = None  # (x, y, yaw) from odometry
        
        # Constant velocity for trajectory generation
        self.linear_velocity = 0.5  # m/s - increased speed
        
        # Subscriber for smooth path
        self.path_subscriber = self.create_subscription(
            Path,
            '/smooth_path',
            self.path_callback,
            10
        )
        
        # Subscriber for robot odometry
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
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
        self.get_logger().info('Trajectories will start from robot current pose (/odom)')
    
    def odom_callback(self, msg: Odometry):
        """
        Callback for odometry data to track current robot pose
        
        Args:
            msg: Odometry message from /odom topic
        """
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Convert quaternion to yaw
        quat = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(quat.x, quat.y, quat.z, quat.w)
        
        # Store current pose
        self.current_pose = (x, y, yaw)
    
    def quaternion_to_yaw(self, qx: float, qy: float, qz: float, qw: float) -> float:
        """
        Convert quaternion to yaw angle
        
        Args:
            qx, qy, qz, qw: Quaternion components
            
        Returns:
            Yaw angle in radians
        """
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)
    
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
        Generate time-parameterized trajectory from robot's current pose to path target
        
        Args:
            path_msg: Input smooth path (target trajectory)
            
        Returns:
            Time-parameterized trajectory starting from current robot pose
        """
        if len(path_msg.poses) < 1:
            self.get_logger().warn('Path has no points, cannot generate trajectory')
            return path_msg
        
        if self.current_pose is None:
            self.get_logger().warn('No current robot pose available, using path as-is')
            return path_msg
        
        # Create modified path starting from current robot position
        modified_poses = []
        
        # Add current robot pose as starting point
        current_pose_stamped = PoseStamped()
        current_pose_stamped.header.frame_id = path_msg.header.frame_id
        current_pose_stamped.pose.position.x = self.current_pose[0]
        current_pose_stamped.pose.position.y = self.current_pose[1]
        current_pose_stamped.pose.position.z = 0.0
        
        # Set current orientation
        qx, qy, qz, qw = self.quaternion_from_yaw(self.current_pose[2])
        current_pose_stamped.pose.orientation.x = qx
        current_pose_stamped.pose.orientation.y = qy
        current_pose_stamped.pose.orientation.z = qz
        current_pose_stamped.pose.orientation.w = qw
        
        modified_poses.append(current_pose_stamped)
        
        # Add all points from the original path
        modified_poses.extend(path_msg.poses)
        
        # Calculate cumulative distances for the modified path
        distances = self.calculate_cumulative_distances(modified_poses)
        total_distance = distances[-1]
        
        if total_distance <= 0:
            self.get_logger().warn('Total trajectory distance is zero')
            return path_msg
        
        self.get_logger().info(
            f'Generating trajectory from current pose ({self.current_pose[0]:.2f}, {self.current_pose[1]:.2f}) '
            f'with total distance: {total_distance:.2f}m'
        )
        
        # Calculate total travel time
        total_time = total_distance / self.linear_velocity
        
        # Create trajectory message
        trajectory_msg = Path()
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        trajectory_msg.header.frame_id = path_msg.header.frame_id
        
        # Generate time-parameterized trajectory points from modified path
        for i, pose in enumerate(modified_poses):
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
            if i < len(modified_poses) - 1:
                # Use next point to calculate yaw
                curr_x = pose.pose.position.x
                curr_y = pose.pose.position.y
                next_x = modified_poses[i+1].pose.position.x
                next_y = modified_poses[i+1].pose.position.y
                
                yaw = self.calculate_yaw_from_points(curr_x, curr_y, next_x, next_y)
            else:
                # For last point, use previous yaw or calculate from previous point
                if i > 0:
                    prev_x = modified_poses[i-1].pose.position.x
                    prev_y = modified_poses[i-1].pose.position.y
                    curr_x = pose.pose.position.x
                    curr_y = pose.pose.position.y
                    
                    yaw = self.calculate_yaw_from_points(prev_x, prev_y, curr_x, curr_y)
                else:
                    # Use current robot yaw for first point
                    yaw = self.current_pose[2] if self.current_pose else 0.0
            
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
            if self.current_pose is None:
                self.get_logger().warn('No robot odometry available yet. Waiting for /odom data...')
                return
            
            # Generate trajectory from robot's current pose to path target
            trajectory = self.generate_trajectory(msg)
            
            # Publish trajectory
            self.trajectory_publisher.publish(trajectory)
            
            # Calculate trajectory info for logging
            if len(trajectory.poses) > 0:
                total_distance = 0.0
                for i in range(1, len(trajectory.poses)):
                    prev = trajectory.poses[i-1].pose.position
                    curr = trajectory.poses[i].pose.position
                    dx = curr.x - prev.x
                    dy = curr.y - prev.y
                    total_distance += math.sqrt(dx*dx + dy*dy)
                
                total_time = total_distance / self.linear_velocity
                
                self.get_logger().info(
                    f'Generated trajectory: {len(trajectory.poses)} points, '
                    f'distance: {total_distance:.2f}m, time: {total_time:.1f}s',
                    throttle_duration_sec=3.0
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