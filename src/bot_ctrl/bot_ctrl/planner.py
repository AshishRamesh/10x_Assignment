#!/usr/bin/env python3
"""
Trajectory Generator Node for ROS2
Subscribes to /smooth_path and generates time-parameterized trajectory with yaw
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import math
from typing import List, Tuple, Optional


class TrajectoryGenerator(Node):
    """ROS2 Node for generating time-parameterized trajectory from smooth path"""
    
    def __init__(self):
        super().__init__('trajectory_generator')
        
        self.current_pose: Optional[Tuple[float, float, float]] = None
        self.linear_velocity = 0.5  # m/s
        
        self.path_subscriber = self.create_subscription(
            Path, '/smooth_path', self.path_callback, 10)
        
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        self.trajectory_publisher = self.create_publisher(
            Path, '/trajectory', 10)
        
        self.get_logger().info('Trajectory Generator Node initialized')
        self.get_logger().info(f'Velocity: {self.linear_velocity} m/s')
    
    def odom_callback(self, msg: Odometry):
        """Update current robot pose from odometry"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(quat.x, quat.y, quat.z, quat.w)
        self.current_pose = (x, y, yaw)
    
    def quaternion_to_yaw(self, qx: float, qy: float, qz: float, qw: float) -> float:
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def calculate_yaw_from_points(self, x1: float, y1: float, x2: float, y2: float) -> float:
        """Calculate yaw angle between two points"""
        return math.atan2(y2 - y1, x2 - x1)
    
    def quaternion_from_yaw(self, yaw: float) -> Tuple[float, float, float, float]:
        """Convert yaw angle to quaternion (x, y, z, w)"""
        half_yaw = yaw * 0.5
        return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))
    
    def calculate_cumulative_distances(self, poses: List[PoseStamped]) -> List[float]:
        """Calculate cumulative distances along path"""
        distances = [0.0]
        
        for i in range(1, len(poses)):
            prev = poses[i-1].pose.position
            curr = poses[i].pose.position
            
            dx = curr.x - prev.x
            dy = curr.y - prev.y
            
            distances.append(distances[-1] + math.sqrt(dx*dx + dy*dy))
        
        return distances
    
    def generate_trajectory(self, path_msg: Path) -> Path:
        """Generate time-parameterized trajectory from current pose"""
        if len(path_msg.poses) < 1:
            self.get_logger().warn('Path has no points')
            return path_msg
        
        if self.current_pose is None:
            self.get_logger().warn('No current robot pose available')
            return path_msg
        
        # Create path starting from current robot position
        modified_poses = []
        
        # Add current pose as starting point
        current_pose_stamped = PoseStamped()
        current_pose_stamped.header.frame_id = path_msg.header.frame_id
        current_pose_stamped.pose.position.x = self.current_pose[0]
        current_pose_stamped.pose.position.y = self.current_pose[1]
        current_pose_stamped.pose.position.z = 0.0
        
        qx, qy, qz, qw = self.quaternion_from_yaw(self.current_pose[2])
        current_pose_stamped.pose.orientation.x = qx
        current_pose_stamped.pose.orientation.y = qy
        current_pose_stamped.pose.orientation.z = qz
        current_pose_stamped.pose.orientation.w = qw
        
        modified_poses.append(current_pose_stamped)
        modified_poses.extend(path_msg.poses)
        
        distances = self.calculate_cumulative_distances(modified_poses)
        total_distance = distances[-1]
        
        if total_distance <= 0:
            self.get_logger().warn('Zero trajectory distance')
            return path_msg
        
        self.get_logger().info(
            f'Generating trajectory: {total_distance:.2f}m from ({self.current_pose[0]:.2f}, {self.current_pose[1]:.2f})')
        
        # Create trajectory message
        trajectory_msg = Path()
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        trajectory_msg.header.frame_id = path_msg.header.frame_id
        
        # Generate time-parameterized trajectory points
        for i, pose in enumerate(modified_poses):
            traj_pose = PoseStamped()
            traj_pose.header.frame_id = trajectory_msg.header.frame_id
            
            # Calculate timestamp
            time_offset = distances[i] / self.linear_velocity
            timestamp_ns = trajectory_msg.header.stamp.sec * 1_000_000_000 + trajectory_msg.header.stamp.nanosec
            timestamp_ns += int(time_offset * 1_000_000_000)
            
            traj_pose.header.stamp.sec = int(timestamp_ns // 1_000_000_000)
            traj_pose.header.stamp.nanosec = int(timestamp_ns % 1_000_000_000)
            
            # Copy position
            traj_pose.pose.position = pose.pose.position
            
            # Calculate yaw angle for orientation
            if i < len(modified_poses) - 1:
                curr_pos = pose.pose.position
                next_pos = modified_poses[i+1].pose.position
                yaw = self.calculate_yaw_from_points(curr_pos.x, curr_pos.y, next_pos.x, next_pos.y)
            elif i > 0:
                prev_pos = modified_poses[i-1].pose.position
                curr_pos = pose.pose.position
                yaw = self.calculate_yaw_from_points(prev_pos.x, prev_pos.y, curr_pos.x, curr_pos.y)
            else:
                yaw = self.current_pose[2] if self.current_pose else 0.0
            
            # Set orientation
            qx, qy, qz, qw = self.quaternion_from_yaw(yaw)
            traj_pose.pose.orientation.x = qx
            traj_pose.pose.orientation.y = qy
            traj_pose.pose.orientation.z = qz
            traj_pose.pose.orientation.w = qw
            
            trajectory_msg.poses.append(traj_pose)
        
        return trajectory_msg
    
    def path_callback(self, msg: Path):
        """Process received smooth path and generate trajectory"""
        try:
            if self.current_pose is None:
                self.get_logger().warn('Waiting for odometry data...')
                return
            
            trajectory = self.generate_trajectory(msg)
            self.trajectory_publisher.publish(trajectory)
            
            if len(trajectory.poses) > 0:
                # Calculate total distance for logging
                total_distance = 0.0
                for i in range(1, len(trajectory.poses)):
                    prev = trajectory.poses[i-1].pose.position
                    curr = trajectory.poses[i].pose.position
                    dx = curr.x - prev.x
                    dy = curr.y - prev.y
                    total_distance += math.sqrt(dx*dx + dy*dy)
                
                self.get_logger().info(
                    f'Trajectory: {len(trajectory.poses)} points, {total_distance:.2f}m, '
                    f'{total_distance/self.linear_velocity:.1f}s',
                    throttle_duration_sec=3.0)
            
        except Exception as e:
            self.get_logger().error(f'Trajectory generation error: {str(e)}')


def main(args=None):
    """Main function to run the trajectory generator node"""
    rclpy.init(args=args)
    
    try:
        trajectory_generator = TrajectoryGenerator()
        rclpy.spin(trajectory_generator)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()