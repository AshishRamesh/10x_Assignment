#!/usr/bin/env python3
"""
Trajectory Tracker Node for ROS2
Implements Pure Pursuit controller for trajectory following
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math
from typing import Optional, Tuple


class TrajectoryTracker(Node):
    """
    ROS2 Node implementing Pure Pursuit controller for trajectory tracking
    """
    
    def __init__(self):
        super().__init__('trajectory_tracker')
        
        # Pure Pursuit controller parameters
        self.lookahead_distance = 0.5  # meters as specified
        self.linear_velocity = 0.15    # m/s as specified
        
        # Current state
        self.current_pose = None
        self.current_trajectory = None
        self.target_point_index = 0
        self.last_trajectory_hash = None  # To track if trajectory changed
        
        # Subscribers
        self.trajectory_subscriber = self.create_subscription(
            Path,
            '/trajectory',
            self.trajectory_callback,
            10
        )
        
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publisher for control commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        self.get_logger().info('Trajectory Tracker Node initialized')
        self.get_logger().info(f'Lookahead distance: {self.lookahead_distance} m')
        self.get_logger().info(f'Linear velocity: {self.linear_velocity} m/s')
    
    def quaternion_to_yaw(self, qx: float, qy: float, qz: float, qw: float) -> float:
        """
        Convert quaternion to yaw angle
        
        Args:
            qx, qy, qz, qw: Quaternion components
            
        Returns:
            Yaw angle in radians
        """
        # Convert quaternion to yaw angle
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def distance_between_points(self, x1: float, y1: float, x2: float, y2: float) -> float:
        """
        Calculate Euclidean distance between two points
        
        Args:
            x1, y1: First point coordinates
            x2, y2: Second point coordinates
            
        Returns:
            Distance between points
        """
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def find_lookahead_point(self) -> Optional[Tuple[float, float]]:
        """
        Find the lookahead point on the trajectory using Pure Pursuit algorithm
        
        Returns:
            Tuple of (x, y) coordinates of lookahead point, or None if not found
        """
        if not self.current_trajectory or not self.current_pose:
            return None
        
        if not self.current_trajectory.poses:
            return None
        
        current_x = self.current_pose[0]
        current_y = self.current_pose[1]
        
        # Start searching from current target point index
        for i in range(self.target_point_index, len(self.current_trajectory.poses)):
            pose = self.current_trajectory.poses[i]
            point_x = pose.pose.position.x
            point_y = pose.pose.position.y
            
            distance = self.distance_between_points(current_x, current_y, point_x, point_y)
            
            # If this point is at or beyond lookahead distance, use it
            if distance >= self.lookahead_distance:
                self.target_point_index = i
                return (point_x, point_y)
            
            # If this is close to lookahead distance, interpolate
            if i > 0 and distance < self.lookahead_distance:
                prev_pose = self.current_trajectory.poses[i-1]
                prev_x = prev_pose.pose.position.x
                prev_y = prev_pose.pose.position.y
                
                prev_distance = self.distance_between_points(current_x, current_y, prev_x, prev_y)
                
                # If previous point was closer and current is farther, interpolate
                if prev_distance < self.lookahead_distance < distance:
                    # Linear interpolation to find exact lookahead point
                    t = (self.lookahead_distance - prev_distance) / (distance - prev_distance)
                    interp_x = prev_x + t * (point_x - prev_x)
                    interp_y = prev_y + t * (point_y - prev_y)
                    
                    self.target_point_index = i
                    return (interp_x, interp_y)
        
        # If no point found at lookahead distance, use the last point
        if self.current_trajectory.poses:
            last_pose = self.current_trajectory.poses[-1]
            return (last_pose.pose.position.x, last_pose.pose.position.y)
        
        return None
    
    def calculate_pure_pursuit_control(self, lookahead_point: Tuple[float, float]) -> float:
        """
        Calculate angular velocity using Pure Pursuit algorithm
        
        Args:
            lookahead_point: Target point (x, y) coordinates
            
        Returns:
            Angular velocity in rad/s
        """
        if not self.current_pose:
            return 0.0
        
        current_x, current_y, current_yaw = self.current_pose
        target_x, target_y = lookahead_point
        
        # Calculate angle to target point
        angle_to_target = math.atan2(target_y - current_y, target_x - current_x)
        
        # Calculate heading error
        heading_error = angle_to_target - current_yaw
        
        # Normalize heading error to [-pi, pi]
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
        # Calculate curvature using Pure Pursuit formula
        # curvature = 2 * sin(alpha) / L
        # where alpha is the angle between robot heading and line to target
        # and L is the lookahead distance
        
        alpha = heading_error
        curvature = 2 * math.sin(alpha) / self.lookahead_distance
        
        # Angular velocity = linear_velocity * curvature
        angular_velocity = self.linear_velocity * curvature
        
        return angular_velocity
    
    def is_goal_reached(self) -> bool:
        """
        Check if the robot has reached the end of the trajectory
        
        Returns:
            True if goal is reached, False otherwise
        """
        if not self.current_trajectory or not self.current_pose:
            return False
        
        if not self.current_trajectory.poses:
            return True
        
        # Check distance to final point
        final_pose = self.current_trajectory.poses[-1]
        final_x = final_pose.pose.position.x
        final_y = final_pose.pose.position.y
        
        current_x = self.current_pose[0]
        current_y = self.current_pose[1]
        
        distance_to_goal = self.distance_between_points(current_x, current_y, final_x, final_y)
        
        # Goal reached if within 0.1 meters
        return distance_to_goal < 0.1
    
    def trajectory_callback(self, msg: Path):
        """
        Callback function for trajectory subscription
        
        Args:
            msg: Received trajectory message
        """
        # Create a simple hash of the trajectory to detect if it's actually new
        if len(msg.poses) > 0:
            # Hash based on first and last points and number of points
            first_pose = msg.poses[0].pose.position
            last_pose = msg.poses[-1].pose.position
            trajectory_hash = hash((
                len(msg.poses),
                round(first_pose.x, 3),
                round(first_pose.y, 3),
                round(last_pose.x, 3),
                round(last_pose.y, 3)
            ))
            
            # Only reset if this is actually a new trajectory
            if trajectory_hash != self.last_trajectory_hash:
                self.current_trajectory = msg
                self.target_point_index = 0  # Reset target point index only for new trajectories
                self.last_trajectory_hash = trajectory_hash
                
                self.get_logger().info(
                    f'Received NEW trajectory with {len(msg.poses)} points - resetting tracking'
                )
            else:
                # Same trajectory, just update the reference but don't reset tracking
                self.current_trajectory = msg
                self.get_logger().debug('Same trajectory received, continuing tracking')
        else:
            self.current_trajectory = msg
            self.target_point_index = 0
    
    def odom_callback(self, msg: Odometry):
        """
        Callback function for odometry subscription
        
        Args:
            msg: Received odometry message
        """
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Extract orientation and convert to yaw
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        yaw = self.quaternion_to_yaw(qx, qy, qz, qw)
        
        # Update current pose
        self.current_pose = (x, y, yaw)
    
    def control_loop(self):
        """
        Main control loop for Pure Pursuit controller
        """
        # Check if we have necessary data
        if not self.current_pose or not self.current_trajectory:
            # Publish stop command if no data
            cmd_vel = Twist()
            self.cmd_vel_publisher.publish(cmd_vel)
            return
        
        # Check if goal is reached
        if self.is_goal_reached():
            # Stop the robot
            cmd_vel = Twist()
            self.cmd_vel_publisher.publish(cmd_vel)
            
            self.get_logger().info(
                'Goal reached! Stopping robot.',
                throttle_duration_sec=2.0
            )
            return
        
        # Find lookahead point
        lookahead_point = self.find_lookahead_point()
        
        if lookahead_point is None:
            # No valid lookahead point found, stop
            cmd_vel = Twist()
            self.cmd_vel_publisher.publish(cmd_vel)
            return
        
        # Calculate Pure Pursuit control
        angular_velocity = self.calculate_pure_pursuit_control(lookahead_point)
        
        # Create and publish control command
        cmd_vel = Twist()
        cmd_vel.linear.x = self.linear_velocity
        cmd_vel.angular.z = angular_velocity
        
        self.cmd_vel_publisher.publish(cmd_vel)
        
        # Debug logging
        self.get_logger().debug(
            f'Control: linear={cmd_vel.linear.x:.3f}, angular={cmd_vel.angular.z:.3f}, '
            f'target=({lookahead_point[0]:.2f}, {lookahead_point[1]:.2f})'
        )


def main(args=None):
    """Main function to run the trajectory tracker node"""
    rclpy.init(args=args)
    
    try:
        trajectory_tracker = TrajectoryTracker()
        rclpy.spin(trajectory_tracker)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()