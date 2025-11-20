#!/usr/bin/env python3
"""
Trajectory Tracker Node for ROS2
Implements Pure Pursuit controller for trajectory following
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
import math
from typing import Optional, Tuple


class TrajectoryTracker(Node):
    """ROS2 Node implementing Pure Pursuit controller for trajectory tracking"""
    
    def __init__(self):
        super().__init__('trajectory_tracker')
        
        self.lookahead_distance = 0.5  # meters
        self.linear_velocity = 0.5     # m/s
        
        self.current_pose = None
        self.current_trajectory = None
        self.target_point_index = 0
        self.last_trajectory_hash = None
        
        self.trajectory_subscriber = self.create_subscription(
            Path, '/trajectory', self.trajectory_callback, 10)
        
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/cmd_vel', 10)
        
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        self.get_logger().info('Trajectory Tracker Node initialized')
        self.get_logger().info(f'Lookahead: {self.lookahead_distance}m, Velocity: {self.linear_velocity}m/s')
    
    def quaternion_to_yaw(self, qx: float, qy: float, qz: float, qw: float) -> float:
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def distance_between_points(self, x1: float, y1: float, x2: float, y2: float) -> float:
        """Calculate Euclidean distance between two points"""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def find_lookahead_point(self) -> Optional[Tuple[float, float]]:
        """Find lookahead point on trajectory using Pure Pursuit algorithm"""
        if not self.current_trajectory or not self.current_pose or not self.current_trajectory.poses:
            return None
        
        current_x, current_y = self.current_pose[0], self.current_pose[1]
        
        # Search from current target point index
        for i in range(self.target_point_index, len(self.current_trajectory.poses)):
            pose = self.current_trajectory.poses[i]
            point_x = pose.pose.position.x
            point_y = pose.pose.position.y
            
            distance = self.distance_between_points(current_x, current_y, point_x, point_y)
            
            # Use point if at or beyond lookahead distance
            if distance >= self.lookahead_distance:
                self.target_point_index = i
                return (point_x, point_y)
            
            # Interpolate between points if needed
            if i > 0 and distance < self.lookahead_distance:
                prev_pose = self.current_trajectory.poses[i-1]
                prev_x = prev_pose.pose.position.x
                prev_y = prev_pose.pose.position.y
                
                prev_distance = self.distance_between_points(current_x, current_y, prev_x, prev_y)
                
                if prev_distance < self.lookahead_distance < distance:
                    # Linear interpolation to exact lookahead point
                    t = (self.lookahead_distance - prev_distance) / (distance - prev_distance)
                    interp_x = prev_x + t * (point_x - prev_x)
                    interp_y = prev_y + t * (point_y - prev_y)
                    
                    self.target_point_index = i
                    return (interp_x, interp_y)
        
        # Use last point if no lookahead point found
        if self.current_trajectory.poses:
            last_pose = self.current_trajectory.poses[-1]
            return (last_pose.pose.position.x, last_pose.pose.position.y)
        
        return None
    
    def calculate_pure_pursuit_control(self, lookahead_point: Tuple[float, float]) -> float:
        """Calculate angular velocity using Pure Pursuit algorithm"""
        if not self.current_pose:
            return 0.0
        
        current_x, current_y, current_yaw = self.current_pose
        target_x, target_y = lookahead_point
        
        # Calculate angle to target
        angle_to_target = math.atan2(target_y - current_y, target_x - current_x)
        
        # Calculate and normalize heading error to [-pi, pi]
        heading_error = angle_to_target - current_yaw
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
        # Pure Pursuit formula: curvature = 2 * sin(alpha) / L
        alpha = heading_error
        curvature = 2 * math.sin(alpha) / self.lookahead_distance
        
        return self.linear_velocity * curvature
    
    def is_goal_reached(self) -> bool:
        """Check if robot has reached the trajectory end"""
        if not self.current_trajectory or not self.current_pose:
            return False
        
        if not self.current_trajectory.poses:
            return True
        
        final_pose = self.current_trajectory.poses[-1]
        final_x = final_pose.pose.position.x
        final_y = final_pose.pose.position.y
        
        current_x = self.current_pose[0]
        current_y = self.current_pose[1]
        
        distance_to_goal = self.distance_between_points(current_x, current_y, final_x, final_y)
        
        return distance_to_goal < 0.1  # Goal reached within 0.1m
    
    def trajectory_callback(self, msg: Path):
        """Process received trajectory and detect if it's new"""
        if len(msg.poses) > 0:
            # Create hash to detect new trajectory
            first_pose = msg.poses[0].pose.position
            last_pose = msg.poses[-1].pose.position
            trajectory_hash = hash((
                len(msg.poses),
                round(first_pose.x, 3), round(first_pose.y, 3),
                round(last_pose.x, 3), round(last_pose.y, 3)
            ))
            
            # Reset tracking only for new trajectories
            if trajectory_hash != self.last_trajectory_hash:
                self.current_trajectory = msg
                self.target_point_index = 0
                self.last_trajectory_hash = trajectory_hash
                
                self.get_logger().info(f'New trajectory: {len(msg.poses)} points - resetting tracking')
            else:
                # Same trajectory, continue tracking
                self.current_trajectory = msg
                self.get_logger().debug('Same trajectory received, continuing tracking')
        else:
            self.current_trajectory = msg
            self.target_point_index = 0
    
    def odom_callback(self, msg: Odometry):
        """Update current robot pose from odometry"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        yaw = self.quaternion_to_yaw(qx, qy, qz, qw)
        self.current_pose = (x, y, yaw)
    
    def control_loop(self):
        """Main Pure Pursuit control loop"""
        # Stop if no data available
        if not self.current_pose or not self.current_trajectory:
            cmd_vel = Twist()
            self.cmd_vel_publisher.publish(cmd_vel)
            return
        
        # Stop if goal reached
        if self.is_goal_reached():
            cmd_vel = Twist()
            self.cmd_vel_publisher.publish(cmd_vel)
            
            self.get_logger().info('Goal reached! Stopping robot.', throttle_duration_sec=2.0)
            return
        
        # Find lookahead point
        lookahead_point = self.find_lookahead_point()
        
        if lookahead_point is None:
            cmd_vel = Twist()
            self.cmd_vel_publisher.publish(cmd_vel)
            return
        
        # Calculate and publish control command
        angular_velocity = self.calculate_pure_pursuit_control(lookahead_point)
        
        cmd_vel = Twist()
        cmd_vel.linear.x = self.linear_velocity
        cmd_vel.angular.z = angular_velocity
        
        self.cmd_vel_publisher.publish(cmd_vel)
        
        self.get_logger().debug(
            f'Control: v={cmd_vel.linear.x:.3f}, ω={cmd_vel.angular.z:.3f}, '
            f'target=({lookahead_point[0]:.2f}, {lookahead_point[1]:.2f})')


def main(args=None):
    """Main function to run the trajectory tracker node"""
    rclpy.init(args=args)
    
    try:
        trajectory_tracker = TrajectoryTracker()
        rclpy.spin(trajectory_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()