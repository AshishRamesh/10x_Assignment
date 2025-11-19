#!/usr/bin/env python3
"""
A* Obstacle Avoidance Node for ROS2
Implements A* pathfinding algorithm on occupancy grids to avoid obstacles
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, PointStamped
import math
import heapq
from typing import List, Tuple, Optional, Set
from dataclasses import dataclass, field
from collections import deque
from enum import Enum


class PlannerState(Enum):
    """Planner navigation states"""
    IDLE = "idle"
    PLANNING = "planning"
    WAITING_FOR_COMPLETION = "waiting"


@dataclass
class AStarNode:
    """
    Node structure for A* algorithm
    """
    x: int              # Map grid x coordinate
    y: int              # Map grid y coordinate  
    g: float            # Cost from start to this node
    h: float            # Heuristic cost from this node to goal
    f: float = field(init=False)  # Total cost (g + h)
    parent: Optional['AStarNode'] = None
    
    def __post_init__(self):
        """Calculate f after initialization"""
        self.f = self.g + self.h
        
    def __lt__(self, other):
        """Comparison for priority queue (lower f cost has higher priority)"""
        return self.f < other.f
    
    def __eq__(self, other):
        """Equality check based on position"""
        return self.x == other.x and self.y == other.y
    
    def __hash__(self):
        """Hash for set operations"""
        return hash((self.x, self.y))


class AStarPlanner:
    """
    A* pathfinding algorithm implementation for occupancy grids
    """
    
    def __init__(self, occupancy_grid: OccupancyGrid):
        """
        Initialize A* planner with occupancy grid
        
        Args:
            occupancy_grid: ROS2 OccupancyGrid message
        """
        self.grid = occupancy_grid
        self.width = occupancy_grid.info.width
        self.height = occupancy_grid.info.height
        self.resolution = occupancy_grid.info.resolution
        self.origin_x = occupancy_grid.info.origin.position.x
        self.origin_y = occupancy_grid.info.origin.position.y
        
        # 8-connected movement: includes diagonals
        # Format: (dx, dy, cost)
        self.movements = [
            (-1, -1, 1.414),  # Northwest diagonal
            (-1,  0, 1.0),    # West
            (-1,  1, 1.414),  # Southwest diagonal
            ( 0, -1, 1.0),    # North
            ( 0,  1, 1.0),    # South
            ( 1, -1, 1.414),  # Northeast diagonal
            ( 1,  0, 1.0),    # East
            ( 1,  1, 1.414),  # Southeast diagonal
        ]
    
    def world_to_map(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """
        Convert world coordinates to map grid indices
        
        Args:
            world_x: X coordinate in world frame
            world_y: Y coordinate in world frame
            
        Returns:
            Tuple of (map_x, map_y) indices
        """
        map_x = int((world_x - self.origin_x) / self.resolution)
        map_y = int((world_y - self.origin_y) / self.resolution)
        return map_x, map_y
    
    def map_to_world(self, map_x: int, map_y: int) -> Tuple[float, float]:
        """
        Convert map grid indices to world coordinates
        
        Args:
            map_x: X index in map grid
            map_y: Y index in map grid
            
        Returns:
            Tuple of (world_x, world_y) coordinates
        """
        world_x = map_x * self.resolution + self.origin_x
        world_y = map_y * self.resolution + self.origin_y
        return world_x, world_y
    
    def is_valid_cell(self, x: int, y: int) -> bool:
        """
        Check if map cell coordinates are within grid bounds
        
        Args:
            x: Map grid x coordinate
            y: Map grid y coordinate
            
        Returns:
            True if coordinates are valid, False otherwise
        """
        return 0 <= x < self.width and 0 <= y < self.height
    
    def is_obstacle(self, x: int, y: int) -> bool:
        """
        Check if a map cell contains an obstacle
        
        Args:
            x: Map grid x coordinate
            y: Map grid y coordinate
            
        Returns:
            True if cell is obstacle or unknown, False if free
        """
        if not self.is_valid_cell(x, y):
            return True  # Out of bounds treated as obstacle
        
        # Convert 2D coordinates to 1D array index
        index = y * self.width + x
        
        # Get occupancy value
        occupancy = self.grid.data[index]
        
        # 0 = free, 100 = obstacle, -1 or unknown = treated as obstacle
        return occupancy != 0
    
    def euclidean_heuristic(self, x1: int, y1: int, x2: int, y2: int) -> float:
        """
        Calculate Euclidean distance heuristic between two points
        
        Args:
            x1, y1: First point coordinates
            x2, y2: Second point coordinates
            
        Returns:
            Euclidean distance
        """
        dx = x2 - x1
        dy = y2 - y1
        return math.sqrt(dx*dx + dy*dy)
    
    def get_neighbors(self, node: AStarNode) -> List[Tuple[int, int, float]]:
        """
        Generate valid neighbors for a node using 8-connected movement
        
        Args:
            node: Current A* node
            
        Returns:
            List of (x, y, cost) tuples for valid neighbors
        """
        neighbors = []
        
        for dx, dy, cost in self.movements:
            new_x = node.x + dx
            new_y = node.y + dy
            
            # Check if neighbor is valid and not an obstacle
            if self.is_valid_cell(new_x, new_y) and not self.is_obstacle(new_x, new_y):
                neighbors.append((new_x, new_y, cost))
        
        return neighbors
    
    def reconstruct_path(self, goal_node: AStarNode) -> List[Tuple[int, int]]:
        """
        Reconstruct path from goal to start by following parent pointers
        
        Args:
            goal_node: A* node representing the goal
            
        Returns:
            List of (x, y) coordinates from start to goal
        """
        path = []
        current = goal_node
        
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent
        
        # Reverse to get path from start to goal
        path.reverse()
        return path
    
    def plan_path(self, start_x: int, start_y: int, goal_x: int, goal_y: int) -> Optional[List[Tuple[int, int]]]:
        """
        Execute A* algorithm to find path from start to goal
        
        Args:
            start_x, start_y: Start position in map coordinates
            goal_x, goal_y: Goal position in map coordinates
            
        Returns:
            List of (x, y) waypoints if path found, None if no path exists
        """
        # Validate start and goal positions
        if self.is_obstacle(start_x, start_y):
            return None  # Start position is in collision
        
        if self.is_obstacle(goal_x, goal_y):
            return None  # Goal position is in collision
        
        # Initialize A* data structures
        open_list = []  # Priority queue of nodes to explore
        closed_set: Set[Tuple[int, int]] = set()  # Set of explored nodes
        
        # Create start node
        start_node = AStarNode(
            x=start_x,
            y=start_y,
            g=0.0,
            h=self.euclidean_heuristic(start_x, start_y, goal_x, goal_y)
        )
        
        # Add start node to open list
        heapq.heappush(open_list, start_node)
        
        # A* main loop
        while open_list:
            # Get node with lowest f cost
            current_node = heapq.heappop(open_list)
            
            # Check if we reached the goal
            if current_node.x == goal_x and current_node.y == goal_y:
                return self.reconstruct_path(current_node)
            
            # Add current node to closed set
            closed_set.add((current_node.x, current_node.y))
            
            # Explore neighbors
            for neighbor_x, neighbor_y, move_cost in self.get_neighbors(current_node):
                # Skip if neighbor already explored
                if (neighbor_x, neighbor_y) in closed_set:
                    continue
                
                # Calculate tentative g cost
                tentative_g = current_node.g + move_cost
                
                # Create neighbor node
                neighbor_node = AStarNode(
                    x=neighbor_x,
                    y=neighbor_y,
                    g=tentative_g,
                    h=self.euclidean_heuristic(neighbor_x, neighbor_y, goal_x, goal_y),
                    parent=current_node
                )
                
                # Check if this path to neighbor is better
                existing_node = None
                for node in open_list:
                    if node.x == neighbor_x and node.y == neighbor_y:
                        existing_node = node
                        break
                
                if existing_node is None:
                    # New node, add to open list
                    heapq.heappush(open_list, neighbor_node)
                elif tentative_g < existing_node.g:
                    # Better path found, update existing node
                    existing_node.g = tentative_g
                    existing_node.f = existing_node.g + existing_node.h
                    existing_node.parent = current_node
        
        # No path found
        return None


class ObstacleAvoidanceNode(Node):
    """
    ROS2 Node for A* obstacle avoidance pathfinding with click-to-navigate
    """
    
    def __init__(self):
        super().__init__('obstacle_avoid')
        
        # Data storage
        self.map_data: Optional[OccupancyGrid] = None
        self.current_pose: Optional[Tuple[float, float]] = None  # Robot's current position from odometry
        self.goal_queue = deque()  # FIFO queue of clicked goals
        self.active_goal: Optional[Tuple[float, float]] = None  # Currently active goal
        
        # State management
        self.current_state = PlannerState.IDLE
        
        # Publishers
        self.path_publisher = self.create_publisher(
            Path,
            '/a_star_path',
            10
        )
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.clicked_point_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10
        )
        
        # Timer for state machine updates
        self.state_timer = self.create_timer(0.1, self.state_machine_update)  # 10 Hz update
        
        self.get_logger().info('A* Obstacle Avoidance Node initialized with click-to-navigate')
        self.get_logger().info('Waiting for /map and /odom messages...')
        self.get_logger().info('Click points in RViz to set navigation goals!')
    
    def map_callback(self, msg: OccupancyGrid):
        """
        Callback for occupancy grid map
        
        Args:
            msg: OccupancyGrid message
        """
        self.map_data = msg
        self.get_logger().info(
            f'Received map: {msg.info.width}x{msg.info.height} '
            f'(resolution: {msg.info.resolution:.3f}m)'
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
    
    def clicked_point_callback(self, msg: PointStamped):
        """
        Callback for clicked points from RViz
        
        Args:
            msg: PointStamped message from /clicked_point topic
        """
        # Extract goal coordinates
        goal = (msg.point.x, msg.point.y)
        
        # Add to FIFO queue
        self.goal_queue.append(goal)
        
        self.get_logger().info(
            f'Added goal ({goal[0]:.2f}, {goal[1]:.2f}) to queue. '
            f'Queue size: {len(self.goal_queue)}'
        )
    
    def state_machine_update(self):
        """
        Main state machine update loop for goal queue processing
        """
        if self.current_state == PlannerState.IDLE:
            # Check if we have goals to process and required data is available
            if (len(self.goal_queue) > 0 and 
                self.current_pose is not None and 
                self.map_data is not None):
                
                # Take first goal from queue
                self.active_goal = self.goal_queue.popleft()
                self.current_state = PlannerState.PLANNING
                
                self.get_logger().info(
                    f'State: IDLE -> PLANNING. Processing goal '
                    f'({self.active_goal[0]:.2f}, {self.active_goal[1]:.2f})'
                )
                
        elif self.current_state == PlannerState.PLANNING:
            if (self.current_pose is not None and 
                self.active_goal is not None and 
                self.map_data is not None):
                
                # Execute A* planning and publish path
                success = self.plan_and_publish_path()
                
                if success:
                    self.current_state = PlannerState.WAITING_FOR_COMPLETION
                    self.get_logger().info(
                        f'State: PLANNING -> WAITING. Path published to '
                        f'({self.active_goal[0]:.2f}, {self.active_goal[1]:.2f})'
                    )
                else:
                    # Planning failed, return to idle
                    self.active_goal = None
                    self.current_state = PlannerState.IDLE
                    self.get_logger().warn('Planning failed. State: PLANNING -> IDLE')
                    
        elif self.current_state == PlannerState.WAITING_FOR_COMPLETION:
            # Check if robot reached the goal (simplified check)
            if self.goal_reached():
                self.active_goal = None
                self.current_state = PlannerState.IDLE
                
                self.get_logger().info('Goal reached! State: WAITING -> IDLE')
                
                # Check if more goals are queued
                if len(self.goal_queue) > 0:
                    self.get_logger().info(f'Processing next goal. Queue size: {len(self.goal_queue)}')
    
    def goal_reached(self) -> bool:
        """
        Check if the robot has reached the active goal
        
        Returns:
            True if goal is reached, False otherwise
        """
        if self.current_pose is None or self.active_goal is None:
            return False
            
        # Calculate distance to goal
        dx = self.current_pose[0] - self.active_goal[0]
        dy = self.current_pose[1] - self.active_goal[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Consider goal reached if within 0.3 meters
        return distance < 0.3
    
    def plan_and_publish_path(self) -> bool:
        """
        Execute A* pathfinding and publish the result
        
        Returns:
            True if planning and publishing succeeded, False otherwise
        """
        try:
            # Create A* planner
            planner = AStarPlanner(self.map_data)
            
            # Convert world coordinates to map coordinates
            start_map_x, start_map_y = planner.world_to_map(
                self.current_pose[0],
                self.current_pose[1]
            )
            
            goal_map_x, goal_map_y = planner.world_to_map(
                self.active_goal[0],
                self.active_goal[1]
            )
            
            self.get_logger().info(
                f'Planning A* path from current pose ({self.current_pose[0]:.2f}, {self.current_pose[1]:.2f}) '
                f'to goal ({self.active_goal[0]:.2f}, {self.active_goal[1]:.2f})'
            )
            
            # Check for collision at start and goal
            if planner.is_obstacle(start_map_x, start_map_y):
                self.get_logger().warn('Current robot pose is in collision! Skipping this goal.')
                self.publish_empty_path()
                return False
            
            if planner.is_obstacle(goal_map_x, goal_map_y):
                self.get_logger().warn('Goal pose is in collision! Skipping this goal.')
                self.publish_empty_path()
                return False
            
            # Execute A* pathfinding
            path_map_coords = planner.plan_path(start_map_x, start_map_y, goal_map_x, goal_map_y)
            
            if path_map_coords is None:
                self.get_logger().warn('No obstacle-free path found! Skipping this goal.')
                self.publish_empty_path()
                return False
            
            # Convert map coordinates back to world coordinates and publish
            self.publish_path(planner, path_map_coords)
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error during path planning: {str(e)}')
            self.publish_empty_path()
            return False
    
    def publish_path(self, planner: AStarPlanner, path_map_coords: List[Tuple[int, int]]):
        """
        Convert map path to world coordinates and publish as nav_msgs/Path
        
        Args:
            planner: A* planner instance for coordinate conversion
            path_map_coords: List of (x, y) waypoints in map coordinates
        """
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        # Convert each waypoint from map to world coordinates
        for map_x, map_y in path_map_coords:
            world_x, world_y = planner.map_to_world(map_x, map_y)
            
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = path_msg.header.stamp
            pose_stamped.header.frame_id = path_msg.header.frame_id
            
            pose_stamped.pose.position.x = world_x
            pose_stamped.pose.position.y = world_y
            pose_stamped.pose.position.z = 0.0
            
            # Set neutral orientation
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose_stamped)
        
        # Publish the path
        self.path_publisher.publish(path_msg)
        
        self.get_logger().info(
            f'Successfully published A* path with {len(path_map_coords)} waypoints'
        )
    
    def publish_empty_path(self):
        """
        Publish an empty path message to indicate no path found
        """
        empty_path = Path()
        empty_path.header.stamp = self.get_clock().now().to_msg()
        empty_path.header.frame_id = 'map'
        
        self.path_publisher.publish(empty_path)
        self.get_logger().info('Published empty path')


def main(args=None):
    """Main function to run the obstacle avoidance node"""
    rclpy.init(args=args)
    
    try:
        obstacle_avoid_node = ObstacleAvoidanceNode()
        rclpy.spin(obstacle_avoid_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()