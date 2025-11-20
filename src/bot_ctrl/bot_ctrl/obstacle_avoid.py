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
    """Node structure for A* algorithm"""
    x: int
    y: int
    g: float
    h: float
    f: float = field(init=False)
    parent: Optional['AStarNode'] = None
    
    def __post_init__(self):
        """Calculate total cost f = g + h"""
        self.f = self.g + self.h
        
    def __lt__(self, other):
        """Priority queue comparison"""
        return self.f < other.f
    
    def __eq__(self, other):
        """Position-based equality"""
        return self.x == other.x and self.y == other.y
    
    def __hash__(self):
        """Hash for set operations"""
        return hash((self.x, self.y))


class AStarPlanner:
    """A* pathfinding for occupancy grids with obstacle inflation"""
    
    def __init__(self, occupancy_grid: OccupancyGrid, inflation_radius: float = 0.5, 
                 obstacle_threshold: int = 50, proximity_cost_weight: float = 2.0):
        """Initialize A* planner with safety parameters"""
        self.grid = occupancy_grid
        self.width = occupancy_grid.info.width
        self.height = occupancy_grid.info.height
        self.resolution = occupancy_grid.info.resolution
        self.origin_x = occupancy_grid.info.origin.position.x
        self.origin_y = occupancy_grid.info.origin.position.y
        
        self.inflation_radius = inflation_radius
        self.obstacle_threshold = obstacle_threshold
        self.proximity_cost_weight = proximity_cost_weight
        
        self.inflation_cells = int(math.ceil(inflation_radius / self.resolution))
        
        self.inflated_obstacles = self._create_inflated_obstacles()
        self.distance_map = self._compute_distance_to_obstacles()
        
        # 8-connected movement with costs
        self.movements = [
            (-1, -1, 1.414), (-1,  0, 1.0), (-1,  1, 1.414),
            ( 0, -1, 1.0),                   ( 0,  1, 1.0),
            ( 1, -1, 1.414), ( 1,  0, 1.0), ( 1,  1, 1.414)
        ]
    
    def _create_inflated_obstacles(self) -> List[List[bool]]:
        """Create inflated obstacle map with safety margins"""
        inflated = [[False for _ in range(self.width)] for _ in range(self.height)]
        
        # Find original obstacles
        original_obstacles = []
        for y in range(self.height):
            for x in range(self.width):
                if self._is_original_obstacle(x, y):
                    original_obstacles.append((x, y))
        
        # Inflate around each obstacle
        for obs_x, obs_y in original_obstacles:
            for dy in range(-self.inflation_cells, self.inflation_cells + 1):
                for dx in range(-self.inflation_cells, self.inflation_cells + 1):
                    new_x = obs_x + dx
                    new_y = obs_y + dy
                    
                    distance = math.sqrt(dx*dx + dy*dy) * self.resolution
                    
                    if (distance <= self.inflation_radius and 
                        self.is_valid_cell(new_x, new_y)):
                        inflated[new_y][new_x] = True
        
        return inflated
    
    def _is_original_obstacle(self, x: int, y: int) -> bool:
        """Check if cell is original obstacle (before inflation)"""
        if not self.is_valid_cell(x, y):
            return True  # Out of bounds = obstacle
        
        index = y * self.width + x
        occupancy = self.grid.data[index]
        
        return (occupancy >= self.obstacle_threshold or occupancy == -1)
    
    def _compute_distance_to_obstacles(self) -> List[List[float]]:
        """Compute distance from each cell to nearest obstacle"""
        distances = [[float('inf') for _ in range(self.width)] for _ in range(self.height)]
        queue = deque()
        
        # Initialize obstacle cells with zero distance
        for y in range(self.height):
            for x in range(self.width):
                if self._is_original_obstacle(x, y):
                    distances[y][x] = 0.0
                    queue.append((x, y, 0.0))
        
        # BFS distance propagation
        directions = [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]
        
        while queue:
            x, y, current_dist = queue.popleft()
            
            for dx, dy in directions:
                new_x, new_y = x + dx, y + dy
                
                if self.is_valid_cell(new_x, new_y):
                    move_dist = math.sqrt(dx*dx + dy*dy) * self.resolution
                    new_dist = current_dist + move_dist
                    
                    if new_dist < distances[new_y][new_x]:
                        distances[new_y][new_x] = new_dist
                        queue.append((new_x, new_y, new_dist))
        
        return distances
    
    def world_to_map(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """Convert world coordinates to map grid indices"""
        map_x = int((world_x - self.origin_x) / self.resolution)
        map_y = int((world_y - self.origin_y) / self.resolution)
        return map_x, map_y
    
    def map_to_world(self, map_x: int, map_y: int) -> Tuple[float, float]:
        """Convert map grid indices to world coordinates"""
        world_x = map_x * self.resolution + self.origin_x
        world_y = map_y * self.resolution + self.origin_y
        return world_x, world_y
    
    def is_valid_cell(self, x: int, y: int) -> bool:
        """Check if coordinates are within grid bounds"""
        return 0 <= x < self.width and 0 <= y < self.height
    
    def is_obstacle(self, x: int, y: int) -> bool:
        """Check if cell is obstacle (including inflated areas)"""
        if not self.is_valid_cell(x, y):
            return True  # Out of bounds = obstacle
        
        return self.inflated_obstacles[y][x]
    
    def euclidean_heuristic(self, x1: int, y1: int, x2: int, y2: int) -> float:
        """Calculate Euclidean distance heuristic"""
        dx = x2 - x1
        dy = y2 - y1
        return math.sqrt(dx*dx + dy*dy)
    
    def get_neighbors(self, node: AStarNode) -> List[Tuple[int, int, float]]:
        """Generate valid neighbors with proximity costs"""
        neighbors = []
        
        for dx, dy, base_cost in self.movements:
            new_x = node.x + dx
            new_y = node.y + dy
            
            if self.is_valid_cell(new_x, new_y) and not self.is_obstacle(new_x, new_y):
                proximity_cost = self._calculate_proximity_cost(new_x, new_y)
                total_cost = base_cost + proximity_cost
                neighbors.append((new_x, new_y, total_cost))
        
        return neighbors
    
    def _calculate_proximity_cost(self, x: int, y: int) -> float:
        """Calculate cost penalty for proximity to obstacles"""
        distance_to_obstacle = self.distance_map[y][x]
        
        if distance_to_obstacle < self.inflation_radius:
            normalized_dist = distance_to_obstacle / self.inflation_radius
            proximity_penalty = self.proximity_cost_weight * (1.0 - normalized_dist)**2
            return proximity_penalty
        
        return 0.0
    
    def reconstruct_path(self, goal_node: AStarNode) -> List[Tuple[int, int]]:
        """Reconstruct path by following parent pointers"""
        path = []
        current = goal_node
        
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent
        
        path.reverse()
        return path
    
    def plan_path(self, start_x: int, start_y: int, goal_x: int, goal_y: int) -> Optional[List[Tuple[int, int]]]:
        """Execute A* algorithm to find path from start to goal"""
        if self.is_obstacle(start_x, start_y) or self.is_obstacle(goal_x, goal_y):
            return None
        
        open_list = []
        closed_set: Set[Tuple[int, int]] = set()
        
        start_node = AStarNode(
            x=start_x,
            y=start_y,
            g=0.0,
            h=self.euclidean_heuristic(start_x, start_y, goal_x, goal_y)
        )
        
        heapq.heappush(open_list, start_node)
        
        while open_list:
            current_node = heapq.heappop(open_list)
            
            if current_node.x == goal_x and current_node.y == goal_y:
                return self.reconstruct_path(current_node)
            
            closed_set.add((current_node.x, current_node.y))
            
            for neighbor_x, neighbor_y, move_cost in self.get_neighbors(current_node):
                if (neighbor_x, neighbor_y) in closed_set:
                    continue
                
                tentative_g = current_node.g + move_cost
                
                neighbor_node = AStarNode(
                    x=neighbor_x,
                    y=neighbor_y,
                    g=tentative_g,
                    h=self.euclidean_heuristic(neighbor_x, neighbor_y, goal_x, goal_y),
                    parent=current_node
                )
                
                existing_node = None
                for node in open_list:
                    if node.x == neighbor_x and node.y == neighbor_y:
                        existing_node = node
                        break
                
                if existing_node is None:
                    heapq.heappush(open_list, neighbor_node)
                elif tentative_g < existing_node.g:
                    existing_node.g = tentative_g
                    existing_node.f = existing_node.g + existing_node.h
                    existing_node.parent = current_node
        
        return None


class ObstacleAvoidanceNode(Node):
    """ROS2 Node for A* pathfinding with click-to-navigate"""
    
    def __init__(self):
        super().__init__('obstacle_avoid')
        
        self.map_data: Optional[OccupancyGrid] = None
        self.current_pose: Optional[Tuple[float, float]] = None
        self.goal_queue = deque()
        self.active_goal: Optional[Tuple[float, float]] = None
        self.current_state = PlannerState.IDLE
        
        # Publishers
        self.path_publisher = self.create_publisher(Path, '/a_star_path', 10)
        
        # Subscribers
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.clicked_point_sub = self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_callback, 10)
        
        # Timer for state machine updates
        self.state_timer = self.create_timer(0.1, self.state_machine_update)
        
        self.get_logger().info('A* Obstacle Avoidance Node initialized with click-to-navigate')
        self.get_logger().info('Waiting for /map and /odom messages...')
        self.get_logger().info('Click points in RViz to set navigation goals!')
    
    def map_callback(self, msg: OccupancyGrid):
        """Callback for occupancy grid map"""
        self.map_data = msg
        self.get_logger().info(
            f'Received map: {msg.info.width}x{msg.info.height} '
            f'(resolution: {msg.info.resolution:.3f}m)'
        )
    
    def odom_callback(self, msg: Odometry):
        """Callback for robot odometry data"""
        self.current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
    
    def clicked_point_callback(self, msg: PointStamped):
        """Callback for clicked points - add goals to queue"""
        goal = (msg.point.x, msg.point.y)
        self.goal_queue.append(goal)
        
        self.get_logger().info(
            f'Added goal ({goal[0]:.2f}, {goal[1]:.2f}) to queue. '
            f'Queue size: {len(self.goal_queue)}'
        )
    
    def state_machine_update(self):
        """Main state machine update loop"""
        if self.current_state == PlannerState.IDLE:
            if (len(self.goal_queue) > 0 and 
                self.current_pose is not None and 
                self.map_data is not None):
                
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
                
                success = self.plan_and_publish_path()
                
                if success:
                    self.current_state = PlannerState.WAITING_FOR_COMPLETION
                    self.get_logger().info(
                        f'State: PLANNING -> WAITING. Path published to '
                        f'({self.active_goal[0]:.2f}, {self.active_goal[1]:.2f})'
                    )
                else:
                    self.active_goal = None
                    self.current_state = PlannerState.IDLE
                    self.get_logger().warn('Planning failed. State: PLANNING -> IDLE')
                    
        elif self.current_state == PlannerState.WAITING_FOR_COMPLETION:
            if self.goal_reached():
                self.active_goal = None
                self.current_state = PlannerState.IDLE
                
                self.get_logger().info('Goal reached! State: WAITING -> IDLE')
                
                if len(self.goal_queue) > 0:
                    self.get_logger().info(f'Processing next goal. Queue size: {len(self.goal_queue)}')
    
    def goal_reached(self) -> bool:
        """Check if robot has reached the active goal"""
        if self.current_pose is None or self.active_goal is None:
            return False
            
        dx = self.current_pose[0] - self.active_goal[0]
        dy = self.current_pose[1] - self.active_goal[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        return distance < 0.3
    
    def plan_and_publish_path(self) -> bool:
        """Execute A* pathfinding and publish result"""
        try:
            planner = AStarPlanner(self.map_data)
            
            start_map_x, start_map_y = planner.world_to_map(
                self.current_pose[0], self.current_pose[1]
            )
            goal_map_x, goal_map_y = planner.world_to_map(
                self.active_goal[0], self.active_goal[1]
            )
            
            self.get_logger().info(
                f'Planning A* path from ({self.current_pose[0]:.2f}, {self.current_pose[1]:.2f}) '
                f'to ({self.active_goal[0]:.2f}, {self.active_goal[1]:.2f})'
            )
            
            if planner.is_obstacle(start_map_x, start_map_y):
                self.get_logger().warn('Current pose in collision! Skipping goal.')
                self.publish_empty_path()
                return False
            
            if planner.is_obstacle(goal_map_x, goal_map_y):
                self.get_logger().warn('Goal pose in collision! Skipping goal.')
                self.publish_empty_path()
                return False
            
            path_map_coords = planner.plan_path(start_map_x, start_map_y, goal_map_x, goal_map_y)
            
            if path_map_coords is None:
                self.get_logger().warn('No path found! Skipping goal.')
                self.publish_empty_path()
                return False
            
            self.publish_path(planner, path_map_coords)
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error during path planning: {str(e)}')
            self.publish_empty_path()
            return False
    
    def publish_path(self, planner: AStarPlanner, path_map_coords: List[Tuple[int, int]]):
        """Convert map coordinates to world and publish path"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for map_x, map_y in path_map_coords:
            world_x, world_y = planner.map_to_world(map_x, map_y)
            
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = path_msg.header.stamp
            pose_stamped.header.frame_id = path_msg.header.frame_id
            
            pose_stamped.pose.position.x = world_x
            pose_stamped.pose.position.y = world_y
            pose_stamped.pose.position.z = 0.0
            
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose_stamped)
        
        self.path_publisher.publish(path_msg)
        
        self.get_logger().info(
            f'Published A* path with {len(path_map_coords)} waypoints'
        )
    
    def publish_empty_path(self):
        """Publish empty path to indicate no path found"""
        empty_path = Path()
        empty_path.header.stamp = self.get_clock().now().to_msg()
        empty_path.header.frame_id = 'map'
        
        self.path_publisher.publish(empty_path)
        self.get_logger().info('Published empty path')


def main(args=None):
    """Main function to run obstacle avoidance node"""
    rclpy.init(args=args)
    
    try:
        obstacle_avoid_node = ObstacleAvoidanceNode()
        rclpy.spin(obstacle_avoid_node)
    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()