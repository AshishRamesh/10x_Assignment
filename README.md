# Path Smoothing and Trajectory Control in 2D Space

## Introduction

This project implements a comprehensive autonomous navigation system for the TurtleBot4 platform in ROS2 Humble, extending from a basic path smoothing assignment to a full-featured navigation suite. The system combines advanced path planning algorithms including cubic spline interpolation, A* obstacle avoidance, and nonlinear trajectory tracking to enable autonomous robot navigation in both static and dynamic environments.

The project demonstrates the complete pipeline from waypoint specification (either hardcoded or interactively selected in RViz) through path smoothing, obstacle-aware trajectory generation, and precise robot control using Pure Pursuit algorithms. Built for TurtleBot4 simulation in Gazebo, the system provides a robust foundation for autonomous mobile robot applications.

---

## Table of Contents

- [Introduction](#introduction)
- [Demo Overview](#demo-overview)
- [Features](#features)
- [Setup & Installation](#setup--installation)
- [Running the System](#running-the-system)
- [File Structure](#file-structure)
- [System Architecture](#system-architecture)
- [Algorithms & Design Choices](#algorithms--design-choices)
- [Extending to a Real Robot](#extending-to-real-robot)
- [Future Improvements](#future-improvements)

---

## Demo Overview

<p align="center">
  <img src="assets/turtlebot4.png" alt="TurtleBot4 Robot Platform" width="400">
</p>
<p align="center"><b>Fig: TurtleBot4 Robot Platform</b></p>

<p align="center">
  <img src="assets/maze_world.png" alt="Maze World Environment for Navigation Testing" width="800">
</p>
<p align="center"><b>Fig: Maze World Environment for Navigation Testing</b></p>

<p align="center">
  <a href="https://youtu.be/egJDMCNASA8" target="_blank">
    <img src="https://img.shields.io/badge/🎥_Watch_Demo-FF0000?style=for-the-badge&logo=youtube&logoColor=white" alt="Watch Demo Video"/>
  </a>
</p>
<p align="center"><b>📺 Complete System Demonstration Video</b></p>

### Demo 1 — Multi-Waypoint Path Smoothing
This demonstration showcases the core path smoothing pipeline using predefined waypoints:

**Waypoints**: `[(-2.75, -0.82), (-3.75, -0.1), (-3.80, 0.88), (-1.81, 0.88), (-1.71, -0.04)]`

The system generates smooth cubic spline trajectories from these discrete points, creating 200 interpolated waypoints for continuous robot motion. The smoothing algorithm uses SciPy's parametric spline functions (`splprep`/`splev`) with automatic fallback to distance-based cubic splines for robustness.

**Output**: High-resolution path visualization showing original waypoints vs. smoothed trajectory.

https://github.com/user-attachments/assets/165494e0-693a-4171-813b-d392dcd5c804
<p align="center"><b>Hardcoded Waypoints Demo</b></p>

<table>
<tr>
<td width="50%">
<p align="center">
  <img src="assets/hardcoded_path.png" alt="Hardcoded Waypoint Path Visualization" width="380" height="285" style="max-width: 380px; max-height: 285px; object-fit: contain;">
</p>
<p align="center"><b>Fig: Hardcoded Waypoint Path Visualization</b></p>
</td>
<td width="50%">
<p align="center">
  <img src="assets/hardcoded_graph.png" alt="Hardcoded Waypoint Performance Graph" width="380" height="285" style="max-width: 380px; max-height: 285px; object-fit: contain;">
</p>
<p align="center"><b>Fig: Hardcoded Waypoint Performance Graph</b></p>
</td>
</tr>
</table>

### Demo 2 — Interactive RViz Waypoint Selection  
This demo enables real-time navigation goal specification through RViz interface:

**Workflow**: User clicks points in RViz → goals queued in FIFO order → automatic path smoothing and execution

The `WaypointManager` node subscribes to `/clicked_point` messages, building a navigation queue that's processed sequentially. Each goal triggers path smoothing from the robot's current position, enabling intuitive exploration and navigation.

**Output**: Dynamic trajectory generation and real-time robot movement following clicked waypoints.

https://github.com/user-attachments/assets/ac689dd3-56ff-4feb-8cd6-cb5ef660e691
<p align="center"><b>Interactive RViz Waypoints Demo</b></p>

<table>
<tr>
<td width="50%">
<p align="center">
  <img src="assets/interactive_path.png" alt="Interactive RViz Waypoint Path" width="380" height="285" style="max-width: 380px; max-height: 285px; object-fit: contain;">
</p>
<p align="center"><b>Fig: Interactive RViz Waypoint Path</b></p>
</td>
<td width="50%">
<p align="center">
  <img src="assets/interactive_graph.png" alt="Interactive Navigation Performance Graph" width="380" height="285" style="max-width: 380px; max-height: 285px; object-fit: contain;">
</p>
<p align="center"><b>Fig: Interactive Navigation Performance Graph</b></p>
</td>
</tr>
</table>

### Demo 3 — Obstacle Avoidance Navigation
Theis demo integrates A* pathfinding for collision-free navigation:

**Pipeline**: A* obstacle-free path generation → spline smoothing refinement → trajectory tracking execution

The system uses the occupancy grid from `/map` to compute obstacle-free paths using A* search with safety inflation. The resulting grid-based path is then refined using the same smoothing pipeline for optimal robot motion.

**Output**: Collision-free navigation with visualization of A* paths vs. smoothed trajectories.

https://github.com/user-attachments/assets/efce912e-8231-47b3-8b2b-e338e6428267
<p align="center"><b>Obstacle Avoidance Demo</b></p>

<table>
<tr>
<td width="50%">
<p align="center">
  <img src="assets/auto_nav_path.png" alt="Obstacle Avoidance Navigation Path" width="380" height="285" style="max-width: 380px; max-height: 285px; object-fit: contain;">
</p>
<p align="center"><b>Fig: Obstacle Avoidance Navigation Path</b></p>
</td>
<td width="50%">
<p align="center">
  <img src="assets/auto_nav_graph.png" alt="Obstacle Avoidance Performance Graph" width="380" height="285" style="max-width: 380px; max-height: 285px; object-fit: contain;">
</p>
<p align="center"><b>Fig: Obstacle Avoidance Performance Graph</b></p>
</td>
</tr>
</table>

### Visualization Examples
```
visualization_output/
├── path_visualization_YYYYMMDD_HHMMSS.png    # Combined path plots
└── tracking_error_YYYYMMDD_HHMMSS.png        # Error analysis graphs
```

---

## Features

### Core Navigation Capabilities
- **Multi-method Path Smoothing**: SciPy parametric splines with distance-based fallback for robust interpolation
- **Interactive Waypoint Selection**: RViz `/clicked_point` integration for real-time navigation goal specification
- **Time-Parameterized Trajectory Generation**: Constant velocity profiles with proper orientation computation
- **Nonlinear Trajectory Tracking**: Pure Pursuit controller with configurable lookahead distance
- **A* Grid-Based Obstacle Avoidance**: Complete occupancy grid pathfinding with safety inflation
- **TF2 Transform Integration**: Robust frame handling between `map` and `base_link` coordinate systems

### Advanced Features
- **Comprehensive Visualization**: Real-time path plotting, trajectory comparison, and tracking error analysis
- **State Machine Architecture**: Robust goal queue management and execution monitoring  
- **Obstacle Proximity Costing**: Exponential penalty functions for safer path planning
- **Dynamic Goal Management**: FIFO queue system for multiple waypoint navigation
- **Error Recovery**: Graceful failure handling and automatic replanning capabilities

---

## Setup & Installation

### Prerequisites
- **ROS2 Humble** (Ubuntu 22.04 recommended)
- **Gazebo Ignition** for simulation
- **Git** for cloning the repository

### Automated Installation (Recommended)

1. **Clone the repository into your ROS2 workspace:**
```bash
git clone https://github.com/AshishRamesh/10x_Assignment
cd ~/10x_Assignment
```

2. **Run the automated setup script:**
```bash
./dev/setup.sh
```

The script will automatically:
- Install Python dependencies (NumPy, SciPy, Matplotlib)
- Install TurtleBot4 simulation packages
- Install additional ROS2 navigation packages
- Build the workspace with colcon
- Provide next steps for verification

### Manual Installation (Alternative)

If you prefer manual installation:

1. **Clone the repository:**
```bash
git clone https://github.com/AshishRamesh/10x_Assignment
cd ~/10x_Assignment
```

2. **Install system dependencies:**
```bash
sudo apt update
sudo apt install python3-numpy python3-scipy python3-matplotlib
sudo apt install ignition-gazebo6 ros-humble-ros-ign-gazebo
sudo apt install ros-humble-turtlebot4-simulator ros-humble-irobot-create-nodes
```

3. **Build the workspace:**
```bash
cd ~/ros2_ws
colcon build --symlink-install
```

4. **Source the workspace:**
```bash
source install/setup.bash
```

### Verify Installation
```bash
ros2 pkg list | grep bot_ctrl
ros2 interface list | grep nav_msgs
```

---

## Running the System

### 1. Start TurtleBot4 Simulation + Map

Ensure the following commands are run at the root of the workspace:
Example:
If your workspace is '10x_Assignment', run the commands from there after sourcing the workspace.

```bash
# Terminal 1: Start TurtleBot4 simulation with mapping
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py rviz:=true world:=maze localization:=true map:="src/bot_ctrl/maps/maze_map.yaml"
```

### 2. Launch Navigation System

Choose one of the following launch configurations:

#### Complete Obstacle Avoidance System
```bash
# Terminal 2: Full system with A* planning
ros2 launch bot_ctrl obstacle_avoidance_launch.py
```

#### Interactive RViz Navigation
```bash  
# Terminal 2: Click-to-navigate system
ros2 launch bot_ctrl clicked_waypoints_launch.py
```

#### Hardcoded Waypoint Navigation
```bash
# Terminal 2: Static waypoint following
ros2 launch bot_ctrl hardcoded_waypoints_launch.py
```

### 3. Using RViz for Interactive Navigation and Obstacle Avoidance

   - Select "Publish Point" tool
   - Click desired locations on the map
   - Robot automatically navigates to clicked points in sequence

---

## File Structure

```
10x_Assignment/
├── src/bot_ctrl/                       # ROS2 package source
│   ├── bot_ctrl/
│   │   ├── obstacle_avoid.py           # A* pathfinding implementation
│   │   ├── smoother_obstacle.py        # A* path smoothing  
│   │   ├── smoother_clicked.py         # Click-based waypoint manager
│   │   ├── smoother.py                 # Hardcoded waypoint smoother
│   │   ├── planner.py                  # Trajectory generator
│   │   ├── controller.py               # Pure Pursuit tracker
│   │   └── visualizer.py               # Path visualization tools
│   ├── launch/
│   │   ├── obstacle_avoidance_launch.py # Complete obstacle avoidance system
│   │   ├── clicked_waypoints_launch.py  # Interactive RViz navigation  
│   │   ├── hardcoded_waypoints_launch.py # Static waypoint navigation
│   │   └── visualization_launch.py     # Visualization-only mode
│   └── maps/
│       ├── maze_map.pgm                # Maze world occupancy grid data
│       └── maze_map.yaml               # Map metadata and configuration
├── dev/                                # Development tools
│   └── setup.sh                        # Automated installation script
├── assets/                             # Documentation images
├── visualization_output/               # Generated analysis plots
├── build/                              # Colcon build artifacts
├── install/                            # Installed packages
└── log/                                # Build logs
```

### Node Responsibilities

| Node | Purpose | Topics | Key Features |
|------|---------|---------|--------------|
| `obstacle_avoid` | A* pathfinding | `/clicked_point` → `/a_star_path` | Grid-based planning, safety inflation |
| `smoother_obstacle` | Path refinement | `/a_star_path` → `/smooth_path` | Spline smoothing of A* output |
| `waypoint_manager` | Interactive navigation | `/clicked_point` → `/smooth_path` | Click-to-navigate, goal queueing |
| `planner` | Trajectory generation | `/smooth_path` → `/trajectory` | Time parameterization, orientation |
| `controller` | Motion control | `/trajectory` + `/odom` → `/cmd_vel` | Pure Pursuit, lookahead control |
| `visualizer` | System monitoring | Multiple → PNG files | Error analysis, performance metrics |

---

## System Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   RViz User     │    │  Hardcoded       │    │  Occupancy      │
│  /clicked_point │────│  Waypoints       │    │     Grid        │
└─────────────────┘    └──────────────────┘    │    /map         │
         │                       │              └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│ waypoint_manager│    │     smoother     │    │ obstacle_avoid  │
│   (clicked)     │    │   (hardcoded)    │    │   (A* planner)  │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         │                       │                       │
         │            /smooth_path         /a_star_path   │
         └──────────────┬──────────────────────┬─────────┘
                        │                      │
                        ▼                      ▼
              ┌─────────────────┐    ┌──────────────────┐
              │     planner     │    │ smoother_obstacle│
              │ (trajectory gen)│    │  (A* smoother)   │
              └─────────────────┘    └──────────────────┘
                        │                      │
                        │         /smooth_path │
                        └──────────┬───────────┘
                                   │
                                   ▼ /trajectory
                        ┌─────────────────┐
                        │   controller    │
                        │ (Pure Pursuit)  │
                        └─────────────────┘
                                   │
                                   ▼ /cmd_vel
                        ┌─────────────────┐
                        │   TurtleBot4    │
                        │   Simulation    │
                        └─────────────────┘
                                   │
                                   ▼ /odom
                        ┌─────────────────┐
                        │   visualizer    │
                        │ (Path Analysis) │
                        └─────────────────┘
```

### Key Data Flows

1. **Waypoint Input**: Multiple sources feed navigation goals
2. **Path Planning**: A* generates obstacle-free grid paths  
3. **Path Smoothing**: Cubic splines create continuous trajectories
4. **Trajectory Generation**: Time parameterization with velocity profiles
5. **Trajectory Tracking**: Pure Pursuit controller generates velocity commands
6. **State Feedback**: Odometry provides position feedback for control and visualization

### Frame Coordination

The navigation system implements a modular pipeline architecture with clear data flow and responsibilities:

<p align="center">
  <img src="assets/topic.png" alt="System Architecture and Topic Flow Diagram" width="800">
</p>
<p align="center"><b>Fig: System Architecture and Topic Flow Diagram</b></p>

---

## Algorithms & Design Choices

### Path Smoothing

**Algorithm**: Cubic Spline Interpolation with Parametric Representation

The system implements a dual-method smoothing approach for robustness:

**Primary Method - Parametric Splines:**
- Uses `scipy.interpolate.splprep` for 2D parametric curve representation
- Treats waypoints as control points for smooth curve generation  
- Handles complex geometries including loops and sharp turns
- Generates 150 interpolated points for high-resolution paths

**Fallback Method - Distance-Based Splines:**
- Uses `scipy.interpolate.CubicSpline` with natural boundary conditions
- Distance-parameterized sampling for uniform point distribution
- Provides robust interpolation when parametric method fails

**Design Rationale**: Cubic splines chosen over Bézier curves for exact waypoint interpolation and continuous second derivatives, ensuring smooth robot motion without oscillations.

### Trajectory Generation

**Algorithm**: Constant Velocity Time Parameterization

**Implementation**:
```python
time_offset = cumulative_distance / linear_velocity  # 0.5 m/s
yaw = atan2(y_next - y_current, x_next - x_current)
```

**Features**:
- Constant linear velocity (0.5 m/s) for predictable motion
- Timestamp assignment based on path distance
- Orientation computation from successive waypoint vectors
- Starting trajectory from current robot pose (via `/odom`)

**Design Rationale**: Constant velocity profiles chosen for simplicity and predictability. More sophisticated velocity planning (considering curvature limits) reserved for future improvements.

### Trajectory Tracking

**Algorithm**: Pure Pursuit Controller

**Mathematical Foundation**:
```
Lookahead Point: L = 0.5m ahead on path
Heading Error: α = angle between robot heading and line to target  
Curvature: κ = 2×sin(α) / L
Angular Velocity: ω = v × κ
```

**Implementation Details**:
- **Lookahead Distance**: 0.5m (tuned for TurtleBot4 dynamics)
- **Control Frequency**: 10Hz for responsive tracking
- **Linear Velocity**: 0.5m/s (increased from original 0.15m/s)
- **Goal Tolerance**: 0.3m for robust goal detection

**Design Rationale**: Pure Pursuit selected for its simplicity, stability, and proven performance with differential drive robots. The geometric nature provides intuitive tuning parameters.

### Obstacle Avoidance

**Algorithm**: A* Search with Safety Inflation and Proximity Costing

**Grid Processing Pipeline**:
1. **Obstacle Inflation**: Expand obstacles by safety radius (0.5m)
2. **Distance Field Computation**: BFS-based distance to nearest obstacle
3. **Proximity Costing**: Exponential penalty near obstacles
4. **A* Search**: 8-connected movement with heuristic guidance

**Safety Features**:
```python
# Obstacle inflation for robot radius safety
inflation_radius = 0.5m  # Robot radius + safety margin

# Exponential proximity cost near obstacles  
if distance_to_obstacle < inflation_radius:
    penalty = weight × (1 - normalized_distance)²
```

**Design Rationale**: A* chosen for optimal pathfinding with safety guarantees. Grid-based planning in `/map` frame avoids odometry drift issues. Post-smoothing applied to A* output for optimal robot motion.

### Frame Handling & Coordinate Transforms

**Architecture**: TF2-Based Multi-Frame Coordination

**Frame Strategy**:
- **Planning Frame**: `/map` - Global, drift-free planning
- **Control Frame**: `/odom` - Smooth, continuous control
- **Robot Frame**: `/base_link` - Local robot coordinates

**Critical Design Decision**: Planning in `/map` frame prevents accumulated odometry drift from affecting long-range navigation, while control in `/odom` frame ensures smooth trajectory tracking.

---

## Extending to a Real Robot

### Hardware Integration Requirements

**SLAM & Localization**:
- Replace simulation mapping with real SLAM (e.g., `slam_toolbox`)
- Implement robust localization using AMCL or equivalent
- Handle GPS-denied indoor navigation scenarios

**Sensor Integration**:
- **LiDAR**: Real-time obstacle detection and map updates
- **Camera**: Visual odometry and landmark-based localization  
- **IMU**: Improved orientation estimation and dead reckoning
- **Wheel Encoders**: High-frequency odometry updates

**Real-World Adaptations**:

1. **Dynamic Obstacle Handling**:
```python
# Emergency stop based on laser scan
def laser_callback(self, scan_msg):
    min_distance = min(scan_msg.ranges)
    if min_distance < EMERGENCY_DISTANCE:
        self.publish_stop_command()
```

2. **Sensor Noise Filtering**:
- Implement Kalman filtering for state estimation
- Use sensor fusion for robust localization
- Handle sensor failures gracefully

3. **Computational Constraints**:
- Optimize A* grid resolution for real-time performance
- Implement hierarchical planning for large environments
- Use onboard computation-friendly controllers

4. **Safety Systems**:
- Laser-based emergency stopping
- Cliff detection for mobile platforms
- Battery monitoring and return-to-dock behaviors

### Real-Time Considerations

**Performance Optimizations**:
- Reduce path planning frequency (1-2Hz vs continuous)
- Implement path caching and incremental updates
- Use multi-threading for sensor processing and control

---

## Future Improvements

### Navigation Enhancements

**Dynamic Obstacle Avoidance**:
- Implement D* Lite for real-time replanning
- Add velocity obstacles for moving obstacle prediction
- Integrate sensor-based reactive behaviors

**Advanced Path Planning**:
- **Curvature-Constrained Planning**: RRT* or hybrid A* for kinodynamic constraints
- **Multi-Objective Optimization**: Balance path length, safety, and energy efficiency  
- **Hierarchical Planning**: Coarse global planning with fine local adjustments

### Control System Improvements  

**Sophisticated Velocity Profiles**:
- **Curvature-Based Speed Control**: Reduce velocity in sharp turns
- **Acceleration Limits**: Smooth acceleration/deceleration profiles
- **Dynamic Window Approach**: Real-time velocity optimization

**Advanced Controllers**:
- **Model Predictive Control (MPC)**: Optimal trajectory tracking with constraints
- **Adaptive Pure Pursuit**: Dynamic lookahead distance based on velocity and curvature
- **Learning-Based Control**: Neural network controllers for complex environments

### System Integration

**Full Nav2 Integration**:
- Replace custom planner with Nav2 planner plugins
- Integrate behavior trees for complex navigation tasks
- Add recovery behaviors for navigation failures

**Real-Time Capabilities**:
- **ROS2 Real-Time**: Deterministic control loops for safety-critical applications  
- **Hardware Acceleration**: GPU-based path planning for complex environments
- **Distributed Computing**: Multi-robot coordination and planning

### Robustness & Safety

**Fault Tolerance**:
- Sensor failure detection and graceful degradation
- Multiple planning algorithm fallbacks  
- Network connectivity loss handling

**Safety Systems**:
- Certified safe control barriers for obstacle avoidance
- Formal verification of control algorithms
- Emergency stop and recovery protocols

---

*This project demonstrates advanced autonomous navigation capabilities suitable for research and industrial applications. The modular architecture enables easy extension and customization for specific robotic platforms and mission requirements.*


