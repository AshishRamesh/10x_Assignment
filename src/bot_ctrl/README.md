# Bot Control Package

This ROS 2 package provides a complete trajectory generation and following system for TurtleBot4 using path smoothing and Pure Pursuit control.

## Package Structure

The package consists of three main nodes:

### 1. Path Smoother (`path_smoother.py`)
- **Function**: Generates a smooth path using SciPy cubic spline interpolation
- **Publishes**: `/smooth_path` (nav_msgs/Path)
- **Features**: 
  - Uses hardcoded waypoints: [(0.0, 0.0), (1.0, 0.5), (2.0, 1.0), (2.5, 0.5), (3.0, 0.0)]
  - Generates 150 interpolated points using SciPy parametric splines (splprep/splev)
  - Fallback to SciPy CubicSpline with natural boundary conditions
  - Publishes smooth path every 1 second

### 2. Trajectory Generator (`trajectory_generator.py`)
- **Function**: Converts smooth path to time-parameterized trajectory
- **Subscribes**: `/smooth_path` (nav_msgs/Path)
- **Publishes**: `/trajectory` (nav_msgs/Path)
- **Features**:
  - Uses constant velocity of 0.15 m/s
  - Computes yaw angles from successive points
  - Adds timestamps to each pose for trajectory timing

### 3. Trajectory Tracker (`trajectory_tracker.py`)
- **Function**: Implements Pure Pursuit controller for trajectory following
- **Subscribes**: 
  - `/trajectory` (nav_msgs/Path)
  - `/odom` (nav_msgs/Odometry)
- **Publishes**: `/cmd_vel` (geometry_msgs/Twist)
- **Parameters**:
  - Lookahead distance: 0.5 m
  - Linear velocity: 0.15 m/s
  - Control frequency: 10 Hz

### 4. Path Visualizer (`path_visualizer.py`)
- **Function**: Comprehensive visualization of the entire pipeline
- **Subscribes**:
  - `/smooth_path` (nav_msgs/Path)
  - `/trajectory` (nav_msgs/Path) 
  - `/odom` (nav_msgs/Odometry)
  - `/cmd_vel` (geometry_msgs/Twist)
- **Features**:
  - Combined path plot showing waypoints, smoothed path, trajectory, and tracked path
  - Trajectory comparison graphs (x(t), y(t), heading(t))
  - Tracking error analysis with RMS statistics
  - Automatic PNG generation every 5 seconds
  - Saves to `/visualizations/` directory

## Dependencies

- ROS 2 (tested with Humble)
- Python 3
- NumPy
- SciPy (for advanced smoothing capabilities)
- Matplotlib (for visualization and plotting)
- Standard ROS 2 message packages (nav_msgs, geometry_msgs)

## Installation

1. Clone the package to your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository_url>
```

2. Install dependencies:
```bash
sudo apt install python3-scipy python3-matplotlib
```

3. Build the package:
```bash
cd ~/ros2_ws
colcon build --symlink-install
```

4. Source the workspace:
```bash
source install/setup.bash
```

## Usage

### Launch All Nodes
To start the complete system with all four nodes (including visualization):

```bash
ros2 launch bot_ctrl bot_ctrl_launch.py
```

### Launch Only Visualization
To run only the visualization node (when other nodes are already running):

```bash
ros2 launch bot_ctrl visualization_launch.py
```

### Launch Individual Nodes
You can also run nodes individually for testing:

```bash
# Path smoother only
ros2 run bot_ctrl path_smoother

# Trajectory generator only
ros2 run bot_ctrl trajectory_generator

# Trajectory tracker only
ros2 run bot_ctrl trajectory_tracker

# Path visualizer only
ros2 run bot_ctrl path_visualizer
```

### With TurtleBot4 in Ignition Gazebo

1. Start the TurtleBot4 simulation:
```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py
```

2. In another terminal, launch the bot control system:
```bash
ros2 launch bot_ctrl bot_ctrl_launch.py
```

The robot should automatically start following the predefined trajectory.

## Topics

- `/smooth_path`: Smoothed path with dense waypoints
- `/trajectory`: Time-parameterized trajectory with orientations
- `/cmd_vel`: Velocity commands for the robot
- `/odom`: Robot odometry (subscribed from TurtleBot4)

## Visualization Output

The visualization node automatically generates PNG files every 5 seconds in the `visualizations/` directory:

- **Combined Path Plot**: Shows waypoints, smoothed path, generated trajectory, and tracked trajectory in one graph
- **Trajectory Comparison**: Separate plots for x(t), y(t), and heading(t) comparing generated vs tracked
- **Tracking Error Analysis**: Error plots with RMS statistics for position and heading tracking

## Parameters

All parameters are currently hardcoded but can be easily modified:

- **Waypoints**: Modify the `waypoints` list in `path_smoother.py`
- **Velocity**: Change `linear_velocity` in both `trajectory_generator.py` and `trajectory_tracker.py`
- **Lookahead Distance**: Modify `lookahead_distance` in `trajectory_tracker.py`
- **Number of Path Points**: Change `num_points` in `path_smoother.py`

## Algorithm Details

### Path Smoothing
Uses SciPy's advanced spline interpolation functions to create smooth curves between waypoints. The implementation includes:

**Primary Method - Parametric Splines:**
- Uses `scipy.interpolate.splprep` to create parametric cubic spline representation
- Treats the path as a 2D parametric curve for optimal smoothness
- Automatically handles complex path geometries including loops and sharp turns
- Forces spline to pass through all waypoints exactly (s=0 parameter)

**Fallback Method - Distance-based Splines:**
- Uses `scipy.interpolate.CubicSpline` with natural boundary conditions
- Distance-based parameterization for uniform sampling along the path
- Provides robust interpolation when parametric method encounters issues

**Features:**
- Dense point generation (150 points by default)
- Automatic method selection with graceful fallback
- Optimized performance using SciPy's compiled routines

### Pure Pursuit Controller
Implements the classic Pure Pursuit algorithm:
1. Find lookahead point at specified distance ahead on trajectory
2. Calculate heading error to lookahead point
3. Compute curvature using: `κ = 2sin(α)/L`
4. Set angular velocity: `ω = v * κ`

Where:
- α = angle between robot heading and line to target
- L = lookahead distance
- v = linear velocity

## Troubleshooting

1. **No movement**: Check if `/odom` topic is being published by the robot
2. **Erratic behavior**: Verify trajectory is being received on `/trajectory` topic
3. **Build errors**: Ensure all dependencies are installed, especially SciPy

## Testing

Monitor the system using RViz or command line tools:

```bash
# View topics
ros2 topic list

# Monitor path
ros2 topic echo /smooth_path

# Monitor trajectory
ros2 topic echo /trajectory

# Monitor control commands
ros2 topic echo /cmd_vel
```