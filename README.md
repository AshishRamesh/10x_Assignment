# 10x_Assignment

## Bot Control Package

This ROS2 package implements a robot control system with path smoothing capabilities using cubic spline interpolation.

### Features

- **Path Smoother Node (`path_smoother.py`)**: Implements cubic spline interpolation for smooth path generation
- Uses only `cmd_vel` for robot control (no Nav2 or external planners)
- Custom cubic spline implementation with natural boundary conditions
- Publishes smoothed paths on `/smooth_path` topic as `nav_msgs/Path`

### Hardcoded Waypoints

The path smoother uses the following hardcoded waypoints:
```python
waypoints = [
    (0.0, 0.0),
    (1.0, 0.5),
    (2.0, 1.0),
    (2.5, 0.5),
    (3.0, 0.0)
]
```

### Build and Run

1. **Build the package**:
   ```bash
   cd /home/ashish/ros2/10x_Assignment
   colcon build --packages-select bot_ctrl
   source install/setup.bash
   ```

2. **Run the path smoother node**:
   ```bash
   ros2 run bot_ctrl path_smoother
   ```

3. **Visualize the path** (optional):
   ```bash
   # In another terminal
   rviz2
   # Add Path display and subscribe to /smooth_path topic
   ```

### Technical Details

- **Cubic Spline Implementation**: Custom natural cubic spline with zero second derivative boundary conditions
- **Path Parameterization**: Uses cumulative distance along waypoints for proper curve parameterization  
- **Sampling Density**: Generates 150 interpolated points (configurable between 100-200)
- **Frame**: Publishes path in 'map' frame
- **Update Rate**: Publishes path every 1 second

### Dependencies

- `rclpy`: ROS2 Python client library
- `nav_msgs`: For Path message type
- `geometry_msgs`: For PoseStamped message type  
- `numpy`: For numerical computations