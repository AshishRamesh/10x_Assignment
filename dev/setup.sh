#!/bin/bash

# TurtleBot4 Navigation System - Automated Setup Script
# This script automates the installation and build process for the bot_ctrl package

set -e  # Exit on any error

echo "========================================"
echo "TurtleBot4 Navigation System Setup"
echo "========================================"

# Check if we're in a ROS2 workspace
if [ ! -f "src/bot_ctrl/package.xml" ]; then
    echo "❌ Error: bot_ctrl package not found!"
    echo "Please run this script from the root of your ROS2 workspace"
    echo "Expected structure: workspace_root/src/bot_ctrl/"
    exit 1
fi

echo "✅ Found bot_ctrl package"

# Check if ROS2 Humble is installed
if ! command -v ros2 &> /dev/null; then
    echo "❌ Error: ROS2 not found!"
    echo "Please install ROS2 Humble first: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

echo "✅ ROS2 found"

# Check if Gazebo Ignition is available (not mandatory, will be installed)
if command -v ign gazebo &> /dev/null; then
    echo "✅ Gazebo Ignition already installed"
else
    echo "ℹ️  Gazebo Ignition will be installed"
fi

# Source ROS2 setup if available
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "🔧 Sourcing ROS2 Humble..."
    source /opt/ros/humble/setup.bash
else
    echo "⚠️  Warning: ROS2 Humble setup.bash not found in default location"
fi

# Update package lists
echo "🔧 Updating package lists..."
sudo apt update

# Install Python dependencies
echo "🔧 Installing Python dependencies..."
sudo apt install -y python3-numpy python3-scipy python3-matplotlib

# Install Gazebo Ignition
echo "🔧 Installing Gazebo Ignition..."
sudo apt install -y ignition-gazebo6 ros-humble-ros-ign-gazebo

# Install TurtleBot4 simulation packages
echo "🔧 Installing TurtleBot4 packages..."
sudo apt install -y ros-humble-turtlebot4-simulator ros-humble-irobot-create-nodes

# Install additional ROS2 packages that might be needed
echo "🔧 Installing additional ROS2 packages..."
sudo apt install -y ros-humble-nav2-bringup ros-humble-navigation2 ros-humble-nav2-msgs

# Install rosdep if not already installed
if ! command -v rosdep &> /dev/null; then
    echo "🔧 Installing rosdep..."
    sudo apt install -y python3-rosdep
    sudo rosdep init
fi

# Update rosdep
echo "🔧 Updating rosdep database..."
rosdep update

# Install dependencies for the workspace
echo "🔧 Installing workspace dependencies..."
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
echo "🔧 Building workspace with colcon..."
colcon build --symlink-install

# Check if build was successful
if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Setup completed successfully!"
    echo ""
    echo "Next steps:"
    echo "1. source install/setup.bash"
    echo "2. ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true"
    echo "3. ros2 launch bot_ctrl <launch_file>.py"
    echo ""
else
    echo "❌ Build failed! Check errors above."
    exit 1
fi