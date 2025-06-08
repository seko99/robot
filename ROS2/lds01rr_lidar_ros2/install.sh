#!/bin/bash

# Installation script for LDS01RR Lidar ROS2 package

set -e

echo "LDS01RR Lidar ROS2 Package Installation Script"
echo "=============================================="

# Check if ROS2 is installed
if ! command -v ros2 &> /dev/null; then
    echo "Error: ROS2 is not installed or not in PATH"
    echo "Please install ROS2 first: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

# Check for ROS2 workspace
if [ -z "$ROS_WS" ]; then
    if [ -d "$HOME/ros2_ws" ]; then
        ROS_WS="$HOME/ros2_ws"
    elif [ -d "$HOME/colcon_ws" ]; then
        ROS_WS="$HOME/colcon_ws"
    else
        echo "ROS2 workspace not found. Creating ~/ros2_ws..."
        mkdir -p "$HOME/ros2_ws/src"
        ROS_WS="$HOME/ros2_ws"
    fi
fi

echo "Using ROS2 workspace: $ROS_WS"

# Install Python dependencies
echo "Installing Python dependencies..."
pip3 install pyserial

# Copy package to workspace
PACKAGE_DIR="$ROS_WS/src/lds01rr_lidar_ros2"
echo "Copying package to $PACKAGE_DIR..."

if [ -d "$PACKAGE_DIR" ]; then
    echo "Package already exists. Removing old version..."
    rm -rf "$PACKAGE_DIR"
fi

cp -r "$(dirname "$0")" "$PACKAGE_DIR"

# Build the package
echo "Building package..."
cd "$ROS_WS"
colcon build --packages-select lds01rr_lidar_ros2

# Source the workspace
echo "Sourcing workspace..."
source "$ROS_WS/install/setup.bash"

echo ""
echo "Installation completed successfully!"
echo ""
echo "To use the package:"
echo "1. Source your workspace: source $ROS_WS/install/setup.bash"
echo "2. Run the node: ros2 run lds01rr_lidar_ros2 lidar_node"
echo "3. Or use launch file: ros2 launch lds01rr_lidar_ros2 lidar_launch.py"
echo ""
echo "Don't forget to set up serial port permissions:"
echo "sudo usermod -a -G dialout \$USER"
echo "Then log out and log back in." 