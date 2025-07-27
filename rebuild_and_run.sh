#!/bin/bash

echo "Rebuilding ROS 2 LiDAR Benchmark package..."
echo "=========================================="

# Navigate to workspace
cd ~/ros2_ws || cd ~/ros2_lidar_benchmark/..

# Clean previous build
echo "Cleaning previous build..."
rm -rf build/ros2_lidar_benchmark install/ros2_lidar_benchmark

# Build the package
echo "Building package..."
colcon build --packages-select ros2_lidar_benchmark --symlink-install

# Source the workspace
echo "Sourcing workspace..."
source install/setup.bash

echo ""
echo "Build complete!"
echo ""

# Test parameter types
echo "Testing parameter types..."
cd ~/ros2_lidar_benchmark
python3 test_parameters.py

echo ""
echo "To run the benchmark:"
echo "ros2 launch ros2_lidar_benchmark benchmark.launch.py"