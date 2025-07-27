#!/bin/bash

echo "Quick rebuild of ros2_lidar_benchmark package..."
echo "=============================================="

# Get the workspace directory
if [ -d "$HOME/ros2_ws" ]; then
    WS_DIR="$HOME/ros2_ws"
elif [ -d "../.." ] && [ -f "../../src/ros2_lidar_benchmark/package.xml" ]; then
    WS_DIR="../.."
else
    WS_DIR="$HOME/ros2_lidar_benchmark/.."
fi

echo "Using workspace: $WS_DIR"
cd "$WS_DIR"

# Build only the benchmark package
echo "Building ros2_lidar_benchmark..."
colcon build --packages-select ros2_lidar_benchmark --symlink-install

# Source the workspace
echo "Sourcing workspace..."
source install/setup.bash

echo ""
echo "Build complete!"
echo ""
echo "You can now run:"
echo "  ros2 launch ros2_lidar_benchmark benchmark.launch.py"