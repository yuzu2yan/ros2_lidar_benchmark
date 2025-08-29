#!/bin/bash

echo "ROS 2 LiDAR Benchmark Setup Script"
echo "=================================="

# Check ROS 2 installation
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS 2 is not sourced. Please source your ROS 2 installation first."
    exit 1
fi

echo "Detected ROS 2 $ROS_DISTRO"

# Install Python dependencies
echo "Installing Python dependencies (pinning numpy==1.26.*)..."
pip3 install --upgrade 'numpy==1.26.*'
pip3 install psutil matplotlib pandas openpyxl

# Make scripts executable
echo "Making scripts executable..."
chmod +x ros2_lidar_benchmark/scripts/*.py

# Create output directories
echo "Creating output directories..."
mkdir -p /tmp/lidar_benchmark

# Build the package
echo "Building ROS 2 package..."
cd "$(dirname "$0")"
colcon build --packages-select ros2_lidar_benchmark

# Source the workspace
source install/setup.bash

echo ""
echo "Setup complete!"
echo ""
echo "Quick start commands:"
echo "--------------------"
echo "1. Start tcpreplay in another terminal:"
echo "   sudo tcpreplay -i eth0 -l 0 your_lidar_data.pcap"
echo ""
echo "2. Run benchmark with visualization:"
echo "   ros2 launch ros2_lidar_benchmark benchmark.launch.py"
echo ""
echo "3. Run benchmark without visualization:"
echo "   ros2 launch ros2_lidar_benchmark benchmark_headless.launch.py"
echo ""
echo "The benchmark report will be saved to: /tmp/lidar_benchmark_report.xlsx"
