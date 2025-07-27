#!/bin/bash

echo "=== Fixing RMW/DDS Issues ==="
echo

# Check current RMW implementation
echo "Current RMW implementation:"
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"

echo
echo "Available RMW implementations:"
ros2 pkg list | grep rmw_

echo
echo "Testing with different RMW implementations..."
echo

# Test with CycloneDDS (usually works best)
echo "1. Testing with CycloneDDS:"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "   RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
timeout 5 ros2 topic hz /vlp16/points_filtered

echo
echo "2. Testing with FastDDS:"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
echo "   RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
timeout 5 ros2 topic hz /vlp16/points_filtered

echo
echo "To fix permanently, add to ~/.bashrc:"
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
echo
echo "Or run benchmark with:"
echo "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 launch ros2_lidar_benchmark benchmark.launch.py"