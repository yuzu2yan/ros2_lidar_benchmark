#!/bin/bash

echo "=== Testing Point Cloud Receiver Node ==="
echo

# Test with BEST_EFFORT QoS (default)
echo "1. Testing with BEST_EFFORT QoS (recommended for sensor data):"
echo "Running: ros2 run ros2_lidar_benchmark pointcloud_receiver.py"
echo

timeout 10 ros2 run ros2_lidar_benchmark pointcloud_receiver.py \
    --ros-args \
    -p input_topic:=/vlp16/velodyne_points \
    -p output_topic:=/test/output \
    -p use_best_effort_qos:=true

echo
echo "2. Testing with DEFAULT QoS:"
echo "Running: ros2 run ros2_lidar_benchmark pointcloud_receiver.py with default QoS"
echo

timeout 10 ros2 run ros2_lidar_benchmark pointcloud_receiver.py \
    --ros-args \
    -p input_topic:=/vlp16/velodyne_points \
    -p output_topic:=/test/output \
    -p use_best_effort_qos:=false

echo
echo "Test complete. Check the output above for any received messages."