#!/bin/bash

echo "=== ROS 2 LiDAR Benchmark Topic Test ==="
echo

echo "1. Checking available topics:"
ros2 topic list

echo
echo "2. Checking /vlp16/velodyne_points topic:"
ros2 topic info /vlp16/velodyne_points

echo
echo "3. Checking topic data rate:"
timeout 5 ros2 topic hz /vlp16/velodyne_points

echo
echo "4. Checking message type:"
ros2 topic echo /vlp16/velodyne_points --once

echo
echo "5. Testing benchmark receiver with direct topic:"
timeout 10 ros2 run ros2_lidar_benchmark pointcloud_receiver.py --ros-args -p input_topic:=/vlp16/velodyne_points -p output_topic:=/test/output