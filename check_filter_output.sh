#!/bin/bash

echo "=== Checking R2R Multi LiDAR Filter Output ==="
echo

# Check all topics related to vlp16
echo "1. Available topics from VLP16:"
ros2 topic list | grep vlp16

echo
echo "2. Checking raw velodyne points:"
echo "   Topic: /vlp16/velodyne_points"
timeout 3 ros2 topic hz /vlp16/velodyne_points

echo
echo "3. Checking filtered points:"
echo "   Topic: /vlp16/points_filtered"
timeout 3 ros2 topic hz /vlp16/points_filtered

echo
echo "4. Topic info for velodyne_points:"
ros2 topic info /vlp16/velodyne_points

echo
echo "5. Topic info for points_filtered:"
ros2 topic info /vlp16/points_filtered

echo
echo "6. Checking message type:"
ros2 topic type /vlp16/velodyne_points
ros2 topic type /vlp16/points_filtered

echo
echo "Which topic should the benchmark use?"
echo "- /vlp16/velodyne_points (raw data)"
echo "- /vlp16/points_filtered (filtered data)"