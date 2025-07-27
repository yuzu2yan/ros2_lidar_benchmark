#!/bin/bash

echo "=== Checking Input Data Source ==="
echo

# First, check if filter is running
echo "1. Checking if r2r_multi_lidar_filter is running:"
ps aux | grep -E "(r2r_multi_lidar_filter|velodyne)" | grep -v grep

echo
echo "2. Checking all available topics:"
ros2 topic list

echo
echo "3. Checking Velodyne-related topics:"
ros2 topic list | grep -E "(velodyne|vlp|points)"

echo
echo "4. Checking node list:"
ros2 node list

echo
echo "5. Testing raw data reception with echo:"
echo "Trying /vlp16/velodyne_points..."
timeout 3 ros2 topic echo /vlp16/velodyne_points --once

echo
echo "Trying /vlp16/points_filtered..."
timeout 3 ros2 topic echo /vlp16/points_filtered --once

echo
echo "6. Checking topic info for debugging:"
echo "Info for /vlp16/velodyne_points:"
ros2 topic info -v /vlp16/velodyne_points

echo
echo "Info for /vlp16/points_filtered:"
ros2 topic info -v /vlp16/points_filtered

echo
echo "=== DIAGNOSIS ==="
echo "If no velodyne nodes are running:"
echo "  You need to start the filter with:"
echo "  ros2 launch r2r_multi_lidar_filter r2r_multi_lidar_filter_vlp_launch.py pcap:=/path/to/your.pcap"
echo
echo "If nodes are running but no data:"
echo "  - Check if pcap file path is correct"
echo "  - Check if pcap playback has finished"
echo "  - Try with read_once:=false for continuous playback"