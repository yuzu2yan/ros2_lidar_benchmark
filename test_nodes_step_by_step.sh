#!/bin/bash

echo "=== Step-by-Step Node Testing ==="
echo "This script will test each node individually"
echo

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "Step 1: Check if input topic has data"
echo "--------------------------------------"
echo "Checking /vlp16/velodyne_points..."
timeout 3 ros2 topic hz /vlp16/velodyne_points
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Input topic has data${NC}"
else
    echo -e "${RED}✗ No data on input topic!${NC}"
    echo "Please ensure your pcap filter is running and publishing to /vlp16/velodyne_points"
    exit 1
fi

echo
echo "Step 2: Start pointcloud_receiver"
echo "---------------------------------"
echo "Starting receiver in background..."
ros2 run ros2_lidar_benchmark pointcloud_receiver.py \
    --ros-args \
    -p input_topic:=/vlp16/velodyne_points \
    -p output_topic:=/benchmark/points \
    -p use_best_effort_qos:=true &
RECEIVER_PID=$!

sleep 3

echo "Checking if receiver is running..."
if ps -p $RECEIVER_PID > /dev/null; then
    echo -e "${GREEN}✓ Receiver is running (PID: $RECEIVER_PID)${NC}"
else
    echo -e "${RED}✗ Receiver failed to start${NC}"
    exit 1
fi

echo
echo "Step 3: Check benchmark/points output"
echo "-------------------------------------"
echo "Checking /benchmark/points..."
timeout 3 ros2 topic hz /benchmark/points
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Benchmark topic has data${NC}"
else
    echo -e "${RED}✗ No data on benchmark topic!${NC}"
    echo "Receiver is not forwarding data properly"
fi

echo
echo "Step 4: Start metrics_collector"
echo "-------------------------------"
echo "Starting metrics collector in background..."
ros2 run ros2_lidar_benchmark metrics_collector.py \
    --ros-args \
    -p topic_to_monitor:=/benchmark/points &
METRICS_PID=$!

sleep 3

echo "Checking if metrics collector is running..."
if ps -p $METRICS_PID > /dev/null; then
    echo -e "${GREEN}✓ Metrics collector is running (PID: $METRICS_PID)${NC}"
else
    echo -e "${RED}✗ Metrics collector failed to start${NC}"
fi

echo
echo "Step 5: Check metrics output"
echo "----------------------------"
echo "Checking /benchmark/metrics..."
timeout 3 ros2 topic echo /benchmark/metrics --once

echo
echo "Step 6: List all topics"
echo "-----------------------"
ros2 topic list | grep benchmark

echo
echo -e "${YELLOW}Press Enter to stop all nodes...${NC}"
read

# Clean up
echo "Stopping nodes..."
kill $RECEIVER_PID 2>/dev/null
kill $METRICS_PID 2>/dev/null

echo "Done!"