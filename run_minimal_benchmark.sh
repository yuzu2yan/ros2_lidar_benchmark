#!/bin/bash

echo "=== Running Minimal Benchmark Setup ==="
echo "Starting each component separately for debugging"
echo

# Kill any existing benchmark nodes
echo "Cleaning up existing nodes..."
pkill -f pointcloud_receiver
pkill -f metrics_collector
pkill -f system_monitor
pkill -f visualizer
sleep 2

# Start each node in sequence
echo "1. Starting pointcloud_receiver..."
ros2 run ros2_lidar_benchmark pointcloud_receiver.py \
    --ros-args \
    -p input_topic:=/vlp16/points_filtered \
    -p output_topic:=/benchmark/points \
    -p use_best_effort_qos:=true &
RECEIVER_PID=$!
sleep 2

echo "2. Starting metrics_collector..."
ros2 run ros2_lidar_benchmark metrics_collector.py \
    --ros-args \
    -p topic_to_monitor:=/benchmark/points \
    -p use_best_effort_qos:=true &
METRICS_PID=$!
sleep 2

echo "3. Starting system_monitor..."
ros2 run ros2_lidar_benchmark system_monitor.py &
SYSTEM_PID=$!
sleep 2

echo "4. Starting visualizer..."
ros2 run ros2_lidar_benchmark visualizer.py &
VISUALIZER_PID=$!

echo
echo "All nodes started. Checking status..."
sleep 3

echo
echo "Active ROS nodes:"
ros2 node list | grep -E "(pointcloud_receiver|metrics_collector|system_monitor|benchmark_visualizer)"

echo
echo "Active topics:"
ros2 topic list | grep benchmark

echo
echo "Press Ctrl+C to stop all nodes..."

# Wait for user interrupt
trap 'echo "Stopping all nodes..."; kill $RECEIVER_PID $METRICS_PID $SYSTEM_PID $VISUALIZER_PID 2>/dev/null; exit' INT
wait