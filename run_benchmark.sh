#!/bin/bash

echo "ROS 2 LiDAR Benchmark - Quick Start"
echo "==================================="

# Check if data is available
echo "Checking for LiDAR data..."
if timeout 2 ros2 topic hz /vlp16/points_filtered > /dev/null 2>&1; then
    echo "✓ LiDAR data detected on /vlp16/points_filtered"
else
    echo "✗ No LiDAR data detected"
    echo "Please ensure:"
    echo "1. tcpreplay is running"
    echo "2. r2r_multi_lidar_filter is running"
    exit 1
fi

# Test RMW implementations
echo ""
echo "Testing RMW implementations..."

# Try CycloneDDS first
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if timeout 2 ros2 topic echo /vlp16/points_filtered --once > /dev/null 2>&1; then
    echo "✓ Using CycloneDDS (recommended for sensor data)"
else
    # Try FastDDS
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    if timeout 2 ros2 topic echo /vlp16/points_filtered --once > /dev/null 2>&1; then
        echo "✓ Using FastDDS"
    else
        echo "✗ No working RMW implementation found"
        echo "Using default RMW implementation"
        unset RMW_IMPLEMENTATION
    fi
fi

echo ""
echo "Starting benchmark with RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo ""

# Parse command line arguments
DURATION=${1:-60}
SHOW_VIZ=${2:-true}

echo "Configuration:"
echo "- Analysis duration: $DURATION seconds"
echo "- Visualization: $SHOW_VIZ"
echo ""

# Launch the benchmark
ros2 launch ros2_lidar_benchmark benchmark.launch.py \
    analysis_duration:=$DURATION \
    show_viz:=$SHOW_VIZ \
    input_topic:=/vlp16/points_filtered \
    use_best_effort_qos:=true

echo ""
echo "Benchmark completed!"
echo "Results saved to:"
echo "- Excel report: /tmp/lidar_benchmark_report.xlsx"
echo "- Plots: /tmp/lidar_benchmark/"