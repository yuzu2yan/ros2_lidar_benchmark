#!/bin/bash

echo "Verifying Graph Output Configuration"
echo "===================================="

# Check if visualization data exists
echo -e "\n1. Checking for visualization data:"
if [ -f "/tmp/lidar_benchmark/visualization_data.json" ]; then
    echo "   ✓ Found: /tmp/lidar_benchmark/visualization_data.json"
    SIZE=$(du -h "/tmp/lidar_benchmark/visualization_data.json" | cut -f1)
    echo "   Size: $SIZE"
else
    echo "   ✗ Not found: /tmp/lidar_benchmark/visualization_data.json"
fi

# Check graph output directory
echo -e "\n2. Checking graph output directory:"
GRAPH_DIR="/tmp/lidar_benchmark_graphs"
if [ -d "$GRAPH_DIR" ]; then
    echo "   ✓ Directory exists: $GRAPH_DIR"
    COUNT=$(find "$GRAPH_DIR" -name "*.png" | wc -l)
    echo "   PNG files found: $COUNT"
    
    # List recent benchmark folders
    echo -e "\n   Recent benchmark folders:"
    ls -ltd "$GRAPH_DIR"/benchmark_* 2>/dev/null | head -5
else
    echo "   ✗ Directory not found: $GRAPH_DIR"
fi

# Check Excel report
echo -e "\n3. Checking Excel report:"
if [ -f "/tmp/lidar_benchmark_report.xlsx" ]; then
    echo "   ✓ Found: /tmp/lidar_benchmark_report.xlsx"
    SIZE=$(du -h "/tmp/lidar_benchmark_report.xlsx" | cut -f1)
    echo "   Size: $SIZE"
else
    echo "   ✗ Not found: /tmp/lidar_benchmark_report.xlsx"
fi

# Check config
echo -e "\n4. Configuration summary:"
CONFIG_FILE="/Users/yuzu/ros2_lidar_benchmark/ros2_lidar_benchmark/config/benchmark_config.yaml"
if [ -f "$CONFIG_FILE" ]; then
    echo "   Config file: $CONFIG_FILE"
    grep -E "graph_output_dir|report_file|enable_visualization" "$CONFIG_FILE" | sed 's/^/   /'
fi

echo -e "\n5. Expected outputs after benchmark:"
echo "   - Excel report: /tmp/lidar_benchmark_report.xlsx"
echo "   - Graphs folder: /tmp/lidar_benchmark_graphs/benchmark_YYYYMMDD_HHMMSS/"
echo "   - Visualization data: /tmp/lidar_benchmark/visualization_data.json"