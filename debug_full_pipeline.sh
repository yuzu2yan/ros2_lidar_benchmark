#!/bin/bash

echo "=== Full Pipeline Debug ==="
echo "Checking each stage of the benchmark pipeline"
echo

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Function to check topic
check_topic() {
    local topic=$1
    local name=$2
    
    echo -n "$name ($topic): "
    
    # Check if topic exists
    if ros2 topic list | grep -q "^$topic$"; then
        # Check data rate
        hz=$(timeout 2 ros2 topic hz $topic 2>&1 | grep "average rate" | head -1)
        if [ -n "$hz" ]; then
            echo -e "${GREEN}OK - $hz${NC}"
            return 0
        else
            echo -e "${RED}EXISTS but NO DATA${NC}"
            return 1
        fi
    else
        echo -e "${RED}NOT FOUND${NC}"
        return 1
    fi
}

echo "1. Input Data Flow:"
echo "==================="
check_topic "/vlp16/velodyne_points" "Raw Velodyne Data"
check_topic "/vlp16/points_filtered" "Filtered Data"

echo
echo "2. Benchmark Pipeline:"
echo "====================="
check_topic "/benchmark/points" "Benchmark Output"

echo
echo "3. Metrics & Monitoring:"
echo "======================="
check_topic "/benchmark/metrics" "Metrics Data"
check_topic "/benchmark/system_resources" "System Resources"

echo
echo "4. Node Status:"
echo "==============="
nodes=(
    "pointcloud_receiver"
    "metrics_collector"
    "system_monitor"
    "benchmark_visualizer"
    "benchmark_analyzer"
)

for node in "${nodes[@]}"; do
    if ros2 node list | grep -q $node; then
        echo -e "$node: ${GREEN}RUNNING${NC}"
    else
        echo -e "$node: ${RED}NOT RUNNING${NC}"
    fi
done

echo
echo "5. Testing Individual Components:"
echo "================================="

# Test metrics topic content
echo "Checking metrics data format:"
timeout 2 ros2 topic echo /benchmark/metrics --once 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}Metrics data available${NC}"
else
    echo -e "${RED}No metrics data${NC}"
fi

echo
echo "6. Recommendations:"
echo "=================="
if ! check_topic "/benchmark/points" "" > /dev/null 2>&1; then
    echo "- pointcloud_receiver is not forwarding data"
    echo "  Check input topic in config: /vlp16/points_filtered"
fi

if ! check_topic "/benchmark/metrics" "" > /dev/null 2>&1; then
    echo "- metrics_collector is not receiving data from /benchmark/points"
fi

echo
echo "To fix visualization issues:"
echo "1. Ensure all nodes are running"
echo "2. Check that /benchmark/metrics has data"
echo "3. Check that /benchmark/system_resources has data"
echo "4. Try running visualizer standalone:"
echo "   ros2 run ros2_lidar_benchmark visualizer.py"