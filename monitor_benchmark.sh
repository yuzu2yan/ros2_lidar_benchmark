#!/bin/bash

echo "=== ROS 2 LiDAR Benchmark Monitor ==="
echo
echo "This script monitors the data flow through the benchmark pipeline"
echo

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to check topic
check_topic() {
    local topic=$1
    local name=$2
    
    echo -n "Checking $name ($topic): "
    
    # Check if topic exists
    if ros2 topic list | grep -q "^$topic$"; then
        echo -e "${GREEN}EXISTS${NC}"
        
        # Get topic info
        info=$(ros2 topic info $topic 2>&1)
        if echo "$info" | grep -q "Publisher count: 0"; then
            echo -e "  ${RED}No publishers!${NC}"
        else
            pub_count=$(echo "$info" | grep "Publisher count:" | awk '{print $3}')
            sub_count=$(echo "$info" | grep "Subscriber count:" | awk '{print $3}')
            echo -e "  Publishers: ${GREEN}$pub_count${NC}, Subscribers: ${GREEN}$sub_count${NC}"
        fi
        
        # Try to get Hz
        echo -n "  Checking data rate... "
        hz=$(timeout 3 ros2 topic hz $topic 2>&1 | grep "average rate" | head -1)
        if [ -n "$hz" ]; then
            echo -e "${GREEN}$hz${NC}"
        else
            echo -e "${RED}No data${NC}"
        fi
    else
        echo -e "${RED}NOT FOUND${NC}"
    fi
    echo
}

# Main monitoring
while true; do
    clear
    echo "=== ROS 2 LiDAR Benchmark Monitor ==="
    echo "Time: $(date)"
    echo
    
    # Check input topic
    check_topic "/vlp16/velodyne_points" "Input Topic"
    
    # Check benchmark output topic  
    check_topic "/benchmark/points" "Benchmark Topic"
    
    # Check metrics topic
    check_topic "/benchmark/metrics" "Metrics Topic"
    
    # Check if benchmark nodes are running
    echo "Benchmark Nodes:"
    if pgrep -f "pointcloud_receiver.py" > /dev/null; then
        echo -e "  pointcloud_receiver: ${GREEN}RUNNING${NC}"
    else
        echo -e "  pointcloud_receiver: ${RED}NOT RUNNING${NC}"
    fi
    
    if pgrep -f "metrics_collector.py" > /dev/null; then
        echo -e "  metrics_collector: ${GREEN}RUNNING${NC}"
    else
        echo -e "  metrics_collector: ${RED}NOT RUNNING${NC}"
    fi
    
    echo
    echo "Press Ctrl+C to exit, refreshing in 5 seconds..."
    sleep 5
done