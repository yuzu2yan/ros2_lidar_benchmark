#!/bin/bash

echo "=== Example: Starting R2R Multi LiDAR Filter ==="
echo

# Check if user provided pcap file
if [ $# -eq 0 ]; then
    echo "Usage: $0 <path_to_pcap_file>"
    echo
    echo "Example:"
    echo "  $0 /path/to/your/lidar_data.pcap"
    echo
    echo "The filter launch command would be:"
    echo "  ros2 launch r2r_multi_lidar_filter r2r_multi_lidar_filter_vlp_launch.py \\"
    echo "    model:=VLP16 \\"
    echo "    pcap:=/path/to/your/lidar_data.pcap \\"
    echo "    read_once:=false"
    echo
    echo "Parameters:"
    echo "  model: VLP16 or 32C"
    echo "  pcap: path to pcap file"
    echo "  read_once: false for continuous playback, true for single playback"
    exit 1
fi

PCAP_FILE=$1

echo "Starting filter with pcap file: $PCAP_FILE"
echo

# Check if pcap file exists
if [ ! -f "$PCAP_FILE" ]; then
    echo "Error: PCAP file not found: $PCAP_FILE"
    exit 1
fi

echo "Launching filter..."
echo "Command:"
echo "ros2 launch r2r_multi_lidar_filter r2r_multi_lidar_filter_vlp_launch.py \\"
echo "  model:=VLP16 \\"
echo "  pcap:=$PCAP_FILE \\"
echo "  read_once:=false"
echo
echo "After the filter starts, run the benchmark in another terminal:"
echo "  ros2 launch ros2_lidar_benchmark benchmark.launch.py"