#!/bin/bash

echo "======================================"
echo "ROS 2 LiDAR Benchmark Results Summary"
echo "======================================"

# Function to format file size
format_size() {
    if [ -f "$1" ]; then
        ls -lh "$1" | awk '{print $5}'
    else
        echo "N/A"
    fi
}

# Check Excel report
EXCEL_FILE="/tmp/lidar_benchmark_report.xlsx"
if [ -f "$EXCEL_FILE" ]; then
    echo -e "\n✓ Excel Report Generated:"
    echo "  Location: $EXCEL_FILE"
    echo "  Size: $(format_size "$EXCEL_FILE")"
    echo "  Modified: $(date -r "$EXCEL_FILE" "+%Y-%m-%d %H:%M:%S")"
else
    echo -e "\n✗ Excel Report NOT FOUND"
fi

# Check visualization data
VIZ_DATA="/tmp/lidar_benchmark/visualization_data.json"
if [ -f "$VIZ_DATA" ]; then
    echo -e "\n✓ Visualization Data Recorded:"
    echo "  Location: $VIZ_DATA"
    echo "  Size: $(format_size "$VIZ_DATA")"
    
    # Count data points
    if command -v jq > /dev/null 2>&1; then
        POINTS=$(jq '.timestamps | length' "$VIZ_DATA" 2>/dev/null || echo "Unknown")
        echo "  Data points: $POINTS"
    fi
else
    echo -e "\n✗ Visualization Data NOT FOUND"
fi

# Check graphs
GRAPH_DIR="/tmp/lidar_benchmark_graphs"
if [ -d "$GRAPH_DIR" ]; then
    # Find the most recent benchmark folder
    LATEST=$(find "$GRAPH_DIR" -type d -name "benchmark_*" 2>/dev/null | sort | tail -1)
    
    if [ -n "$LATEST" ]; then
        echo -e "\n✓ Graphs Generated:"
        echo "  Directory: $LATEST"
        
        # Count PNG files
        PNG_COUNT=$(find "$LATEST" -name "*.png" 2>/dev/null | wc -l)
        echo "  Graph files: $PNG_COUNT"
        
        # List graph files
        echo "  Graphs:"
        find "$LATEST" -name "*.png" -exec basename {} \; | sort | sed 's/^/    - /'
        
        # Check metadata
        if [ -f "$LATEST/metadata.json" ]; then
            echo "  ✓ Metadata file present"
        fi
    else
        echo -e "\n⚠ Graph directory exists but no benchmark folders found"
    fi
else
    echo -e "\n✗ Graph Directory NOT FOUND"
fi

# Performance summary from JSON if available
JSON_FILE="/tmp/lidar_benchmark_report.json"
if [ -f "$JSON_FILE" ] && command -v jq > /dev/null 2>&1; then
    echo -e "\n📊 Performance Summary:"
    
    # Extract key metrics
    AVG_HZ=$(jq -r '.lidar_metrics.average_hz // "N/A"' "$JSON_FILE" 2>/dev/null)
    AVG_JITTER=$(jq -r '.lidar_metrics.average_jitter_ms // "N/A"' "$JSON_FILE" 2>/dev/null)
    PERF_RATING=$(jq -r '.analysis.performance_rating // "N/A"' "$JSON_FILE" 2>/dev/null)
    STAB_RATING=$(jq -r '.analysis.stability_rating // "N/A"' "$JSON_FILE" 2>/dev/null)
    
    echo "  Average Hz: $AVG_HZ"
    echo "  Average Jitter: $AVG_JITTER ms"
    echo "  Performance Rating: $PERF_RATING"
    echo "  Stability Rating: $STAB_RATING"
fi

echo -e "\n======================================"
echo "To view graphs, open: $GRAPH_DIR"
echo "To view Excel report, open: $EXCEL_FILE"
echo "======================================" 