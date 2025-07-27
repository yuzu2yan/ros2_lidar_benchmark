#!/bin/bash

echo "Checking Benchmark Output Files"
echo "==============================="

echo -e "\n1. Visualization Data:"
for path in "/tmp/lidar_benchmark/visualization_data.json" \
           "/tmp/visualization_data.json" \
           "./visualization_data.json"; do
    if [ -f "$path" ]; then
        echo "   ✓ Found: $path"
        SIZE=$(ls -lh "$path" | awk '{print $5}')
        LINES=$(wc -l < "$path")
        echo "     Size: $SIZE, Lines: $LINES"
        
        # Check if it has actual data
        if grep -q '"timestamps"' "$path" 2>/dev/null; then
            POINTS=$(grep -o '"timestamps": \[' "$path" | head -1 | wc -l)
            if [ $POINTS -gt 0 ]; then
                echo "     Contains timestamp data"
            fi
        fi
    else
        echo "   ✗ Not found: $path"
    fi
done

echo -e "\n2. Excel Report:"
for path in "/tmp/lidar_benchmark_report.xlsx" \
           "./lidar_benchmark_report.xlsx"; do
    if [ -f "$path" ]; then
        echo "   ✓ Found: $path"
        SIZE=$(ls -lh "$path" | awk '{print $5}')
        echo "     Size: $SIZE"
    fi
done

echo -e "\n3. JSON Report:"
for path in "/tmp/lidar_benchmark_report.json" \
           "./lidar_benchmark_report.json"; do
    if [ -f "$path" ]; then
        echo "   ✓ Found: $path"
        SIZE=$(ls -lh "$path" | awk '{print $5}')
        echo "     Size: $SIZE"
    fi
done

echo -e "\n4. Graph Directories:"
for dir in "/tmp/lidar_benchmark_graphs" \
          "./lidar_benchmark_graphs"; do
    if [ -d "$dir" ]; then
        echo "   ✓ Found directory: $dir"
        COUNT=$(find "$dir" -type d -name "benchmark_*" | wc -l)
        echo "     Benchmark folders: $COUNT"
        
        # List recent folders
        if [ $COUNT -gt 0 ]; then
            echo "     Recent folders:"
            find "$dir" -type d -name "benchmark_*" -exec basename {} \; | sort | tail -3 | sed 's/^/       - /'
            
            # Count PNG files in most recent
            LATEST=$(find "$dir" -type d -name "benchmark_*" | sort | tail -1)
            if [ -n "$LATEST" ]; then
                PNG_COUNT=$(find "$LATEST" -name "*.png" | wc -l)
                echo "     PNG files in latest: $PNG_COUNT"
            fi
        fi
    else
        echo "   ✗ Not found: $dir"
    fi
done

echo -e "\n5. Process Check:"
# Check if any benchmark nodes are running
if pgrep -f "ros2.*benchmark" > /dev/null; then
    echo "   ⚠ Benchmark processes are still running"
else
    echo "   ✓ No benchmark processes running"
fi