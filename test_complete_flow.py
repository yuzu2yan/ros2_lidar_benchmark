#!/usr/bin/env python3

import subprocess
import time
import sys

def run_command(cmd, description):
    """Run a command and return output"""
    print(f"\n{'='*60}")
    print(f"Running: {description}")
    print(f"Command: {cmd}")
    print('='*60)
    
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        if result.stdout:
            print("Output:")
            print(result.stdout)
        if result.stderr:
            print("Errors:")
            print(result.stderr)
        return result.returncode == 0
    except Exception as e:
        print(f"Error running command: {e}")
        return False

def main():
    print("ROS2 LiDAR Benchmark Flow Test")
    print("="*80)
    
    # Step 1: Check ROS2 is running
    if not run_command("ros2 daemon status", "Check ROS2 daemon"):
        print("Starting ROS2 daemon...")
        run_command("ros2 daemon start", "Start ROS2 daemon")
    
    # Step 2: List all nodes
    run_command("ros2 node list", "List all active nodes")
    
    # Step 3: List all topics
    run_command("ros2 topic list | grep -E '(points|velodyne)'", "List point cloud topics")
    
    # Step 4: Check specific topics
    topics_to_check = [
        "/vlp16/velodyne_points",
        "/vlp16/points_filtered",
        "/vlp32/velodyne_points", 
        "/vlp32/points_filtered"
    ]
    
    print("\n" + "="*80)
    print("Checking individual topics...")
    print("="*80)
    
    for topic in topics_to_check:
        print(f"\nChecking: {topic}")
        
        # Check if topic exists
        result = subprocess.run(f"ros2 topic list | grep -x {topic}", 
                              shell=True, capture_output=True, text=True)
        
        if result.returncode == 0:
            print(f"✓ Topic exists")
            
            # Get topic info
            run_command(f"ros2 topic info {topic} -v", f"Info for {topic}")
            
            # Try to get one message
            print(f"\nTrying to get one message from {topic}...")
            try:
                result = subprocess.run(
                    f"timeout 3 ros2 topic echo {topic} --once",
                    shell=True, capture_output=True, text=True
                )
                if result.returncode == 0 and result.stdout:
                    lines = result.stdout.strip().split('\n')
                    print(f"✓ Got message! First few lines:")
                    for line in lines[:10]:
                        print(f"  {line}")
                    if len(lines) > 10:
                        print(f"  ... ({len(lines)-10} more lines)")
                else:
                    print("✗ No message received within 3 seconds")
            except Exception as e:
                print(f"✗ Error getting message: {e}")
        else:
            print(f"✗ Topic does not exist")
    
    # Step 5: Check QoS compatibility
    print("\n" + "="*80)
    print("QoS Compatibility Check")
    print("="*80)
    
    for topic in ["/vlp16/points_filtered", "/vlp16/velodyne_points"]:
        result = subprocess.run(f"ros2 topic list | grep -x {topic}", 
                              shell=True, capture_output=True, text=True)
        if result.returncode == 0:
            print(f"\nQoS info for {topic}:")
            run_command(f"ros2 topic info {topic} --verbose", f"Verbose info for {topic}")

if __name__ == "__main__":
    main()