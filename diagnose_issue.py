#!/usr/bin/env python3

import subprocess
import time
import sys

def run_command(cmd, timeout=5):
    """Run a command and return output"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        return result.stdout, result.stderr, result.returncode
    except subprocess.TimeoutExpired:
        return "", "Timeout", -1

def check_topic_exists(topic):
    """Check if topic exists"""
    stdout, _, _ = run_command("ros2 topic list")
    return topic in stdout

def check_topic_data(topic):
    """Check if topic has data"""
    stdout, stderr, code = run_command(f"ros2 topic hz {topic}", timeout=3)
    if "average rate" in stdout:
        return True, stdout.strip()
    return False, "No data"

def main():
    print("=== ROS 2 LiDAR Benchmark Diagnostic Tool ===\n")
    
    # Check ROS 2
    print("1. Checking ROS 2 environment...")
    stdout, _, _ = run_command("ros2 topic list", timeout=2)
    if not stdout:
        print("❌ ROS 2 is not running or not sourced properly")
        sys.exit(1)
    print("✅ ROS 2 is running\n")
    
    # Check input topic
    input_topic = "/vlp16/velodyne_points"
    print(f"2. Checking input topic: {input_topic}")
    
    if check_topic_exists(input_topic):
        print(f"✅ Topic exists")
        has_data, info = check_topic_data(input_topic)
        if has_data:
            print(f"✅ Topic has data: {info}")
        else:
            print(f"❌ Topic exists but has no data")
            print("   Please check your pcap filter node")
    else:
        print(f"❌ Topic does not exist")
        print("   Available topics:")
        stdout, _, _ = run_command("ros2 topic list")
        for line in stdout.split('\n'):
            if line.strip():
                print(f"     - {line.strip()}")
    
    print("\n3. Testing pointcloud_receiver directly...")
    print("   Starting receiver for 5 seconds...")
    
    # Start receiver in background
    receiver_cmd = """ros2 run ros2_lidar_benchmark pointcloud_receiver.py \
        --ros-args \
        -p input_topic:=/vlp16/velodyne_points \
        -p output_topic:=/benchmark/points \
        -p use_best_effort_qos:=true"""
    
    proc = subprocess.Popen(receiver_cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    
    # Wait a bit
    time.sleep(3)
    
    # Check if receiver is still running
    if proc.poll() is None:
        print("✅ Receiver is running")
        
        # Check output topic
        if check_topic_exists("/benchmark/points"):
            print("✅ Output topic /benchmark/points exists")
            has_data, info = check_topic_data("/benchmark/points")
            if has_data:
                print(f"✅ Output topic has data: {info}")
            else:
                print("❌ Output topic exists but has no data")
        else:
            print("❌ Output topic /benchmark/points does not exist")
    else:
        print("❌ Receiver crashed")
        stdout, stderr = proc.communicate()
        print(f"   Error: {stderr}")
    
    # Clean up
    proc.terminate()
    time.sleep(1)
    
    print("\n4. Summary:")
    print("   - If input topic has no data: Check your pcap filter")
    print("   - If input has data but output doesn't: Check QoS settings")
    print("   - If receiver crashes: Check for Python errors")
    
    print("\n5. Recommended next steps:")
    print("   a) Run: ./test_nodes_step_by_step.sh")
    print("   b) Run: ros2 launch ros2_lidar_benchmark debug_benchmark.launch.py")


if __name__ == "__main__":
    main()