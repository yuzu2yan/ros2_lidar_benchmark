#!/usr/bin/env python3

import os
import subprocess
import time
import sys

def run_command(cmd, capture_output=False):
    """Run a command and return the result"""
    if capture_output:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        return result.stdout, result.stderr, result.returncode
    else:
        return subprocess.run(cmd, shell=True)

def check_rmw_implementation():
    """Check and set the best RMW implementation"""
    print("=" * 60)
    print("Checking RMW Implementation")
    print("=" * 60)
    
    # Get current RMW
    current_rmw = os.environ.get('RMW_IMPLEMENTATION', 'Not set')
    print(f"Current RMW_IMPLEMENTATION: {current_rmw}")
    
    # Check available RMW implementations
    stdout, _, _ = run_command("ros2 pkg list | grep rmw_", capture_output=True)
    print("\nAvailable RMW implementations:")
    print(stdout)
    
    # Test with CycloneDDS
    print("\nTesting with CycloneDDS...")
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
    
    # Check if data is visible
    stdout, stderr, returncode = run_command(
        "timeout 3 ros2 topic echo /vlp16/points_filtered --once", 
        capture_output=True
    )
    
    if returncode == 0 and len(stdout) > 10:
        print("✓ CycloneDDS works! Data received successfully.")
        return 'rmw_cyclonedds_cpp'
    else:
        print("✗ CycloneDDS failed to receive data")
        
        # Try FastDDS
        print("\nTesting with FastDDS...")
        os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
        
        stdout, stderr, returncode = run_command(
            "timeout 3 ros2 topic echo /vlp16/points_filtered --once", 
            capture_output=True
        )
        
        if returncode == 0 and len(stdout) > 10:
            print("✓ FastDDS works! Data received successfully.")
            return 'rmw_fastrtps_cpp'
        else:
            print("✗ FastDDS failed to receive data")
    
    return None

def check_topic_info():
    """Get detailed topic information"""
    print("\n" + "=" * 60)
    print("Topic Information")
    print("=" * 60)
    
    # List topics
    stdout, _, _ = run_command("ros2 topic list | grep -E '(vlp16|benchmark)'", capture_output=True)
    print("Relevant topics:")
    print(stdout)
    
    # Get detailed info
    stdout, _, _ = run_command("ros2 topic info -v /vlp16/points_filtered", capture_output=True)
    print("\nDetailed topic info:")
    print(stdout)

def create_fixed_launch_script(rmw_impl):
    """Create a launch script with the correct RMW implementation"""
    script_content = f'''#!/bin/bash
# Launch script with fixed RMW implementation

export RMW_IMPLEMENTATION={rmw_impl}

echo "Starting ROS 2 LiDAR Benchmark"
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo

# Launch the benchmark
ros2 launch ros2_lidar_benchmark benchmark.launch.py "$@"
'''
    
    script_path = '/Users/yuzu/ros2_lidar_benchmark/launch_benchmark_fixed.sh'
    with open(script_path, 'w') as f:
        f.write(script_content)
    
    os.chmod(script_path, 0o755)
    print(f"\nCreated fixed launch script: {script_path}")
    return script_path

def create_qos_override_config():
    """Create QoS override configuration"""
    config_content = '''# QoS override configuration for sensor data
/pointcloud_receiver:
  ros__parameters:
    qos_overrides:
      /vlp16/points_filtered:
        subscription:
          reliability: best_effort
          durability: volatile
          depth: 1
          
/metrics_collector:
  ros__parameters:
    qos_overrides:
      /benchmark/points:
        subscription:
          reliability: best_effort
          durability: volatile
          depth: 10
'''
    
    config_path = '/Users/yuzu/ros2_lidar_benchmark/qos_override.yaml'
    with open(config_path, 'w') as f:
        f.write(config_content)
    
    print(f"Created QoS override config: {config_path}")
    return config_path

def test_direct_subscription():
    """Test direct subscription with the correct settings"""
    print("\n" + "=" * 60)
    print("Testing Direct Subscription")
    print("=" * 60)
    
    test_script = '''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSPresetProfiles
import sys

class TestSubscriber(Node):
    def __init__(self):
        super().__init__('test_subscriber')
        self.received = False
        
        # Use sensor data QoS profile
        qos = QoSPresetProfiles.SENSOR_DATA.value
        
        self.subscription = self.create_subscription(
            PointCloud2,
            '/vlp16/points_filtered',
            self.callback,
            qos
        )
        
        self.timer = self.create_timer(3.0, self.check_status)
        self.get_logger().info('Waiting for data...')
    
    def callback(self, msg):
        if not self.received:
            self.received = True
            self.get_logger().info(f'SUCCESS! Received data: {msg.width}x{msg.height} points')
            self.get_logger().info(f'Frame ID: {msg.header.frame_id}')
            sys.exit(0)
    
    def check_status(self):
        if not self.received:
            self.get_logger().error('No data received after 3 seconds')
            sys.exit(1)

def main():
    rclpy.init()
    node = TestSubscriber()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
    
    # Save and run test script
    test_path = '/tmp/test_direct_sub.py'
    with open(test_path, 'w') as f:
        f.write(test_script)
    
    print("Running direct subscription test...")
    result = run_command(f"python3 {test_path}")
    
    return result.returncode == 0 if hasattr(result, 'returncode') else False

def main():
    print("ROS 2 LiDAR Benchmark - Data Reception Fix")
    print("=" * 60)
    
    # Step 1: Check topic information
    check_topic_info()
    
    # Step 2: Test RMW implementations
    working_rmw = check_rmw_implementation()
    
    if working_rmw:
        print(f"\n✓ Found working RMW implementation: {working_rmw}")
        
        # Create fixed launch script
        launch_script = create_fixed_launch_script(working_rmw)
        
        # Create QoS override config
        qos_config = create_qos_override_config()
        
        # Test direct subscription
        if test_direct_subscription():
            print("\n✓ Direct subscription test passed!")
        
        print("\n" + "=" * 60)
        print("SOLUTION SUMMARY")
        print("=" * 60)
        print(f"1. Set RMW_IMPLEMENTATION={working_rmw}")
        print(f"2. Use the fixed launch script: {launch_script}")
        print(f"3. Or set environment variable before launching:")
        print(f"   export RMW_IMPLEMENTATION={working_rmw}")
        print(f"   ros2 launch ros2_lidar_benchmark benchmark.launch.py")
        print("\nThe benchmark should now receive data correctly!")
        
    else:
        print("\n✗ Could not find a working RMW implementation")
        print("Please check:")
        print("1. Is the r2r_multi_lidar_filter running?")
        print("2. Is tcpreplay sending data?")
        print("3. Network/DDS configuration")

if __name__ == '__main__':
    main()