#!/usr/bin/env python3

import subprocess
import sys

def run_command(cmd):
    """Run command and return output"""
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    return result.stdout, result.stderr

def main():
    topic = '/vlp16/points_filtered'
    
    print("=" * 60)
    print("QoS Information Check")
    print("=" * 60)
    
    # Check if topic exists
    print(f"\n1. Checking if topic {topic} exists...")
    stdout, _ = run_command("ros2 topic list")
    if topic in stdout:
        print(f"✓ Topic {topic} exists")
    else:
        print(f"✗ Topic {topic} not found")
        print("Available topics:")
        print(stdout)
        return
    
    # Get topic info with QoS
    print(f"\n2. Getting QoS info for {topic}...")
    stdout, stderr = run_command(f"ros2 topic info -v {topic}")
    print(stdout)
    
    # Try to echo one message
    print(f"\n3. Trying to echo one message...")
    stdout, stderr = run_command(f"timeout 2 ros2 topic echo {topic} --once")
    if stdout:
        print("✓ Successfully received data with ros2 topic echo")
        # Extract first few lines
        lines = stdout.split('\n')[:10]
        print("First few lines of data:")
        for line in lines:
            print(f"  {line}")
    else:
        print("✗ No data received with ros2 topic echo")
    
    # Check publishing rate
    print(f"\n4. Checking publishing rate...")
    stdout, stderr = run_command(f"timeout 3 ros2 topic hz {topic}")
    if stdout:
        print(stdout)
    
    # Get node info
    print("\n5. Checking nodes and topics...")
    stdout, _ = run_command("ros2 node list")
    print("Active nodes:")
    for node in stdout.strip().split('\n'):
        if node:
            print(f"  {node}")
    
    # Find publisher
    print(f"\n6. Finding publisher of {topic}...")
    stdout, _ = run_command(f"ros2 topic info {topic}")
    print(stdout)

if __name__ == '__main__':
    main()