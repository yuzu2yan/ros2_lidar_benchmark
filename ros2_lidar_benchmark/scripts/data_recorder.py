#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Empty
import json
import os
from collections import deque
import time
import threading


class DataRecorder(Node):
    """Records benchmark data for later graph generation"""
    
    def __init__(self):
        super().__init__('data_recorder')
        
        self.declare_parameter('output_dir', '/tmp/lidar_benchmark')
        self.declare_parameter('max_points', 3600)  # 1 hour at 1Hz
        
        self.output_dir = self.get_parameter('output_dir').value
        self.max_points = self.get_parameter('max_points').value
        
        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Data storage
        self.data = {
            'timestamps': [],
            'hz': [],
            'jitter': [],
            'bandwidth': [],
            'throughput': [],
            'cpu': [],
            'memory': [],
            'temperature': []
        }
        
        self.start_time = time.time()
        self.data_lock = threading.Lock()
        
        # Subscribers
        self.metrics_sub = self.create_subscription(
            Float64MultiArray,
            '/benchmark/metrics',
            self.metrics_callback,
            10
        )
        
        self.system_sub = self.create_subscription(
            Float64MultiArray,
            '/benchmark/system_resources',
            self.system_callback,
            10
        )
        
        # Shutdown subscriber
        self.shutdown_sub = self.create_subscription(
            Empty,
            '/benchmark/shutdown',
            self.shutdown_callback,
            10
        )
        
        # Periodic save timer
        self.save_timer = self.create_timer(10.0, self.save_data)
        
        self.get_logger().info(f'Data Recorder started. Saving to: {self.output_dir}')
        
    def metrics_callback(self, msg):
        with self.data_lock:
            current_time = time.time() - self.start_time
            self.data['timestamps'].append(current_time)
            
            if len(msg.data) >= 9:
                self.data['hz'].append(msg.data[0])
                self.data['jitter'].append(msg.data[1])
                self.data['bandwidth'].append(msg.data[2])
                self.data['throughput'].append(msg.data[8])  # kpoints_per_second
                
                # Trim if too many points
                if len(self.data['timestamps']) > self.max_points:
                    for key in ['timestamps', 'hz', 'jitter', 'bandwidth', 'throughput']:
                        if key in self.data and len(self.data[key]) > 0:
                            self.data[key].pop(0)
    
    def system_callback(self, msg):
        with self.data_lock:
            if len(msg.data) >= 7:
                # Align with metrics timestamps
                if len(self.data['timestamps']) > len(self.data['cpu']):
                    self.data['cpu'].append(msg.data[0])
                    self.data['memory'].append(msg.data[1])
                    self.data['temperature'].append(msg.data[6])
    
    def save_data(self):
        """Save data to JSON file"""
        with self.data_lock:
            if len(self.data['timestamps']) > 0:
                data_file = os.path.join(self.output_dir, 'visualization_data.json')
                try:
                    with open(data_file, 'w') as f:
                        json.dump(self.data, f, indent=2)
                    self.get_logger().info(f'Saved {len(self.data["timestamps"])} data points to {data_file}')
                except Exception as e:
                    self.get_logger().error(f'Failed to save data: {e}')
    
    def shutdown_callback(self, msg):
        """Handle shutdown signal"""
        self.get_logger().info('Received shutdown signal, saving final data...')
        self.save_data()
        self.get_logger().info('Data recording complete')
        # Set flag to exit main loop
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    
    node = DataRecorder()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.get_logger().info('Shutting down data_recorder...')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()