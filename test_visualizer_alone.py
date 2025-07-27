#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
import time


class TestDataPublisher(Node):
    def __init__(self):
        super().__init__('test_data_publisher')
        
        # Publishers for test data
        self.metrics_pub = self.create_publisher(
            Float64MultiArray,
            '/benchmark/metrics',
            10
        )
        
        self.system_pub = self.create_publisher(
            Float64MultiArray,
            '/benchmark/system_resources',
            10
        )
        
        # Timer for publishing test data
        self.timer = self.create_timer(0.5, self.publish_test_data)
        self.counter = 0
        self.start_time = time.time()
        
        self.get_logger().info('Publishing test data for visualizer...')
        self.get_logger().info('Start the visualizer in another terminal:')
        self.get_logger().info('  ros2 run ros2_lidar_benchmark visualizer.py')
        
    def publish_test_data(self):
        current_time = time.time() - self.start_time
        
        # Generate synthetic metrics data
        metrics_msg = Float64MultiArray()
        metrics_msg.data = [
            10.0 + 2.0 * math.sin(current_time * 0.5),      # Hz (varying 8-12)
            5.0 + 3.0 * math.sin(current_time * 0.3),       # Jitter (varying 2-8 ms)
            50.0 + 10.0 * math.sin(current_time * 0.4),     # Bandwidth (varying 40-60 Mbps)
            100.0,                                            # Avg message size KB
            self.counter,                                     # Total messages
            self.counter * 0.1,                              # Total MB
            10.0,                                            # Messages per second
            5.0,                                             # MB per second
            300.0 + 50.0 * math.sin(current_time * 0.6)     # K points per second
        ]
        self.metrics_pub.publish(metrics_msg)
        
        # Generate synthetic system data
        system_msg = Float64MultiArray()
        system_msg.data = [
            30.0 + 20.0 * math.sin(current_time * 0.2),     # CPU percent
            40.0 + 10.0 * math.sin(current_time * 0.15),    # Memory percent
            35.0,                                            # CPU avg 1min
            42.0,                                            # Memory avg 1min
            15.0,                                            # Process CPU
            256.0,                                           # Process memory MB
            45.0 + 5.0 * math.sin(current_time * 0.1)       # CPU temp
        ]
        self.system_pub.publish(system_msg)
        
        self.counter += 1
        
        if self.counter % 10 == 0:
            self.get_logger().info(f'Published {self.counter} test messages')


def main(args=None):
    rclpy.init(args=args)
    node = TestDataPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()