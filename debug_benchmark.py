#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import time


class DebugNode(Node):
    def __init__(self):
        super().__init__('debug_benchmark')
        
        # Subscribe to the input topic
        self.input_sub = self.create_subscription(
            PointCloud2,
            '/vlp16/velodyne_points',
            self.input_callback,
            10
        )
        
        # Subscribe to the benchmark output topic
        self.output_sub = self.create_subscription(
            PointCloud2,
            '/benchmark/points',
            self.output_callback,
            10
        )
        
        self.input_count = 0
        self.output_count = 0
        self.start_time = time.time()
        
        # Timer for status updates
        self.timer = self.create_timer(2.0, self.status_callback)
        
        self.get_logger().info('Debug node started')
        self.get_logger().info('Monitoring:')
        self.get_logger().info('  Input: /vlp16/velodyne_points')
        self.get_logger().info('  Output: /benchmark/points')
        
    def input_callback(self, msg):
        self.input_count += 1
        if self.input_count == 1:
            self.get_logger().info(f'First message received on input topic!')
            self.get_logger().info(f'  Width: {msg.width}')
            self.get_logger().info(f'  Height: {msg.height}')
            self.get_logger().info(f'  Point step: {msg.point_step}')
            self.get_logger().info(f'  Row step: {msg.row_step}')
            self.get_logger().info(f'  Data size: {len(msg.data)} bytes')
    
    def output_callback(self, msg):
        self.output_count += 1
        if self.output_count == 1:
            self.get_logger().info(f'First message received on output topic!')
    
    def status_callback(self):
        elapsed = time.time() - self.start_time
        input_rate = self.input_count / elapsed if elapsed > 0 else 0
        output_rate = self.output_count / elapsed if elapsed > 0 else 0
        
        self.get_logger().info(
            f'Status: Input msgs: {self.input_count} ({input_rate:.1f} Hz), '
            f'Output msgs: {self.output_count} ({output_rate:.1f} Hz)'
        )
        
        if self.input_count > 0 and self.output_count == 0:
            self.get_logger().warning('Input messages received but no output! Check pointcloud_receiver node.')
        elif self.input_count == 0:
            self.get_logger().warning('No input messages received. Check tcpreplay and topic name.')


def main(args=None):
    rclpy.init(args=args)
    node = DebugNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()