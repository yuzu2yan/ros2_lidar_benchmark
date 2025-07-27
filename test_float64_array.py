#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

class TestFloat64Array(Node):
    def __init__(self):
        super().__init__('test_float64_array')
        
        self.publisher = self.create_publisher(Float64MultiArray, '/test/float64', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('Testing Float64MultiArray publishing...')
        
    def timer_callback(self):
        msg = Float64MultiArray()
        
        # Test different data types
        test_data = [
            1,      # int
            2.0,    # float
            np.float64(3.0),  # numpy float64
            np.float32(4.0),  # numpy float32
            int(5),           # explicit int
            float(6),         # explicit float
        ]
        
        self.get_logger().info('Original data types:')
        for i, val in enumerate(test_data):
            self.get_logger().info(f'  {i}: {val} (type: {type(val).__name__})')
        
        # Convert all to float
        msg.data = [float(x) for x in test_data]
        
        self.get_logger().info('Converted data types:')
        for i, val in enumerate(msg.data):
            self.get_logger().info(f'  {i}: {val} (type: {type(val).__name__})')
        
        try:
            self.publisher.publish(msg)
            self.get_logger().info('Successfully published Float64MultiArray')
        except Exception as e:
            self.get_logger().error(f'Failed to publish: {e}')

def main():
    rclpy.init()
    node = TestFloat64Array()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()