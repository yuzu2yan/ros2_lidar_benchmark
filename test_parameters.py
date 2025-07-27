#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class TestParameterNode(Node):
    def __init__(self):
        super().__init__('test_parameter_node')
        
        # Test declaring different parameter types
        self.declare_parameter('test_float', 60.0)
        self.declare_parameter('test_int', 60)
        self.declare_parameter('test_string', 'test')
        self.declare_parameter('test_bool', True)
        
        # Get parameters
        float_val = self.get_parameter('test_float').value
        int_val = self.get_parameter('test_int').value
        string_val = self.get_parameter('test_string').value
        bool_val = self.get_parameter('test_bool').value
        
        self.get_logger().info(f'Float parameter: {float_val} (type: {type(float_val)})')
        self.get_logger().info(f'Int parameter: {int_val} (type: {type(int_val)})')
        self.get_logger().info(f'String parameter: {string_val} (type: {type(string_val)})')
        self.get_logger().info(f'Bool parameter: {bool_val} (type: {type(bool_val)})')

def main():
    rclpy.init()
    node = TestParameterNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()