#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data
import sys

class SimpleReceiver(Node):
    def __init__(self):
        super().__init__('simple_receiver')
        
        # Topic from the filter
        topic = '/vlp16/points_filtered'
        
        self.get_logger().info(f'Testing simple receiver for {topic}')
        self.get_logger().info('Using same QoS as rviz2 (qos_profile_sensor_data)')
        
        self.msg_count = 0
        
        # Create subscription with sensor data QoS (same as rviz2)
        self.subscription = self.create_subscription(
            PointCloud2,
            topic,
            self.callback,
            qos_profile_sensor_data
        )
        
        # Timer to report status
        self.timer = self.create_timer(1.0, self.timer_callback)
        
    def callback(self, msg):
        self.msg_count += 1
        if self.msg_count == 1:
            self.get_logger().info('SUCCESS! First message received!')
            self.get_logger().info(f'  Frame ID: {msg.header.frame_id}')
            self.get_logger().info(f'  Points: {msg.width * msg.height}')
            self.get_logger().info(f'  Point step: {msg.point_step}')
            self.get_logger().info(f'  Row step: {msg.row_step}')
        
    def timer_callback(self):
        if self.msg_count > 0:
            self.get_logger().info(f'Receiving data: {self.msg_count} messages so far')
        else:
            self.get_logger().warning('No data received yet...')

def main():
    rclpy.init()
    node = SimpleReceiver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f'Total messages received: {node.msg_count}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()