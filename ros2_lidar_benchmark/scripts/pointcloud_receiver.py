#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import time


class PointCloudReceiver(Node):
    def __init__(self):
        super().__init__('pointcloud_receiver')
        
        self.declare_parameter('input_topic', '/lidar/points')
        self.declare_parameter('output_topic', '/benchmark/points')
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        
        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.pointcloud_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            PointCloud2,
            output_topic,
            10
        )
        
        self.msg_count = 0
        self.last_msg_time = None
        self.start_time = time.time()
        
        self.get_logger().info(f'PointCloud Receiver started. Listening on {input_topic}')
        
    def pointcloud_callback(self, msg):
        current_time = time.time()
        
        if self.last_msg_time:
            interval = current_time - self.last_msg_time
            self.get_logger().debug(f'Message interval: {interval:.3f}s')
        
        self.last_msg_time = current_time
        self.msg_count += 1
        
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)
        
        if self.msg_count % 10 == 0:
            elapsed = current_time - self.start_time
            avg_rate = self.msg_count / elapsed
            self.get_logger().info(
                f'Received {self.msg_count} messages. Avg rate: {avg_rate:.2f} Hz'
            )


def main(args=None):
    rclpy.init(args=args)
    
    node = PointCloudReceiver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()