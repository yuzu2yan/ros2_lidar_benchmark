#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import sys


class SimpleChecker(Node):
    def __init__(self, use_best_effort=True):
        super().__init__('simple_checker')
        
        self.topic_name = '/vlp16/velodyne_points'
        self.msg_count = 0
        
        # Configure QoS
        if use_best_effort:
            qos_profile = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.VOLATILE
            )
            self.get_logger().info(f'Using BEST_EFFORT QoS for {self.topic_name}')
        else:
            qos_profile = QoSProfile(depth=10)
            self.get_logger().info(f'Using DEFAULT QoS for {self.topic_name}')
        
        # Create subscription
        self.subscription = self.create_subscription(
            PointCloud2,
            self.topic_name,
            self.callback,
            qos_profile
        )
        
        # Timer for status
        self.timer = self.create_timer(2.0, self.status_callback)
        
    def callback(self, msg):
        self.msg_count += 1
        if self.msg_count == 1:
            self.get_logger().info('SUCCESS! First message received!')
            self.get_logger().info(f'  Width: {msg.width}, Height: {msg.height}')
            self.get_logger().info(f'  Point cloud size: {msg.width * msg.height} points')
            self.get_logger().info(f'  Data size: {len(msg.data)} bytes')
    
    def status_callback(self):
        if self.msg_count == 0:
            self.get_logger().warning(f'No messages received yet on {self.topic_name}')
        else:
            self.get_logger().info(f'Total messages received: {self.msg_count}')


def main(args=None):
    # Check command line arguments
    use_best_effort = True
    if len(sys.argv) > 1 and sys.argv[1] == '--default-qos':
        use_best_effort = False
    
    rclpy.init(args=args)
    node = SimpleChecker(use_best_effort)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()