#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time


class PointCloudReceiver(Node):
    def __init__(self):
        super().__init__('pointcloud_receiver')
        
        self.declare_parameter('input_topic', '/lidar/points')
        self.declare_parameter('output_topic', '/benchmark/points')
        self.declare_parameter('use_best_effort_qos', True)
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        use_best_effort = self.get_parameter('use_best_effort_qos').value
        
        # Configure QoS for sensor data
        if use_best_effort:
            qos_profile = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.VOLATILE
            )
            self.get_logger().info('Using BEST_EFFORT QoS (recommended for sensor data)')
        else:
            qos_profile = QoSProfile(depth=10)
            self.get_logger().info('Using default QoS')
        
        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.pointcloud_callback,
            qos_profile
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
        self.get_logger().info(f'Publishing to {output_topic}')
        
        # Debug timer to check subscription status
        self.debug_timer = self.create_timer(5.0, self.debug_callback)
        
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
    
    def debug_callback(self):
        if self.msg_count == 0:
            self.get_logger().warning(f'No messages received on {self.get_parameter("input_topic").value}')
            self.get_logger().warning('Please check:')
            self.get_logger().warning('1. Is tcpreplay running?')
            self.get_logger().warning('2. Is the topic name correct?')
            self.get_logger().warning('3. Run: ros2 topic list')
            self.get_logger().warning('4. Run: ros2 topic hz /vlp16/velodyne_points')


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