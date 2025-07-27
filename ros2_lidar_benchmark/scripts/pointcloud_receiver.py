#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data
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
            # Use the standard sensor data QoS profile
            qos_profile = qos_profile_sensor_data
            self.get_logger().info('Using SENSOR_DATA QoS profile (best for LiDAR data)')
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
            input_topic = self.get_parameter("input_topic").value
            self.get_logger().warning(f'No messages received on {input_topic}')
            
            # Check if topic exists
            topics = dict(self.get_topic_names_and_types())
            if input_topic in topics:
                self.get_logger().info(f'✓ Topic {input_topic} exists')
                
                # Check for publishers
                pub_info = self.get_publishers_info_by_topic(input_topic)
                if pub_info:
                    self.get_logger().info(f'✓ Found {len(pub_info)} publisher(s)')
                    for info in pub_info:
                        self.get_logger().info(f'  Publisher: {info.node_namespace}/{info.node_name}')
                else:
                    self.get_logger().warning('✗ No publishers found!')
            else:
                self.get_logger().error(f'✗ Topic {input_topic} does not exist!')
                self.get_logger().info('Available PointCloud2 topics:')
                for topic, types in topics.items():
                    if 'sensor_msgs/msg/PointCloud2' in types:
                        self.get_logger().info(f'  - {topic}')
            
            self.get_logger().warning('Please verify:')
            self.get_logger().warning('1. The filter launch file is running')
            self.get_logger().warning('2. The topic name in benchmark_config.yaml is correct')
            self.get_logger().warning(f'3. Run: ros2 topic hz {input_topic}')


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