#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time


class TopicTester(Node):
    def __init__(self):
        super().__init__('topic_tester')
        
        # QoS for sensor data
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Topics to test
        self.topics = [
            '/vlp16/velodyne_points',
            '/vlp16/points_filtered'
        ]
        
        self.msg_counts = {}
        self.first_msg_info = {}
        
        # Create subscriptions for each topic
        for topic in self.topics:
            self.msg_counts[topic] = 0
            self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, t=topic: self.callback(msg, t),
                sensor_qos
            )
        
        # Timer for status
        self.timer = self.create_timer(2.0, self.status_callback)
        self.start_time = time.time()
        
        self.get_logger().info('Testing both VLP16 topics...')
        
    def callback(self, msg, topic):
        self.msg_counts[topic] += 1
        
        if self.msg_counts[topic] == 1:
            self.first_msg_info[topic] = {
                'width': msg.width,
                'height': msg.height,
                'points': msg.width * msg.height,
                'size': len(msg.data),
                'frame_id': msg.header.frame_id
            }
            self.get_logger().info(f'First message on {topic}:')
            self.get_logger().info(f'  Points: {self.first_msg_info[topic]["points"]}')
            self.get_logger().info(f'  Size: {self.first_msg_info[topic]["size"]} bytes')
            self.get_logger().info(f'  Frame: {self.first_msg_info[topic]["frame_id"]}')
    
    def status_callback(self):
        elapsed = time.time() - self.start_time
        
        self.get_logger().info('-' * 50)
        self.get_logger().info(f'Status after {elapsed:.1f} seconds:')
        
        for topic in self.topics:
            count = self.msg_counts[topic]
            rate = count / elapsed if elapsed > 0 else 0
            if count > 0:
                self.get_logger().info(f'{topic}: {count} msgs ({rate:.1f} Hz) ✓')
            else:
                self.get_logger().warning(f'{topic}: NO DATA ✗')
        
        # Recommendation
        active_topics = [t for t in self.topics if self.msg_counts[t] > 0]
        if active_topics:
            self.get_logger().info(f'\nRecommended topic for benchmark: {active_topics[0]}')


def main(args=None):
    rclpy.init(args=args)
    node = TopicTester()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()