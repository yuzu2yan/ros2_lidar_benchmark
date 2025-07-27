#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time


class ComprehensiveTopicTester(Node):
    def __init__(self):
        super().__init__('comprehensive_topic_tester')
        
        # Try both QoS profiles
        qos_configs = [
            ("BEST_EFFORT", QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.VOLATILE
            )),
            ("RELIABLE", QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.VOLATILE
            ))
        ]
        
        # Topics to test
        self.test_topics = [
            '/vlp16/velodyne_points',
            '/vlp16/points_filtered',
            '/velodyne_points',  # Sometimes without namespace
            '/points_filtered'   # Sometimes without namespace
        ]
        
        self.results = {}
        
        # Test each topic with each QoS
        for topic in self.test_topics:
            self.results[topic] = {}
            for qos_name, qos_profile in qos_configs:
                key = f"{topic}_{qos_name}"
                self.results[topic][qos_name] = 0
                
                # Create subscription
                self.create_subscription(
                    PointCloud2,
                    topic,
                    lambda msg, t=topic, q=qos_name: self.callback(msg, t, q),
                    qos_profile
                )
        
        # Timer for status
        self.timer = self.create_timer(3.0, self.status_callback)
        self.start_time = time.time()
        
        self.get_logger().info('Testing all possible topic/QoS combinations...')
        
    def callback(self, msg, topic, qos_name):
        self.results[topic][qos_name] += 1
        
        if self.results[topic][qos_name] == 1:
            self.get_logger().info(f'SUCCESS! Data received on {topic} with {qos_name} QoS')
            self.get_logger().info(f'  Points: {msg.width * msg.height}')
            self.get_logger().info(f'  Frame ID: {msg.header.frame_id}')
    
    def status_callback(self):
        elapsed = time.time() - self.start_time
        
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Status after {elapsed:.1f} seconds:')
        
        working_configs = []
        
        for topic in self.test_topics:
            topic_works = False
            for qos_name, count in self.results[topic].items():
                if count > 0:
                    rate = count / elapsed
                    self.get_logger().info(f'{topic} + {qos_name}: {count} msgs ({rate:.1f} Hz) ✓')
                    working_configs.append((topic, qos_name))
                    topic_works = True
            
            if not topic_works:
                self.get_logger().warning(f'{topic}: NO DATA with any QoS ✗')
        
        if working_configs:
            self.get_logger().info('\nWorking configurations found:')
            for topic, qos in working_configs:
                self.get_logger().info(f'  - Topic: {topic}, QoS: {qos}')
            
            # Recommend the best one
            best_topic, best_qos = working_configs[0]
            self.get_logger().info(f'\nRECOMMENDED CONFIG:')
            self.get_logger().info(f'  input_topic: "{best_topic}"')
            self.get_logger().info(f'  use_best_effort_qos: {best_qos == "BEST_EFFORT"}')
        else:
            self.get_logger().error('\nNO WORKING CONFIGURATION FOUND!')
            self.get_logger().error('Please check:')
            self.get_logger().error('1. Is the filter running?')
            self.get_logger().error('2. Is the pcap file being played?')
            self.get_logger().error('3. Run: ros2 topic list')


def main(args=None):
    rclpy.init(args=args)
    node = ComprehensiveTopicTester()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()