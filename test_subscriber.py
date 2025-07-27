#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time

class TestSubscriber(Node):
    def __init__(self):
        super().__init__('test_subscriber')
        
        # Create different QoS profiles to test
        self.qos_profiles = {
            'default': QoSProfile(depth=10),
            'sensor_data': QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
                durability=DurabilityPolicy.VOLATILE
            ),
            'reliable': QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                durability=DurabilityPolicy.VOLATILE
            ),
            'transient_local': QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                durability=DurabilityPolicy.TRANSIENT_LOCAL
            )
        }
        
        # Test topics based on the launch file
        self.test_topics = [
            '/vlp16/points_filtered',
            '/vlp32/points_filtered',
            '/vlp16/velodyne_points',
            '/vlp32/velodyne_points',
            'vlp16/points_filtered',  # without leading slash
            'vlp32/points_filtered',
            '/points_filtered',
            'points_filtered'
        ]
        
        self.subscribers = []
        self.message_counts = {}
        self.last_messages = {}
        
        # Create subscribers for all topic/QoS combinations
        for topic in self.test_topics:
            for qos_name, qos_profile in self.qos_profiles.items():
                sub_name = f"{topic}_{qos_name}"
                try:
                    sub = self.create_subscription(
                        PointCloud2,
                        topic,
                        lambda msg, t=topic, q=qos_name: self.callback(msg, t, q),
                        qos_profile
                    )
                    self.subscribers.append(sub)
                    self.message_counts[sub_name] = 0
                    self.get_logger().info(f"Created subscription to '{topic}' with QoS '{qos_name}'")
                except Exception as e:
                    self.get_logger().error(f"Failed to create subscription to '{topic}' with QoS '{qos_name}': {e}")
        
        # Create timer to report status
        self.timer = self.create_timer(2.0, self.report_status)
        self.start_time = time.time()
        
    def callback(self, msg, topic, qos_name):
        sub_name = f"{topic}_{qos_name}"
        self.message_counts[sub_name] += 1
        self.last_messages[sub_name] = {
            'frame_id': msg.header.frame_id,
            'timestamp': msg.header.stamp,
            'width': msg.width,
            'height': msg.height,
            'point_step': msg.point_step,
            'row_step': msg.row_step,
            'data_size': len(msg.data)
        }
        
        # Log first message received for each subscription
        if self.message_counts[sub_name] == 1:
            self.get_logger().info(f"First message received on '{topic}' with QoS '{qos_name}':")
            self.get_logger().info(f"  Frame ID: {msg.header.frame_id}")
            self.get_logger().info(f"  Points: {msg.width} x {msg.height}")
            self.get_logger().info(f"  Data size: {len(msg.data)} bytes")
    
    def report_status(self):
        elapsed = time.time() - self.start_time
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"Status report after {elapsed:.1f} seconds:")
        
        # Group by topic
        topics_with_data = {}
        for sub_name, count in self.message_counts.items():
            if count > 0:
                topic = sub_name.rsplit('_', 1)[0]
                qos = sub_name.rsplit('_', 1)[1]
                if topic not in topics_with_data:
                    topics_with_data[topic] = []
                topics_with_data[topic].append((qos, count))
        
        if topics_with_data:
            self.get_logger().info("Topics receiving data:")
            for topic, qos_data in topics_with_data.items():
                self.get_logger().info(f"\n  Topic: '{topic}'")
                for qos, count in qos_data:
                    sub_name = f"{topic}_{qos}"
                    if sub_name in self.last_messages:
                        info = self.last_messages[sub_name]
                        self.get_logger().info(f"    QoS '{qos}': {count} messages")
                        self.get_logger().info(f"      Frame: {info['frame_id']}, Points: {info['width']}x{info['height']}")
        else:
            self.get_logger().warning("No messages received on any topic!")
        
        # List discovered topics
        self.get_logger().info(f"\nDiscovered topics of type PointCloud2:")
        topics = self.get_topic_names_and_types()
        for topic, types in topics:
            if 'sensor_msgs/msg/PointCloud2' in types:
                self.get_logger().info(f"  - {topic}")

def main(args=None):
    rclpy.init(args=args)
    node = TestSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()