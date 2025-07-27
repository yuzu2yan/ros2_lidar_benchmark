#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
import time

class RvizCompatibleSubscriber(Node):
    def __init__(self):
        super().__init__('rviz_compatible_subscriber')
        
        # Use the same QoS profile that rviz2 uses for sensor data
        self.qos_sensor = qos_profile_sensor_data
        
        # Also create a custom QoS that's commonly used
        self.qos_custom = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Based on the launch file analysis, the filtered points should be at:
        # For VLP16: namespace is 'vlp16', output topic is 'points_filtered'
        # So the full topic should be '/vlp16/points_filtered'
        
        self.topics_to_test = [
            '/vlp16/points_filtered',
            '/vlp16/velodyne_points',
            'vlp16/points_filtered',
            '/points_filtered'
        ]
        
        self.subscribers = []
        self.message_info = {}
        
        for topic in self.topics_to_test:
            # Try with sensor data QoS (what rviz2 uses)
            try:
                sub = self.create_subscription(
                    PointCloud2,
                    topic,
                    lambda msg, t=topic: self.callback(msg, t, 'sensor_data'),
                    self.qos_sensor
                )
                self.subscribers.append((topic, 'sensor_data', sub))
                self.get_logger().info(f"Subscribed to '{topic}' with sensor_data QoS")
            except Exception as e:
                self.get_logger().error(f"Failed to subscribe to '{topic}': {e}")
        
        # Timer to check topic discovery
        self.timer = self.create_timer(1.0, self.check_topics)
        self.first_check = True
        
    def callback(self, msg, topic, qos_type):
        key = f"{topic}_{qos_type}"
        if key not in self.message_info:
            self.message_info[key] = {'count': 0, 'first_msg': None}
        
        self.message_info[key]['count'] += 1
        
        if self.message_info[key]['count'] == 1:
            self.get_logger().info(f"\n{'='*60}")
            self.get_logger().info(f"SUCCESS! Received first message on '{topic}' with {qos_type} QoS")
            self.get_logger().info(f"Message details:")
            self.get_logger().info(f"  - Frame ID: {msg.header.frame_id}")
            self.get_logger().info(f"  - Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
            self.get_logger().info(f"  - Width (points): {msg.width}")
            self.get_logger().info(f"  - Height: {msg.height}")
            self.get_logger().info(f"  - Point step: {msg.point_step}")
            self.get_logger().info(f"  - Row step: {msg.row_step}")
            self.get_logger().info(f"  - Data size: {len(msg.data)} bytes")
            self.get_logger().info(f"  - Is bigendian: {msg.is_bigendian}")
            self.get_logger().info(f"  - Is dense: {msg.is_dense}")
            self.get_logger().info(f"  - Fields: {[f.name for f in msg.fields]}")
            self.get_logger().info(f"{'='*60}\n")
            
            self.message_info[key]['first_msg'] = {
                'frame_id': msg.header.frame_id,
                'width': msg.width,
                'fields': [f.name for f in msg.fields]
            }
        
        if self.message_info[key]['count'] % 10 == 0:
            self.get_logger().info(f"Received {self.message_info[key]['count']} messages on '{topic}'")
    
    def check_topics(self):
        if self.first_check:
            self.first_check = False
            self.get_logger().info("\nSearching for PointCloud2 topics...")
            
            # Get all topics
            topics_and_types = self.get_topic_names_and_types()
            pc2_topics = []
            
            for topic, types in topics_and_types:
                if 'sensor_msgs/msg/PointCloud2' in types:
                    pc2_topics.append(topic)
            
            if pc2_topics:
                self.get_logger().info(f"\nFound {len(pc2_topics)} PointCloud2 topics:")
                for topic in pc2_topics:
                    self.get_logger().info(f"  - {topic}")
                    
                    # Get publisher info
                    pub_info = self.get_publishers_info_by_topic(topic)
                    if pub_info:
                        self.get_logger().info(f"    Publishers: {len(pub_info)}")
                        for info in pub_info:
                            self.get_logger().info(f"      - Node: {info.node_name}")
                            self.get_logger().info(f"        Namespace: {info.node_namespace}")
                            self.get_logger().info(f"        QoS: {info.qos_profile}")
            else:
                self.get_logger().warning("No PointCloud2 topics found!")
            
            self.get_logger().info("\nWaiting for messages...\n")

def main(args=None):
    rclpy.init(args=args)
    node = RvizCompatibleSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Print summary
        if node.message_info:
            node.get_logger().info("\nFinal summary:")
            for key, info in node.message_info.items():
                topic = key.rsplit('_', 1)[0]
                qos = key.rsplit('_', 1)[1]
                node.get_logger().info(f"  {topic} ({qos}): {info['count']} messages")
                if info['first_msg']:
                    node.get_logger().info(f"    Frame: {info['first_msg']['frame_id']}")
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()