#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data, QoSPresetProfiles
import time


class PointCloudReceiver(Node):
    def __init__(self):
        super().__init__('pointcloud_receiver')
        
        self.declare_parameter('input_topic', '/lidar/points')
        self.declare_parameter('output_topic', '/benchmark/points')
        self.declare_parameter('use_best_effort_qos', True)
        self.declare_parameter('auto_detect_qos', True)
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        use_best_effort = self.get_parameter('use_best_effort_qos').value
        auto_detect = self.get_parameter('auto_detect_qos').value
        
        self.input_topic = input_topic
        self.output_topic = output_topic
        self.subscription = None
        self.msg_count = 0
        self.last_msg_time = None
        self.start_time = time.time()
        
        # Create publisher first
        self.publisher = self.create_publisher(
            PointCloud2,
            output_topic,
            10
        )
        
        if auto_detect:
            self.get_logger().info('Auto-detecting optimal QoS settings...')
            self.auto_detect_qos()
        else:
            # Use configured QoS
            if use_best_effort:
                qos_profile = qos_profile_sensor_data
                self.get_logger().info('Using sensor_data QoS profile (BEST_EFFORT)')
            else:
                qos_profile = QoSProfile(depth=10)
                self.get_logger().info('Using default QoS (RELIABLE)')
            
            self.create_subscription_with_qos(qos_profile, 'configured')
        
        # Debug timer
        self.debug_timer = self.create_timer(5.0, self.debug_callback)
        
    def auto_detect_qos(self):
        """Try different QoS profiles to find one that works"""
        qos_profiles = [
            ('sensor_data', qos_profile_sensor_data),
            ('best_effort_volatile', QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=5
            )),
            ('reliable_volatile', QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )),
            ('default', QoSProfile(depth=10))
        ]
        
        # First, check if topic exists
        topics = dict(self.get_topic_names_and_types())
        if self.input_topic not in topics:
            self.get_logger().error(f'Topic {self.input_topic} does not exist!')
            self.get_logger().info('Available PointCloud2 topics:')
            for topic, types in topics.items():
                if 'sensor_msgs/msg/PointCloud2' in types:
                    self.get_logger().info(f'  - {topic}')
            return
        
        # Try to get publisher info for QoS matching
        pub_info = self.get_publishers_info_by_topic(self.input_topic)
        if pub_info:
            self.get_logger().info(f'Found {len(pub_info)} publisher(s) for {self.input_topic}')
            for info in pub_info:
                self.get_logger().info(f'  Publisher: {info.node_namespace}/{info.node_name}')
                # Try to match publisher QoS
                if info.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT:
                    self.get_logger().info('  Publisher uses BEST_EFFORT - using sensor_data QoS')
                    self.create_subscription_with_qos(qos_profile_sensor_data, 'auto-detected')
                    return
        
        # Try each QoS profile
        for name, qos in qos_profiles:
            try:
                self.get_logger().info(f'Trying {name} QoS...')
                self.create_subscription_with_qos(qos, name)
                # Wait a bit to see if we get messages
                rclpy.spin_once(self, timeout_sec=0.5)
                if self.msg_count > 0:
                    self.get_logger().info(f'Success with {name} QoS!')
                    return
                else:
                    # Remove subscription to try next
                    self.subscription = None
            except Exception as e:
                self.get_logger().warning(f'Failed with {name} QoS: {e}')
        
        self.get_logger().error('Could not establish subscription with any QoS profile!')
        
    def create_subscription_with_qos(self, qos_profile, qos_name):
        """Create subscription with given QoS"""
        try:
            self.subscription = self.create_subscription(
                PointCloud2,
                self.input_topic,
                self.pointcloud_callback,
                qos_profile
            )
            self.get_logger().info(f'Created subscription to {self.input_topic} with {qos_name} QoS')
            self.get_logger().info(f'Publishing to {self.output_topic}')
        except Exception as e:
            self.get_logger().error(f'Failed to create subscription: {e}')
            raise
        
    def pointcloud_callback(self, msg):
        current_time = time.time()
        
        if self.last_msg_time:
            interval = current_time - self.last_msg_time
            self.get_logger().debug(f'Message interval: {interval:.3f}s')
        
        self.last_msg_time = current_time
        self.msg_count += 1
        
        # Update timestamp
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)
        
        # Log first message details
        if self.msg_count == 1:
            self.get_logger().info(f'First message received!')
            self.get_logger().info(f'  Frame ID: {msg.header.frame_id}')
            self.get_logger().info(f'  Points: {msg.width}')
            self.get_logger().info(f'  Fields: {[f.name for f in msg.fields]}')
        
        if self.msg_count % 10 == 0:
            elapsed = current_time - self.start_time
            avg_rate = self.msg_count / elapsed
            self.get_logger().info(
                f'Received {self.msg_count} messages. Avg rate: {avg_rate:.2f} Hz'
            )
    
    def debug_callback(self):
        if self.msg_count == 0:
            self.get_logger().warning(f'No messages received on {self.input_topic}')
            self.get_logger().warning('Checking topic status...')
            
            # Check if topic exists
            topics = dict(self.get_topic_names_and_types())
            if self.input_topic in topics:
                self.get_logger().info(f'✓ Topic {self.input_topic} exists')
                
                # Check publishers
                pub_info = self.get_publishers_info_by_topic(self.input_topic)
                if pub_info:
                    self.get_logger().info(f'✓ {len(pub_info)} publisher(s) found')
                else:
                    self.get_logger().warning('✗ No publishers found on topic')
            else:
                self.get_logger().error(f'✗ Topic {self.input_topic} does not exist!')
                self.get_logger().info('Available PointCloud2 topics:')
                for topic, types in topics.items():
                    if 'sensor_msgs/msg/PointCloud2' in types:
                        self.get_logger().info(f'  - {topic}')


def main(args=None):
    rclpy.init(args=args)
    
    node = PointCloudReceiver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.msg_count > 0:
            elapsed = time.time() - node.start_time
            avg_rate = node.msg_count / elapsed
            node.get_logger().info(f'\nFinal stats: {node.msg_count} messages in {elapsed:.1f}s ({avg_rate:.2f} Hz)')
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()