#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time


class DataFlowChecker(Node):
    def __init__(self):
        super().__init__('data_flow_checker')
        
        # Try different QoS profiles
        qos_profiles = [
            # Default profile
            QoSProfile(depth=10),
            # Best effort profile (common for sensor data)
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.VOLATILE
            ),
            # Reliable profile
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.VOLATILE
            )
        ]
        
        self.topic_name = '/vlp16/velodyne_points'
        self.subscriptions = []
        self.msg_counts = [0, 0, 0]
        
        # Create subscriptions with different QoS profiles
        for i, qos in enumerate(qos_profiles):
            # Create a unique callback for each subscription
            def make_callback(idx):
                return lambda msg: self.callback(msg, idx)
            
            sub = self.create_subscription(
                PointCloud2,
                self.topic_name,
                make_callback(i),
                qos
            )
            self.subscriptions.append(sub)
        
        self.get_logger().info(f'Checking data flow on {self.topic_name} with different QoS profiles...')
        self.get_logger().info('Profile 0: Default')
        self.get_logger().info('Profile 1: Best Effort (sensor data)')
        self.get_logger().info('Profile 2: Reliable')
        
        # Timer for status updates
        self.timer = self.create_timer(2.0, self.status_callback)
        self.start_time = time.time()
        
    def callback(self, msg, profile_idx):
        self.msg_counts[profile_idx] += 1
        
        if self.msg_counts[profile_idx] == 1:
            self.get_logger().info(f'First message received on Profile {profile_idx}!')
            self.get_logger().info(f'  Frame ID: {msg.header.frame_id}')
            self.get_logger().info(f'  Width x Height: {msg.width} x {msg.height}')
            self.get_logger().info(f'  Point step: {msg.point_step}')
            self.get_logger().info(f'  Row step: {msg.row_step}')
            self.get_logger().info(f'  Data size: {len(msg.data)} bytes')
            self.get_logger().info(f'  Is bigendian: {msg.is_bigendian}')
            self.get_logger().info(f'  Is dense: {msg.is_dense}')
            
            # Print field information
            self.get_logger().info('  Fields:')
            for field in msg.fields:
                self.get_logger().info(f'    - {field.name}: offset={field.offset}, datatype={field.datatype}, count={field.count}')
    
    def status_callback(self):
        elapsed = time.time() - self.start_time
        
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'Elapsed time: {elapsed:.1f}s')
        
        for i, count in enumerate(self.msg_counts):
            rate = count / elapsed if elapsed > 0 else 0
            self.get_logger().info(f'Profile {i}: {count} msgs ({rate:.1f} Hz)')
        
        if sum(self.msg_counts) == 0:
            self.get_logger().warning('No messages received on any QoS profile!')
            self.get_logger().warning('Possible issues:')
            self.get_logger().warning('1. Check if the pcap filter node is running')
            self.get_logger().warning('2. Verify the topic name matches exactly')
            self.get_logger().warning('3. Check ROS_DOMAIN_ID if using domain isolation')
        else:
            # Find which profile works best
            best_profile = self.msg_counts.index(max(self.msg_counts))
            self.get_logger().info(f'Best performing profile: {best_profile}')
            
            if best_profile == 1:
                self.get_logger().info('-> Use BEST_EFFORT QoS for sensor data')
            elif best_profile == 2:
                self.get_logger().info('-> Use RELIABLE QoS')


def main(args=None):
    rclpy.init(args=args)
    node = DataFlowChecker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()