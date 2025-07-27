#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.qos import QoSPresetProfiles, qos_profile_sensor_data
import time
import sys

class BenchmarkDiagnostic(Node):
    def __init__(self):
        super().__init__('benchmark_diagnostic')
        
        self.get_logger().info("Starting benchmark diagnostic...")
        
        # Test configuration from benchmark_config.yaml
        self.benchmark_topic = '/vlp16/points_filtered'
        
        # QoS profiles to test
        self.qos_profiles = {
            'sensor_data': qos_profile_sensor_data,
            'sensor_preset': QoSPresetProfiles.SENSOR_DATA.value,
            'best_effort': QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=5
            ),
            'reliable': QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        }
        
        self.subscribers = {}
        self.message_counts = {}
        self.qos_matches = {}
        
        # Create subscribers with different QoS
        for qos_name, qos_profile in self.qos_profiles.items():
            try:
                sub = self.create_subscription(
                    PointCloud2,
                    self.benchmark_topic,
                    lambda msg, q=qos_name: self.callback(msg, q),
                    qos_profile
                )
                self.subscribers[qos_name] = sub
                self.message_counts[qos_name] = 0
                self.get_logger().info(f"Created subscription with {qos_name} QoS")
            except Exception as e:
                self.get_logger().error(f"Failed to create subscription with {qos_name}: {e}")
        
        # Timer for diagnostics
        self.timer = self.create_timer(2.0, self.diagnostic_callback)
        self.start_time = time.time()
        self.first_report = True
        
    def callback(self, msg, qos_name):
        self.message_counts[qos_name] += 1
        
        if self.message_counts[qos_name] == 1:
            self.qos_matches[qos_name] = True
            self.get_logger().info(f"\n{'='*60}")
            self.get_logger().info(f"SUCCESS: {qos_name} QoS is receiving data!")
            self.get_logger().info(f"Frame ID: {msg.header.frame_id}")
            self.get_logger().info(f"Points: {msg.width}")
            self.get_logger().info(f"{'='*60}\n")
    
    def diagnostic_callback(self):
        elapsed = time.time() - self.start_time
        
        if self.first_report:
            self.first_report = False
            self.get_logger().info("\n" + "="*80)
            self.get_logger().info("DIAGNOSTIC REPORT")
            self.get_logger().info("="*80)
            
            # Check available topics
            topics_and_types = self.get_topic_names_and_types()
            pc2_topics = []
            
            for topic, types in topics_and_types:
                if 'sensor_msgs/msg/PointCloud2' in types:
                    pc2_topics.append(topic)
            
            self.get_logger().info(f"\nAvailable PointCloud2 topics:")
            if pc2_topics:
                for topic in pc2_topics:
                    self.get_logger().info(f"  ✓ {topic}")
                    
                    # Check if benchmark topic exists
                    if topic == self.benchmark_topic:
                        self.get_logger().info(f"    → This is the configured benchmark topic!")
                        
                        # Get publisher QoS
                        pub_info = self.get_publishers_info_by_topic(topic)
                        if pub_info:
                            for info in pub_info:
                                self.get_logger().info(f"    Publisher: {info.node_namespace}/{info.node_name}")
                                self.get_logger().info(f"    Publisher QoS: {info.qos_profile}")
            else:
                self.get_logger().error("  ✗ No PointCloud2 topics found!")
            
            self.get_logger().info(f"\nLooking for topic: {self.benchmark_topic}")
            if self.benchmark_topic in [t for t, _ in topics_and_types]:
                self.get_logger().info("  ✓ Topic exists")
            else:
                self.get_logger().error("  ✗ Topic does NOT exist!")
                self.get_logger().info("\nPossible issues:")
                self.get_logger().info("  1. The filter node is not running")
                self.get_logger().info("  2. The topic name in benchmark_config.yaml is incorrect")
                self.get_logger().info("  3. The namespace is different")
        
        # Report message counts
        if elapsed > 5.0:
            self.get_logger().info(f"\nAfter {elapsed:.1f} seconds:")
            
            if any(self.message_counts.values()):
                self.get_logger().info("\nWorking QoS configurations:")
                for qos_name, count in self.message_counts.items():
                    if count > 0:
                        rate = count / elapsed
                        self.get_logger().info(f"  ✓ {qos_name}: {count} messages ({rate:.1f} Hz)")
                
                # Recommend the best QoS
                best_qos = max(self.message_counts, key=self.message_counts.get)
                self.get_logger().info(f"\nRECOMMENDATION: Use '{best_qos}' QoS for best results")
                
                # Check if sensor_data or sensor_preset works
                if self.message_counts.get('sensor_data', 0) > 0:
                    self.get_logger().info("\n✓ The benchmark's default sensor_data QoS should work!")
                    self.get_logger().info("  The pointcloud_receiver.py is correctly configured.")
                elif self.message_counts.get('sensor_preset', 0) > 0:
                    self.get_logger().info("\n✓ The QoSPresetProfiles.SENSOR_DATA should work!")
                else:
                    self.get_logger().warning("\n⚠ The default sensor QoS may not be optimal")
                    self.get_logger().warning("  Consider modifying pointcloud_receiver.py")
                
            else:
                self.get_logger().error("\n✗ No messages received with any QoS configuration!")
                self.get_logger().error("\nTroubleshooting steps:")
                self.get_logger().error("1. Check if the filter is running:")
                self.get_logger().error("   ros2 node list | grep filter")
                self.get_logger().error("2. Check if data is being published:")
                self.get_logger().error(f"   ros2 topic hz {self.benchmark_topic}")
                self.get_logger().error("3. Check topic info:")
                self.get_logger().error(f"   ros2 topic info {self.benchmark_topic}")
                self.get_logger().error("4. Echo the topic:")
                self.get_logger().error(f"   ros2 topic echo {self.benchmark_topic} --no-arr")
            
            # Shutdown after diagnosis
            if elapsed > 10.0:
                self.get_logger().info("\nDiagnosis complete. Shutting down...")
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = BenchmarkDiagnostic()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()