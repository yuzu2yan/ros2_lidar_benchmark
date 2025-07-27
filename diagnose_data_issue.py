#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, LivelinessPolicy
from rclpy.qos import QoSPresetProfiles
import time
import subprocess


class DataDiagnostics(Node):
    def __init__(self):
        super().__init__('data_diagnostics')
        
        # Get the actual topic from config
        self.test_topic = '/vlp16/points_filtered'
        
        self.get_logger().info(f'Diagnosing data issue for topic: {self.test_topic}')
        
        # First, check with ros2 topic echo
        self.check_with_cli()
        
        # Try various QoS profiles
        self.qos_profiles = {
            'sensor_data': QoSPresetProfiles.SENSOR_DATA.value,
            'best_effort_volatile': QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                liveliness=LivelinessPolicy.AUTOMATIC
            ),
            'best_effort_10': QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            ),
            'reliable': QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            ),
            'transient_local': QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        }
        
        self.subscriptions = {}
        self.msg_counts = {}
        
        # Create subscriptions with different QoS profiles
        for name, qos in self.qos_profiles.items():
            self.msg_counts[name] = 0
            sub = self.create_subscription(
                PointCloud2,
                self.test_topic,
                lambda msg, n=name: self.callback(msg, n),
                qos
            )
            self.subscriptions[name] = sub
            self.get_logger().info(f'Created subscription with {name} QoS')
        
        # Timer for status
        self.timer = self.create_timer(2.0, self.status_callback)
        self.start_time = time.time()
        
    def check_with_cli(self):
        """Check if ros2 topic echo works"""
        self.get_logger().info('Checking with ros2 topic echo...')
        
        try:
            # Try to get one message with ros2 topic echo
            result = subprocess.run(
                f'timeout 2 ros2 topic echo {self.test_topic} --once',
                shell=True,
                capture_output=True,
                text=True
            )
            
            if result.returncode == 0 and len(result.stdout) > 0:
                self.get_logger().info('✓ ros2 topic echo WORKS - data is flowing')
                
                # Try to get QoS info
                qos_result = subprocess.run(
                    f'ros2 topic info -v {self.test_topic}',
                    shell=True,
                    capture_output=True,
                    text=True
                )
                if 'QoS profile' in qos_result.stdout:
                    self.get_logger().info('Publisher QoS profile:')
                    for line in qos_result.stdout.split('\n'):
                        if 'Reliability:' in line or 'Durability:' in line:
                            self.get_logger().info(f'  {line.strip()}')
            else:
                self.get_logger().warning('✗ ros2 topic echo failed')
        except Exception as e:
            self.get_logger().error(f'Error checking with CLI: {e}')
    
    def callback(self, msg, qos_name):
        self.msg_counts[qos_name] += 1
        
        if self.msg_counts[qos_name] == 1:
            self.get_logger().info(f'SUCCESS with {qos_name} QoS!')
            self.get_logger().info(f'  Points: {msg.width * msg.height}')
            self.get_logger().info(f'  Frame: {msg.header.frame_id}')
    
    def status_callback(self):
        elapsed = time.time() - self.start_time
        
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'Results after {elapsed:.1f}s:')
        
        working_profiles = []
        for name, count in self.msg_counts.items():
            if count > 0:
                rate = count / elapsed
                self.get_logger().info(f'{name}: {count} msgs ({rate:.1f} Hz) ✓')
                working_profiles.append(name)
            else:
                self.get_logger().info(f'{name}: NO DATA ✗')
        
        if working_profiles:
            self.get_logger().info(f'\nWorking QoS profiles: {", ".join(working_profiles)}')
            self.get_logger().info('\nRECOMMENDATION:')
            if 'sensor_data' in working_profiles:
                self.get_logger().info('Use QoSPresetProfiles.SENSOR_DATA')
            else:
                self.get_logger().info(f'Use {working_profiles[0]} profile')
        else:
            self.get_logger().error('\nPROBLEM: Data visible with echo but not in ROS 2 node')
            self.get_logger().error('Possible causes:')
            self.get_logger().error('1. QoS mismatch (but we tried many profiles)')
            self.get_logger().error('2. Network/DDS configuration issue')
            self.get_logger().error('3. Try setting: export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp')


def main(args=None):
    rclpy.init(args=args)
    node = DataDiagnostics()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()