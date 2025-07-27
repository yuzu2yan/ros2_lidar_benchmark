#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import collections
import time
import json


class MetricsCollector(Node):
    def __init__(self):
        super().__init__('metrics_collector')
        
        self.declare_parameter('topic_to_monitor', '/benchmark/points')
        self.declare_parameter('window_size', 100)
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('use_best_effort_qos', True)
        
        self.topic = self.get_parameter('topic_to_monitor').value
        self.window_size = self.get_parameter('window_size').value
        self.publish_rate = self.get_parameter('publish_rate').value
        use_best_effort = self.get_parameter('use_best_effort_qos').value
        
        # Configure QoS - matching the publisher
        if use_best_effort:
            # Use the standard sensor data QoS profile
            from rclpy.qos import QoSPresetProfiles
            qos_profile = QoSPresetProfiles.SENSOR_DATA.value
            self.get_logger().info(f'Using SENSOR_DATA QoS profile for monitoring {self.topic}')
        else:
            qos_profile = QoSProfile(depth=10)
            self.get_logger().info(f'Using default QoS for monitoring {self.topic}')
        
        self.subscription = self.create_subscription(
            PointCloud2,
            self.topic,
            self.message_callback,
            qos_profile
        )
        
        self.metrics_pub = self.create_publisher(
            Float64MultiArray,
            '/benchmark/metrics',
            10
        )
        
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            10
        )
        
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_metrics
        )
        
        self.timestamps = collections.deque(maxlen=self.window_size)
        self.message_sizes = collections.deque(maxlen=self.window_size)
        self.intervals = collections.deque(maxlen=self.window_size)
        self.points_per_message = collections.deque(maxlen=self.window_size)
        
        self.total_messages = 0
        self.total_bytes = 0
        self.total_points = 0
        self.start_time = time.time()
        self.last_throughput_time = time.time()
        self.throughput_messages = 0
        self.throughput_bytes = 0
        self.throughput_points = 0
        
        self.get_logger().info(f'Metrics Collector started for topic: {self.topic}')
        
        # Debug timer
        self.debug_timer = self.create_timer(5.0, self.debug_callback)
        
    def message_callback(self, msg):
        current_time = time.time()
        self.timestamps.append(current_time)
        
        message_size = len(msg.data) if hasattr(msg, 'data') else 0
        if message_size == 0:
            message_size = msg.width * msg.height * msg.point_step
        
        # Calculate number of points in the message
        num_points = msg.width * msg.height
        self.points_per_message.append(num_points)
        
        self.message_sizes.append(message_size)
        self.total_messages += 1
        self.total_bytes += message_size
        self.total_points += num_points
        
        # Update throughput counters
        self.throughput_messages += 1
        self.throughput_bytes += message_size
        self.throughput_points += num_points
        
        if len(self.timestamps) > 1:
            interval = self.timestamps[-1] - self.timestamps[-2]
            self.intervals.append(interval)
    
    def calculate_metrics(self):
        metrics = {}
        
        if len(self.timestamps) < 2:
            return metrics
        
        current_time = time.time()
        
        intervals_array = np.array(self.intervals)
        if len(intervals_array) > 0:
            metrics['current_hz'] = 1.0 / np.mean(intervals_array)
            metrics['jitter_ms'] = np.std(intervals_array) * 1000
            metrics['min_interval_ms'] = np.min(intervals_array) * 1000
            metrics['max_interval_ms'] = np.max(intervals_array) * 1000
        
        if len(self.timestamps) > 0:
            time_window = self.timestamps[-1] - self.timestamps[0]
            if time_window > 0:
                metrics['avg_hz'] = len(self.timestamps) / time_window
                metrics['bandwidth_mbps'] = (sum(self.message_sizes) / time_window) * 8 / 1e6
        
        elapsed_time = current_time - self.start_time
        if elapsed_time > 0:
            metrics['overall_avg_hz'] = self.total_messages / elapsed_time
            metrics['overall_bandwidth_mbps'] = (self.total_bytes / elapsed_time) * 8 / 1e6
        
        metrics['total_messages'] = self.total_messages
        metrics['total_mb'] = self.total_bytes / 1e6
        
        if len(self.message_sizes) > 0:
            metrics['avg_message_size_kb'] = np.mean(self.message_sizes) / 1024
        
        # Throughput calculations
        throughput_time_window = current_time - self.last_throughput_time
        if throughput_time_window > 0:
            metrics['messages_per_second'] = self.throughput_messages / throughput_time_window
            metrics['mbytes_per_second'] = (self.throughput_bytes / throughput_time_window) / 1e6
            metrics['points_per_second'] = self.throughput_points / throughput_time_window
            metrics['kpoints_per_second'] = metrics['points_per_second'] / 1000
            
            # Reset throughput counters every calculation
            if throughput_time_window >= 1.0:  # Reset every second
                self.last_throughput_time = current_time
                self.throughput_messages = 0
                self.throughput_bytes = 0
                self.throughput_points = 0
        
        if len(self.points_per_message) > 0:
            metrics['avg_points_per_message'] = np.mean(self.points_per_message)
            metrics['total_points_millions'] = self.total_points / 1e6
        
        return metrics
    
    def publish_metrics(self):
        metrics = self.calculate_metrics()
        
        if not metrics:
            return
        
        metrics_msg = Float64MultiArray()
        metrics_msg.data = [
            metrics.get('current_hz', 0.0),
            metrics.get('jitter_ms', 0.0),
            metrics.get('bandwidth_mbps', 0.0),
            metrics.get('avg_message_size_kb', 0.0),
            metrics.get('total_messages', 0.0),
            metrics.get('total_mb', 0.0),
            metrics.get('messages_per_second', 0.0),
            metrics.get('mbytes_per_second', 0.0),
            metrics.get('kpoints_per_second', 0.0)
        ]
        self.metrics_pub.publish(metrics_msg)
        
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        status = DiagnosticStatus()
        status.name = f"LiDAR Benchmark: {self.topic}"
        status.level = DiagnosticStatus.OK
        
        if metrics.get('current_hz', 0) < 5:
            status.level = DiagnosticStatus.WARN
            status.message = "Low frequency detected"
        elif metrics.get('jitter_ms', 0) > 50:
            status.level = DiagnosticStatus.WARN
            status.message = "High jitter detected"
        else:
            status.message = "Operating normally"
        
        for key, value in metrics.items():
            kv = KeyValue()
            kv.key = key
            kv.value = f"{value:.3f}"
            status.values.append(kv)
        
        diag_array.status.append(status)
        self.diagnostics_pub.publish(diag_array)
        
        self.get_logger().info(
            f"Hz: {metrics.get('current_hz', 0):.2f}, "
            f"Jitter: {metrics.get('jitter_ms', 0):.2f}ms, "
            f"BW: {metrics.get('bandwidth_mbps', 0):.2f}Mbps, "
            f"Throughput: {metrics.get('kpoints_per_second', 0):.1f}K pts/s"
        )
    
    def debug_callback(self):
        if self.total_messages == 0:
            self.get_logger().warning(f'No messages received on {self.topic}')
            self.get_logger().warning('Waiting for point cloud data...')


def main(args=None):
    rclpy.init(args=args)
    
    node = MetricsCollector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()