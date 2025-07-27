#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np
import threading
import time


class BenchmarkVisualizer(Node):
    def __init__(self):
        super().__init__('benchmark_visualizer')
        
        self.declare_parameter('window_seconds', 60)
        self.declare_parameter('update_rate', 2.0)
        self.declare_parameter('save_plots', False)
        self.declare_parameter('output_dir', '/tmp/lidar_benchmark')
        
        self.window_seconds = self.get_parameter('window_seconds').value
        self.update_rate = self.get_parameter('update_rate').value
        self.save_plots = self.get_parameter('save_plots').value
        self.output_dir = self.get_parameter('output_dir').value
        
        self.metrics_sub = self.create_subscription(
            Float64MultiArray,
            '/benchmark/metrics',
            self.metrics_callback,
            10
        )
        
        self.system_sub = self.create_subscription(
            Float64MultiArray,
            '/benchmark/system_resources',
            self.system_callback,
            10
        )
        
        self.max_points = int(self.window_seconds * self.update_rate)
        
        self.timestamps = deque(maxlen=self.max_points)
        self.hz_values = deque(maxlen=self.max_points)
        self.jitter_values = deque(maxlen=self.max_points)
        self.bandwidth_values = deque(maxlen=self.max_points)
        self.cpu_values = deque(maxlen=self.max_points)
        self.memory_values = deque(maxlen=self.max_points)
        self.temp_values = deque(maxlen=self.max_points)
        
        self.start_time = time.time()
        self.data_lock = threading.Lock()
        
        self.setup_plots()
        
        self.get_logger().info('Benchmark Visualizer started')
        
    def metrics_callback(self, msg):
        with self.data_lock:
            current_time = time.time() - self.start_time
            self.timestamps.append(current_time)
            
            if len(msg.data) >= 3:
                self.hz_values.append(msg.data[0])
                self.jitter_values.append(msg.data[1])
                self.bandwidth_values.append(msg.data[2])
            
            if len(msg.data) >= 9:
                self.throughput_values.append(msg.data[8])  # kpoints_per_second
    
    def system_callback(self, msg):
        with self.data_lock:
            if len(msg.data) >= 2:
                self.cpu_values.append(msg.data[0])
                self.memory_values.append(msg.data[1])
            if len(msg.data) >= 7:
                self.temp_values.append(msg.data[6])  # CPU temperature
    
    def setup_plots(self):
        plt.style.use('dark_background')
        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4), (self.ax5, self.ax6)) = plt.subplots(3, 2, figsize=(12, 10))
        self.fig.suptitle('ROS 2 LiDAR Benchmark Dashboard', fontsize=16)
        
        self.ax1.set_title('Publishing Frequency')
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Frequency (Hz)')
        self.ax1.grid(True, alpha=0.3)
        self.line_hz, = self.ax1.plot([], [], 'g-', linewidth=2)
        
        self.ax2.set_title('Jitter')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Jitter (ms)')
        self.ax2.grid(True, alpha=0.3)
        self.line_jitter, = self.ax2.plot([], [], 'y-', linewidth=2)
        
        self.ax3.set_title('Bandwidth Usage')
        self.ax3.set_xlabel('Time (s)')
        self.ax3.set_ylabel('Bandwidth (Mbps)')
        self.ax3.grid(True, alpha=0.3)
        self.line_bandwidth, = self.ax3.plot([], [], 'c-', linewidth=2)
        
        self.ax4.set_title('System Resources')
        self.ax4.set_xlabel('Time (s)')
        self.ax4.set_ylabel('Usage (%)')
        self.ax4.grid(True, alpha=0.3)
        self.line_cpu, = self.ax4.plot([], [], 'r-', linewidth=2, label='CPU')
        self.line_memory, = self.ax4.plot([], [], 'b-', linewidth=2, label='Memory')
        self.ax4.legend(loc='upper right')
        
        self.ax5.set_title('Temperature')
        self.ax5.set_xlabel('Time (s)')
        self.ax5.set_ylabel('Temperature (°C)')
        self.ax5.grid(True, alpha=0.3)
        self.line_temp, = self.ax5.plot([], [], 'm-', linewidth=2, label='CPU Temp')
        self.ax5.legend(loc='upper right')
        
        self.ax6.set_title('Point Cloud Throughput')
        self.ax6.set_xlabel('Time (s)')
        self.ax6.set_ylabel('Throughput (K points/sec)')
        self.ax6.grid(True, alpha=0.3)
        self.line_throughput, = self.ax6.plot([], [], 'g-', linewidth=2)
        
        # Store throughput values
        self.throughput_values = deque(maxlen=self.max_points)
        
        plt.tight_layout()
        
        self.ani = animation.FuncAnimation(
            self.fig,
            self.update_plots,
            interval=int(1000/self.update_rate),
            blit=False
        )
        
    def update_plots(self, frame):
        with self.data_lock:
            if len(self.timestamps) > 0:
                times = list(self.timestamps)
                
                if len(self.hz_values) > 0:
                    self.line_hz.set_data(times[:len(self.hz_values)], list(self.hz_values))
                    self.ax1.relim()
                    self.ax1.autoscale_view()
                    
                    avg_hz = np.mean(self.hz_values)
                    self.ax1.set_title(f'Publishing Frequency (Avg: {avg_hz:.1f} Hz)')
                
                if len(self.jitter_values) > 0:
                    self.line_jitter.set_data(times[:len(self.jitter_values)], list(self.jitter_values))
                    self.ax2.relim()
                    self.ax2.autoscale_view()
                    
                    avg_jitter = np.mean(self.jitter_values)
                    self.ax2.set_title(f'Jitter (Avg: {avg_jitter:.1f} ms)')
                
                if len(self.bandwidth_values) > 0:
                    self.line_bandwidth.set_data(times[:len(self.bandwidth_values)], list(self.bandwidth_values))
                    self.ax3.relim()
                    self.ax3.autoscale_view()
                    
                    avg_bw = np.mean(self.bandwidth_values)
                    self.ax3.set_title(f'Bandwidth Usage (Avg: {avg_bw:.1f} Mbps)')
                
                if len(self.cpu_values) > 0 and len(self.memory_values) > 0:
                    cpu_times = times[-len(self.cpu_values):]
                    mem_times = times[-len(self.memory_values):]
                    
                    self.line_cpu.set_data(cpu_times, list(self.cpu_values))
                    self.line_memory.set_data(mem_times, list(self.memory_values))
                    self.ax4.relim()
                    self.ax4.autoscale_view()
                    self.ax4.set_ylim(0, 100)
                
                if len(self.temp_values) > 0:
                    temp_times = times[-len(self.temp_values):]
                    self.line_temp.set_data(temp_times, list(self.temp_values))
                    self.ax5.relim()
                    self.ax5.autoscale_view()
                    
                    avg_temp = np.mean(self.temp_values)
                    max_temp = np.max(self.temp_values)
                    self.ax5.set_title(f'Temperature (Avg: {avg_temp:.1f}°C, Max: {max_temp:.1f}°C)')
                
                if len(self.throughput_values) > 0:
                    throughput_times = times[:len(self.throughput_values)]
                    self.line_throughput.set_data(throughput_times, list(self.throughput_values))
                    self.ax6.relim()
                    self.ax6.autoscale_view()
                    
                    avg_throughput = np.mean(self.throughput_values)
                    max_throughput = np.max(self.throughput_values)
                    self.ax6.set_title(f'Point Cloud Throughput (Avg: {avg_throughput:.1f}K pts/s, Max: {max_throughput:.1f}K pts/s)')
                
                for ax in [self.ax1, self.ax2, self.ax3, self.ax4, self.ax5, self.ax6]:
                    if len(times) > 0:
                        ax.set_xlim(max(0, times[-1] - self.window_seconds), times[-1] + 1)
        
        return self.line_hz, self.line_jitter, self.line_bandwidth, self.line_cpu, self.line_memory, self.line_temp, self.line_throughput
    
    def save_current_plots(self):
        if self.save_plots:
            import os
            os.makedirs(self.output_dir, exist_ok=True)
            timestamp = time.strftime('%Y%m%d_%H%M%S')
            filename = os.path.join(self.output_dir, f'benchmark_{timestamp}.png')
            self.fig.savefig(filename, dpi=150, bbox_inches='tight')
            self.get_logger().info(f'Saved plot to {filename}')


def main(args=None):
    rclpy.init(args=args)
    
    node = BenchmarkVisualizer()
    
    def spin_node():
        rclpy.spin(node)
    
    spin_thread = threading.Thread(target=spin_node)
    spin_thread.start()
    
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.save_current_plots()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()


if __name__ == '__main__':
    main()