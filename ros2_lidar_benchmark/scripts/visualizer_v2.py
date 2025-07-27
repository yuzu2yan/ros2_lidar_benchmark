#!/usr/bin/env python3

import matplotlib
matplotlib.use('TkAgg')  # Set backend before importing pyplot

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
from collections import deque
import numpy as np
import threading
import time
import json
import os


class BenchmarkVisualizer(Node):
    def __init__(self):
        super().__init__('benchmark_visualizer')
        
        self.declare_parameter('window_seconds', 60.0)
        self.declare_parameter('update_rate', 5.0)
        self.declare_parameter('save_plots', True)
        self.declare_parameter('output_dir', '/tmp/lidar_benchmark')
        
        self.window_seconds = self.get_parameter('window_seconds').value
        self.update_rate = self.get_parameter('update_rate').value
        self.save_plots = self.get_parameter('save_plots').value
        self.output_dir = self.get_parameter('output_dir').value
        
        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.subscription_metrics = self.create_subscription(
            Float64MultiArray,
            '/benchmark/metrics',
            self.metrics_callback,
            10
        )
        
        self.subscription_system = self.create_subscription(
            Float64MultiArray,
            '/benchmark/system_resources',
            self.system_callback,
            10
        )
        
        self.max_points = int(self.window_seconds * 10)
        
        # Data storage
        self.data = {
            'timestamps': deque(maxlen=self.max_points),
            'hz': deque(maxlen=self.max_points),
            'jitter': deque(maxlen=self.max_points),
            'bandwidth': deque(maxlen=self.max_points),
            'throughput': deque(maxlen=self.max_points),
            'cpu': deque(maxlen=self.max_points),
            'memory': deque(maxlen=self.max_points),
            'temperature': deque(maxlen=self.max_points)
        }
        
        # Store all data for Excel export
        self.all_data = {
            'timestamps': [],
            'hz': [],
            'jitter': [],
            'bandwidth': [],
            'throughput': [],
            'cpu': [],
            'memory': [],
            'temperature': []
        }
        
        self.start_time = time.time()
        self.data_lock = threading.Lock()
        self.last_metrics_time = 0
        self.last_system_time = 0
        
        # Initialize plot in main thread
        self.fig = None
        self.axes = None
        self.lines = None
        self.plot_initialized = False
        
        self.get_logger().info('Benchmark Visualizer started')
        
        # Debug timer
        self.debug_timer = self.create_timer(5.0, self.debug_callback)
        
    def metrics_callback(self, msg):
        with self.data_lock:
            current_time = time.time() - self.start_time
            self.last_metrics_time = current_time
            
            # Store data
            self.data['timestamps'].append(current_time)
            self.all_data['timestamps'].append(current_time)
            
            if len(msg.data) >= 9:
                self.data['hz'].append(msg.data[0])
                self.data['jitter'].append(msg.data[1])
                self.data['bandwidth'].append(msg.data[2])
                self.data['throughput'].append(msg.data[8])  # kpoints_per_second
                
                self.all_data['hz'].append(msg.data[0])
                self.all_data['jitter'].append(msg.data[1])
                self.all_data['bandwidth'].append(msg.data[2])
                self.all_data['throughput'].append(msg.data[8])
            
            self.get_logger().debug(f'Received metrics: Hz={msg.data[0]:.2f}')
    
    def system_callback(self, msg):
        with self.data_lock:
            current_time = time.time() - self.start_time
            self.last_system_time = current_time
            
            if len(msg.data) >= 7:
                # Align system data with metrics timestamps
                if len(self.data['timestamps']) > len(self.data['cpu']):
                    self.data['cpu'].append(msg.data[0])
                    self.data['memory'].append(msg.data[1])
                    self.data['temperature'].append(msg.data[6])
                    
                    self.all_data['cpu'].append(msg.data[0])
                    self.all_data['memory'].append(msg.data[1])
                    self.all_data['temperature'].append(msg.data[6])
            
            self.get_logger().debug(f'Received system: CPU={msg.data[0]:.1f}%')
    
    def setup_plots(self):
        """Setup matplotlib plots"""
        plt.ion()  # Interactive mode ON
        
        # Create figure with 6 subplots
        self.fig = plt.figure(figsize=(14, 10))
        self.fig.suptitle('ROS 2 LiDAR Benchmark Real-time Monitor', fontsize=16)
        
        # Create subplots
        gs = self.fig.add_gridspec(3, 2, hspace=0.3, wspace=0.3)
        self.axes = {
            'hz': self.fig.add_subplot(gs[0, 0]),
            'jitter': self.fig.add_subplot(gs[0, 1]),
            'bandwidth': self.fig.add_subplot(gs[1, 0]),
            'throughput': self.fig.add_subplot(gs[1, 1]),
            'cpu': self.fig.add_subplot(gs[2, 0]),
            'memory_temp': self.fig.add_subplot(gs[2, 1])
        }
        
        # Configure each axis
        self.axes['hz'].set_title('Publishing Frequency')
        self.axes['hz'].set_ylabel('Frequency (Hz)')
        self.axes['hz'].grid(True, alpha=0.3)
        
        self.axes['jitter'].set_title('Jitter Analysis')
        self.axes['jitter'].set_ylabel('Jitter (ms)')
        self.axes['jitter'].grid(True, alpha=0.3)
        
        self.axes['bandwidth'].set_title('Bandwidth Usage')
        self.axes['bandwidth'].set_ylabel('Bandwidth (Mbps)')
        self.axes['bandwidth'].grid(True, alpha=0.3)
        
        self.axes['throughput'].set_title('Point Cloud Throughput')
        self.axes['throughput'].set_ylabel('Throughput (K points/sec)')
        self.axes['throughput'].grid(True, alpha=0.3)
        
        self.axes['cpu'].set_title('CPU Usage')
        self.axes['cpu'].set_ylabel('CPU (%)')
        self.axes['cpu'].set_ylim(0, 105)
        self.axes['cpu'].grid(True, alpha=0.3)
        
        self.axes['memory_temp'].set_title('Memory & Temperature')
        self.axes['memory_temp'].set_ylabel('Memory (%) / Temp (°C)')
        self.axes['memory_temp'].grid(True, alpha=0.3)
        
        # Set x-axis labels
        for ax in self.axes.values():
            ax.set_xlabel('Time (s)')
        
        # Create lines
        self.lines = {
            'hz': self.axes['hz'].plot([], [], 'b-', linewidth=2, label='Hz')[0],
            'jitter': self.axes['jitter'].plot([], [], 'r-', linewidth=2, label='Jitter')[0],
            'bandwidth': self.axes['bandwidth'].plot([], [], 'g-', linewidth=2, label='Bandwidth')[0],
            'throughput': self.axes['throughput'].plot([], [], 'm-', linewidth=2, label='Throughput')[0],
            'cpu': self.axes['cpu'].plot([], [], 'orange', linewidth=2, label='CPU')[0],
            'memory': self.axes['memory_temp'].plot([], [], 'c-', linewidth=2, label='Memory')[0],
            'temp': self.axes['memory_temp'].plot([], [], 'r--', linewidth=2, label='Temperature')[0]
        }
        
        # Add legends
        self.axes['memory_temp'].legend(loc='upper left')
        
        plt.tight_layout()
        
        # Show the plot
        self.fig.show()
        
        self.plot_initialized = True
        self.get_logger().info('Plots initialized successfully')
        
    def update_plots(self):
        """Update all plots with current data"""
        if not self.plot_initialized:
            return
            
        with self.data_lock:
            if len(self.data['timestamps']) == 0:
                return
            
            # Get current data
            times = list(self.data['timestamps'])
            
            # Update each line
            if len(self.data['hz']) > 0:
                self.lines['hz'].set_data(times[:len(self.data['hz'])], list(self.data['hz']))
                self.axes['hz'].relim()
                self.axes['hz'].autoscale_view()
                
                # Update title with average
                avg_hz = np.mean(self.data['hz']) if len(self.data['hz']) > 0 else 0
                self.axes['hz'].set_title(f'Publishing Frequency (Avg: {avg_hz:.1f} Hz)')
            
            if len(self.data['jitter']) > 0:
                self.lines['jitter'].set_data(times[:len(self.data['jitter'])], list(self.data['jitter']))
                self.axes['jitter'].relim()
                self.axes['jitter'].autoscale_view()
                
                avg_jitter = np.mean(self.data['jitter']) if len(self.data['jitter']) > 0 else 0
                self.axes['jitter'].set_title(f'Jitter Analysis (Avg: {avg_jitter:.1f} ms)')
            
            if len(self.data['bandwidth']) > 0:
                self.lines['bandwidth'].set_data(times[:len(self.data['bandwidth'])], list(self.data['bandwidth']))
                self.axes['bandwidth'].relim()
                self.axes['bandwidth'].autoscale_view()
                
                avg_bw = np.mean(self.data['bandwidth']) if len(self.data['bandwidth']) > 0 else 0
                self.axes['bandwidth'].set_title(f'Bandwidth Usage (Avg: {avg_bw:.1f} Mbps)')
            
            if len(self.data['throughput']) > 0:
                self.lines['throughput'].set_data(times[:len(self.data['throughput'])], list(self.data['throughput']))
                self.axes['throughput'].relim()
                self.axes['throughput'].autoscale_view()
                
                avg_tp = np.mean(self.data['throughput']) if len(self.data['throughput']) > 0 else 0
                self.axes['throughput'].set_title(f'Throughput (Avg: {avg_tp:.1f} K pts/s)')
            
            if len(self.data['cpu']) > 0:
                self.lines['cpu'].set_data(times[:len(self.data['cpu'])], list(self.data['cpu']))
                
                avg_cpu = np.mean(self.data['cpu']) if len(self.data['cpu']) > 0 else 0
                self.axes['cpu'].set_title(f'CPU Usage (Avg: {avg_cpu:.1f}%)')
            
            if len(self.data['memory']) > 0:
                self.lines['memory'].set_data(times[:len(self.data['memory'])], list(self.data['memory']))
                
                avg_mem = np.mean(self.data['memory']) if len(self.data['memory']) > 0 else 0
                self.axes['memory_temp'].set_title(f'Memory (Avg: {avg_mem:.1f}%) & Temperature')
            
            if len(self.data['temperature']) > 0:
                self.lines['temp'].set_data(times[:len(self.data['temperature'])], list(self.data['temperature']))
                self.axes['memory_temp'].relim()
                self.axes['memory_temp'].autoscale_view()
            
            # Update x-axis limits for all plots
            if len(times) > 0:
                xlim_min = max(0, times[-1] - self.window_seconds)
                xlim_max = times[-1] + 1
                for ax in self.axes.values():
                    ax.set_xlim(xlim_min, xlim_max)
            
            # Force redraw
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
    
    def save_plots_to_file(self):
        """Save current plots and data"""
        if self.save_plots and self.plot_initialized:
            timestamp = time.strftime('%Y%m%d_%H%M%S')
            
            # Save plot image
            plot_file = os.path.join(self.output_dir, f'benchmark_plot_{timestamp}.png')
            self.fig.savefig(plot_file, dpi=150, bbox_inches='tight')
            self.get_logger().info(f'Saved plot to {plot_file}')
            
            # Save data for Excel
            data_file = os.path.join(self.output_dir, 'visualization_data.json')
            with self.data_lock:
                with open(data_file, 'w') as f:
                    json.dump(self.all_data, f, indent=2)
            self.get_logger().info(f'Saved data to {data_file}')
    
    def debug_callback(self):
        with self.data_lock:
            self.get_logger().info(
                f'Data points - Hz: {len(self.data["hz"])}, '
                f'CPU: {len(self.data["cpu"])}, '
                f'Last metrics: {self.last_metrics_time:.1f}s ago'
            )


def main(args=None):
    rclpy.init(args=args)
    
    node = BenchmarkVisualizer()
    
    # Setup plots in main thread
    node.setup_plots()
    
    # Create update timer
    update_timer = None
    
    def update_callback():
        node.update_plots()
    
    # Start update timer after plots are initialized
    if node.plot_initialized:
        update_timer = node.create_timer(1.0 / node.update_rate, update_callback)
    
    # ROS spin in separate thread
    def spin_node():
        rclpy.spin(node)
    
    spin_thread = threading.Thread(target=spin_node, daemon=True)
    spin_thread.start()
    
    try:
        # Keep matplotlib event loop running
        plt.show(block=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_plots_to_file()
        node.destroy_node()
        rclpy.shutdown()
        if spin_thread.is_alive():
            spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()