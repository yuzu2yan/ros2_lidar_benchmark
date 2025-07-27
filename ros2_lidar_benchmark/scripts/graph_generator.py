#!/usr/bin/env python3

import json
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
import numpy as np
import os
import argparse
from datetime import datetime
import yaml


class GraphGenerator:
    def __init__(self, json_file, visualization_data_file=None, output_dir=None):
        """Initialize graph generator"""
        self.json_file = json_file
        self.visualization_data_file = visualization_data_file
        self.output_dir = output_dir or './lidar_benchmark_graphs'
        
        # Load benchmark data
        with open(json_file, 'r') as f:
            self.data = json.load(f)
        
        # Load visualization data if available
        self.viz_data = None
        if visualization_data_file and os.path.exists(visualization_data_file):
            with open(visualization_data_file, 'r') as f:
                self.viz_data = json.load(f)
            print(f"Loaded visualization data with {len(self.viz_data.get('timestamps', []))} data points")
    
    def generate_all_graphs(self):
        """Generate all graphs and save to output directory"""
        # Create output directory with timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_path = os.path.join(self.output_dir, f'benchmark_{timestamp}')
        
        print(f"Creating directory: {output_path}")
        try:
            os.makedirs(output_path, exist_ok=True)
            print(f"Directory created successfully")
        except Exception as e:
            print(f"Failed to create directory: {e}")
            raise
        
        print(f"Generating graphs in: {output_path}")
        
        # Generate time series graphs if visualization data exists
        if self.viz_data:
            self._generate_time_series_graphs(output_path)
        
        # Generate summary graphs
        self._generate_summary_graphs(output_path)
        
        # Save metadata
        metadata = {
            'timestamp': timestamp,
            'analysis_duration': self.data.get('duration_seconds', 0),
            'performance_rating': self.data.get('summary', {}).get('performance_rating', 'N/A'),
            'stability_rating': self.data.get('summary', {}).get('stability_rating', 'N/A'),
            'graph_count': len(os.listdir(output_path)) - 1  # Exclude metadata file
        }
        
        with open(os.path.join(output_path, 'metadata.json'), 'w') as f:
            json.dump(metadata, f, indent=2)
        
        print(f"Generated {metadata['graph_count']} graphs")
        print(f"Graphs location: {output_path}")
        return output_path
    
    def _generate_time_series_graphs(self, output_dir):
        """Generate time series graphs"""
        # Use a clean style
        plt.style.use('default')
        
        # Common settings
        fig_size = (12, 8)
        colors = {
            'hz': '#1f77b4',
            'jitter': '#ff7f0e',
            'bandwidth': '#2ca02c',
            'throughput': '#d62728',
            'cpu': '#9467bd',
            'memory': '#8c564b',
            'temperature': '#e377c2'
        }
        
        # 1. Publishing Frequency
        if 'hz' in self.viz_data and len(self.viz_data['hz']) > 0:
            self._create_time_series_plot(
                self.viz_data['timestamps'][:len(self.viz_data['hz'])],
                self.viz_data['hz'],
                'LiDAR Publishing Frequency Over Time',
                'Time (seconds)',
                'Frequency (Hz)',
                colors['hz'],
                os.path.join(output_dir, '01_frequency_timeline.png')
            )
        
        # 2. Jitter
        if 'jitter' in self.viz_data and len(self.viz_data['jitter']) > 0:
            self._create_time_series_plot(
                self.viz_data['timestamps'][:len(self.viz_data['jitter'])],
                self.viz_data['jitter'],
                'Message Jitter Over Time',
                'Time (seconds)',
                'Jitter (ms)',
                colors['jitter'],
                os.path.join(output_dir, '02_jitter_timeline.png')
            )
        
        # 3. Bandwidth
        if 'bandwidth' in self.viz_data and len(self.viz_data['bandwidth']) > 0:
            self._create_time_series_plot(
                self.viz_data['timestamps'][:len(self.viz_data['bandwidth'])],
                self.viz_data['bandwidth'],
                'Network Bandwidth Usage Over Time',
                'Time (seconds)',
                'Bandwidth (Mbps)',
                colors['bandwidth'],
                os.path.join(output_dir, '03_bandwidth_timeline.png')
            )
        
        # 4. Throughput
        if 'throughput' in self.viz_data and len(self.viz_data['throughput']) > 0:
            self._create_time_series_plot(
                self.viz_data['timestamps'][:len(self.viz_data['throughput'])],
                self.viz_data['throughput'],
                'Point Cloud Throughput Over Time',
                'Time (seconds)',
                'Throughput (K points/sec)',
                colors['throughput'],
                os.path.join(output_dir, '04_throughput_timeline.png')
            )
        
        # 5. CPU Usage
        if 'cpu' in self.viz_data and len(self.viz_data['cpu']) > 0:
            self._create_time_series_plot(
                self.viz_data['timestamps'][:len(self.viz_data['cpu'])],
                self.viz_data['cpu'],
                'CPU Usage Over Time',
                'Time (seconds)',
                'CPU Usage (%)',
                colors['cpu'],
                os.path.join(output_dir, '05_cpu_timeline.png'),
                ylim=(0, 105)
            )
        
        # 6. Memory Usage
        if 'memory' in self.viz_data and len(self.viz_data['memory']) > 0:
            self._create_time_series_plot(
                self.viz_data['timestamps'][:len(self.viz_data['memory'])],
                self.viz_data['memory'],
                'Memory Usage Over Time',
                'Time (seconds)',
                'Memory Usage (%)',
                colors['memory'],
                os.path.join(output_dir, '06_memory_timeline.png'),
                ylim=(0, 105)
            )
        
        # 7. Temperature
        if 'temperature' in self.viz_data and len(self.viz_data['temperature']) > 0:
            self._create_time_series_plot(
                self.viz_data['timestamps'][:len(self.viz_data['temperature'])],
                self.viz_data['temperature'],
                'CPU Temperature Over Time',
                'Time (seconds)',
                'Temperature (°C)',
                colors['temperature'],
                os.path.join(output_dir, '07_temperature_timeline.png')
            )
    
    def _create_time_series_plot(self, x_data, y_data, title, xlabel, ylabel, color, filename, ylim=None):
        """Create a single time series plot"""
        plt.figure(figsize=(12, 8))
        
        # Main plot
        plt.plot(x_data, y_data, color=color, linewidth=2, alpha=0.8)
        
        # Calculate statistics
        avg_val = np.mean(y_data)
        std_val = np.std(y_data)
        min_val = np.min(y_data)
        max_val = np.max(y_data)
        
        # Add average line
        plt.axhline(y=avg_val, color='red', linestyle='--', linewidth=2, 
                   label=f'Average: {avg_val:.2f}')
        
        # Add min/max lines
        plt.axhline(y=min_val, color='gray', linestyle=':', linewidth=1, 
                   label=f'Min: {min_val:.2f}')
        plt.axhline(y=max_val, color='gray', linestyle=':', linewidth=1, 
                   label=f'Max: {max_val:.2f}')
        
        # Add shaded area for standard deviation
        if std_val > 0:
            plt.fill_between(x_data, avg_val - std_val, avg_val + std_val, 
                           alpha=0.2, color='gray', label=f'±1 STD: {std_val:.2f}')
        
        # Formatting
        plt.title(title, fontsize=18, fontweight='bold', pad=20)
        plt.xlabel(xlabel, fontsize=14)
        plt.ylabel(ylabel, fontsize=14)
        plt.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
        plt.legend(loc='best', fontsize=12, framealpha=0.9)
        
        if ylim:
            plt.ylim(ylim)
        
        # Add statistics text box
        stats_text = f'Statistics:\n'
        stats_text += f'Average: {avg_val:.2f}\n'
        stats_text += f'Std Dev: {std_val:.2f}\n'
        stats_text += f'Min: {min_val:.2f}\n'
        stats_text += f'Max: {max_val:.2f}'
        
        plt.text(0.02, 0.98, stats_text, transform=plt.gca().transAxes, 
                verticalalignment='top', fontsize=11,
                bbox=dict(boxstyle='round,pad=0.5', facecolor='white', 
                         edgecolor='gray', alpha=0.8))
        
        plt.tight_layout()
        plt.savefig(filename, dpi=300, bbox_inches='tight', facecolor='white')
        plt.close()
    
    def _generate_summary_graphs(self, output_dir):
        """Generate summary/comparison graphs"""
        # 1. Performance Overview Bar Chart
        self._create_performance_overview(output_dir)
        
        # 2. System Resource Usage Pie Chart
        self._create_resource_usage_chart(output_dir)
        
        # 3. Performance Rating Visualization
        self._create_rating_visualization(output_dir)
    
    def _create_performance_overview(self, output_dir):
        """Create performance overview bar chart"""
        plt.figure(figsize=(10, 8))
        
        metrics = {
            'Avg Hz': self.data['lidar_metrics']['average_hz'],
            'Avg Jitter (ms)': self.data['lidar_metrics']['average_jitter_ms'],
            'Avg BW (Mbps)': self.data['lidar_metrics']['average_bandwidth_mbps'],
            'Total Messages (K)': self.data['lidar_metrics']['total_messages'] / 1000,
            'Total Data (GB)': self.data['lidar_metrics']['total_mb'] / 1024
        }
        
        x = range(len(metrics))
        values = list(metrics.values())
        labels = list(metrics.keys())
        
        bars = plt.bar(x, values, color=['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd'])
        
        # Add value labels on bars
        for bar, value in zip(bars, values):
            height = bar.get_height()
            plt.text(bar.get_x() + bar.get_width()/2., height,
                    f'{value:.2f}',
                    ha='center', va='bottom', fontsize=12, fontweight='bold')
        
        plt.xlabel('Metrics', fontsize=14)
        plt.ylabel('Values', fontsize=14)
        plt.title('LiDAR Benchmark Performance Overview', fontsize=18, fontweight='bold', pad=20)
        plt.xticks(x, labels, rotation=45, ha='right')
        plt.grid(True, axis='y', alpha=0.3)
        plt.tight_layout()
        
        plt.savefig(os.path.join(output_dir, '08_performance_overview.png'), 
                   dpi=300, bbox_inches='tight', facecolor='white')
        plt.close()
    
    def _create_resource_usage_chart(self, output_dir):
        """Create resource usage pie chart"""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 7))
        
        # CPU Usage Distribution
        cpu_avg = self.data['system_resources']['average_cpu_percent']
        cpu_data = [cpu_avg, 100 - cpu_avg]
        cpu_labels = ['Used', 'Available']
        colors1 = ['#ff9999', '#99ff99']
        
        wedges1, texts1, autotexts1 = ax1.pie(cpu_data, labels=cpu_labels, colors=colors1, 
                                               autopct='%1.1f%%', startangle=90)
        ax1.set_title(f'Average CPU Usage\n({cpu_avg:.1f}%)', fontsize=16, fontweight='bold')
        
        # Memory Usage Distribution
        mem_avg = self.data['system_resources']['average_memory_percent']
        mem_data = [mem_avg, 100 - mem_avg]
        mem_labels = ['Used', 'Available']
        colors2 = ['#ffcc99', '#99ccff']
        
        wedges2, texts2, autotexts2 = ax2.pie(mem_data, labels=mem_labels, colors=colors2, 
                                               autopct='%1.1f%%', startangle=90)
        ax2.set_title(f'Average Memory Usage\n({mem_avg:.1f}%)', fontsize=16, fontweight='bold')
        
        plt.suptitle('System Resource Usage Distribution', fontsize=18, fontweight='bold')
        plt.tight_layout()
        
        plt.savefig(os.path.join(output_dir, '09_resource_usage.png'), 
                   dpi=300, bbox_inches='tight', facecolor='white')
        plt.close()
    
    def _create_rating_visualization(self, output_dir):
        """Create rating visualization"""
        plt.figure(figsize=(10, 8))
        
        # Define rating colors
        rating_colors = {
            'Excellent': '#00ff00',
            'Good': '#90EE90',
            'Fair': '#FFD700',
            'Poor': '#FF6B6B'
        }
        
        # Get ratings
        perf_rating = self.data['analysis']['performance_rating']
        stab_rating = self.data['analysis']['stability_rating']
        
        # Create bar chart
        categories = ['Performance', 'Stability']
        ratings = [perf_rating, stab_rating]
        colors = [rating_colors.get(r, '#808080') for r in ratings]
        
        bars = plt.bar(categories, [1, 1], color=colors, width=0.6)
        
        # Add rating text
        for i, (bar, rating) in enumerate(zip(bars, ratings)):
            plt.text(bar.get_x() + bar.get_width()/2., 0.5,
                    rating,
                    ha='center', va='center', fontsize=24, fontweight='bold', color='black')
        
        plt.ylim(0, 1.2)
        plt.ylabel('')
        plt.title('Benchmark Performance Ratings', fontsize=18, fontweight='bold', pad=20)
        plt.xticks(fontsize=16)
        
        # Remove y-axis
        ax = plt.gca()
        ax.set_yticks([])
        
        # Add legend
        legend_elements = [plt.Rectangle((0,0),1,1, facecolor=color, label=rating) 
                          for rating, color in rating_colors.items()]
        plt.legend(handles=legend_elements, loc='upper right', fontsize=12)
        
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, '10_performance_ratings.png'), 
                   dpi=300, bbox_inches='tight', facecolor='white')
        plt.close()


def main():
    parser = argparse.ArgumentParser(description='Generate benchmark graphs')
    parser.add_argument('--json-file', type=str, required=True,
                       help='Path to benchmark JSON report')
    parser.add_argument('--viz-data', type=str,
                       help='Path to visualization data JSON')
    parser.add_argument('--output-dir', type=str,
                       help='Output directory for graphs')
    parser.add_argument('--config', type=str,
                       help='Path to config file')
    
    args = parser.parse_args()
    
    # Load config if provided
    output_dir = args.output_dir
    if args.config and os.path.exists(args.config):
        with open(args.config, 'r') as f:
            config = yaml.safe_load(f)
            if not output_dir:
                output_dir = config.get('benchmark', {}).get('graph_output_dir', './lidar_benchmark_graphs')
    
    # Generate graphs
    generator = GraphGenerator(args.json_file, args.viz_data, output_dir)
    output_path = generator.generate_all_graphs()
    
    print(f"\nGraphs saved to: {output_path}")


if __name__ == '__main__':
    main()