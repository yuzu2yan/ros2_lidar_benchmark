#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Empty
import json
import time
import os
from datetime import datetime


class BenchmarkAnalyzer(Node):
    def __init__(self):
        super().__init__('benchmark_analyzer')
        
        self.declare_parameter('output_file', '/tmp/lidar_benchmark_report.xlsx')
        self.declare_parameter('analysis_duration', 60.0)
        self.declare_parameter('config_file', '')
        
        self.output_file = self.get_parameter('output_file').value
        self.analysis_duration = self.get_parameter('analysis_duration').value
        config_file = self.get_parameter('config_file').value
        
        # Load config file if provided
        self.config_data = None
        self.viz_output_dir = '/tmp/lidar_benchmark'  # Default
        
        if config_file and os.path.exists(config_file):
            import yaml
            with open(config_file, 'r') as f:
                self.config_data = yaml.safe_load(f)
                self.get_logger().info(f'Loaded config from: {config_file}')
                
                # Get visualization output directory from config
                if 'visualization' in self.config_data:
                    self.viz_output_dir = self.config_data['visualization'].get('output_dir', self.viz_output_dir)
        
        self.diagnostics_sub = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
            10
        )
        
        self.data = {
            'start_time': datetime.now().isoformat(),
            'duration_seconds': self.analysis_duration,
            'metrics': {},
            'system_stats': {},
            'summary': {}
        }
        
        self.metrics_history = []
        self.system_history = []
        self.start_time = time.time()
        self.analysis_completed = False  # Flag to prevent multiple analysis
        
        self.timer = self.create_timer(1.0, self.check_duration)
        
        # Publisher for shutdown signal
        self.shutdown_pub = self.create_publisher(Empty, '/benchmark/shutdown', 10)
        
        self.get_logger().info(f'Benchmark Analyzer started. Recording for {self.analysis_duration} seconds...')
        
    def diagnostics_callback(self, msg):
        for status in msg.status:
            data_point = {
                'timestamp': time.time() - self.start_time,
                'name': status.name,
                'level': status.level,
                'message': status.message,
                'values': {}
            }
            
            for kv in status.values:
                try:
                    data_point['values'][kv.key] = float(kv.value)
                except:
                    data_point['values'][kv.key] = kv.value
            
            if 'LiDAR Benchmark' in status.name:
                self.metrics_history.append(data_point)
            elif 'System Resources' in status.name:
                self.system_history.append(data_point)
    
    def check_duration(self):
        elapsed = time.time() - self.start_time
        if elapsed >= self.analysis_duration and not self.analysis_completed:
            # Set flag to prevent multiple analysis
            self.analysis_completed = True
            
            self.get_logger().info('Analysis duration reached. Processing results...')
            
            # Analyze and save (this includes graph generation)
            self.analyze_and_save()
            
            # Wait for file I/O to complete
            self.get_logger().info('Waiting for file operations to complete...')
            time.sleep(3.0)
            
            self.get_logger().info('Analysis complete. Shutting down...')
            
            # Send shutdown signal to other nodes
            shutdown_msg = Empty()
            self.shutdown_pub.publish(shutdown_msg)
            
            # Wait longer for other nodes to clean up properly
            self.get_logger().info('Waiting for other nodes to shutdown...')
            time.sleep(5.0)
            
            # Exit this node
            raise SystemExit
    
    def analyze_and_save(self):
        if self.metrics_history:
            hz_values = [d['values'].get('current_hz', 0) for d in self.metrics_history]
            jitter_values = [d['values'].get('jitter_ms', 0) for d in self.metrics_history]
            bandwidth_values = [d['values'].get('bandwidth_mbps', 0) for d in self.metrics_history]
            messages_per_sec = [d['values'].get('messages_per_second', 0) for d in self.metrics_history]
            mbytes_per_sec = [d['values'].get('mbytes_per_second', 0) for d in self.metrics_history]
            kpoints_per_sec = [d['values'].get('kpoints_per_second', 0) for d in self.metrics_history]
            points_per_msg = [d['values'].get('avg_points_per_message', 0) for d in self.metrics_history if d['values'].get('avg_points_per_message', 0) > 0]
            
            self.data['metrics'] = {
                'samples': len(self.metrics_history),
                'frequency': {
                    'average_hz': sum(hz_values) / len(hz_values) if hz_values else 0,
                    'min_hz': min(hz_values) if hz_values else 0,
                    'max_hz': max(hz_values) if hz_values else 0,
                    'std_hz': self.calculate_std(hz_values)
                },
                'jitter': {
                    'average_ms': sum(jitter_values) / len(jitter_values) if jitter_values else 0,
                    'min_ms': min(jitter_values) if jitter_values else 0,
                    'max_ms': max(jitter_values) if jitter_values else 0,
                    'std_ms': self.calculate_std(jitter_values)
                },
                'bandwidth': {
                    'average_mbps': sum(bandwidth_values) / len(bandwidth_values) if bandwidth_values else 0,
                    'min_mbps': min(bandwidth_values) if bandwidth_values else 0,
                    'max_mbps': max(bandwidth_values) if bandwidth_values else 0
                },
                'throughput': {
                    'average_messages_per_sec': sum(messages_per_sec) / len(messages_per_sec) if messages_per_sec else 0,
                    'average_mbytes_per_sec': sum(mbytes_per_sec) / len(mbytes_per_sec) if mbytes_per_sec else 0,
                    'average_kpoints_per_sec': sum(kpoints_per_sec) / len(kpoints_per_sec) if kpoints_per_sec else 0,
                    'max_kpoints_per_sec': max(kpoints_per_sec) if kpoints_per_sec else 0,
                    'average_points_per_message': sum(points_per_msg) / len(points_per_msg) if points_per_msg else 0
                }
            }
        
        if self.system_history:
            cpu_values = [d['values'].get('cpu_percent', 0) for d in self.system_history]
            memory_values = [d['values'].get('memory_percent', 0) for d in self.system_history]
            temp_values = [d['values'].get('cpu_temp_c', 0) for d in self.system_history if d['values'].get('cpu_temp_c', 0) > 0]
            
            self.data['system_stats'] = {
                'samples': len(self.system_history),
                'cpu': {
                    'average_percent': sum(cpu_values) / len(cpu_values) if cpu_values else 0,
                    'max_percent': max(cpu_values) if cpu_values else 0,
                    'min_percent': min(cpu_values) if cpu_values else 0
                },
                'memory': {
                    'average_percent': sum(memory_values) / len(memory_values) if memory_values else 0,
                    'max_percent': max(memory_values) if memory_values else 0,
                    'min_percent': min(memory_values) if memory_values else 0
                }
            }
            
            if temp_values:
                self.data['system_stats']['temperature'] = {
                    'average_celsius': sum(temp_values) / len(temp_values),
                    'max_celsius': max(temp_values),
                    'min_celsius': min(temp_values)
                }
                
                # Extract Jetson-specific temperatures if available
                jetson_temps = {}
                for d in self.system_history:
                    for key, value in d['values'].items():
                        if key.startswith('jetson_') and key.endswith('_temp_c'):
                            if key not in jetson_temps:
                                jetson_temps[key] = []
                            jetson_temps[key].append(value)
                
                if jetson_temps:
                    self.data['system_stats']['jetson_temperatures'] = {}
                    for zone, temps in jetson_temps.items():
                        zone_name = zone.replace('jetson_', '').replace('_temp_c', '')
                        self.data['system_stats']['jetson_temperatures'][zone_name] = {
                            'average_celsius': sum(temps) / len(temps),
                            'max_celsius': max(temps),
                            'min_celsius': min(temps)
                        }
        
        self.data['summary'] = {
            'performance_rating': self.calculate_performance_rating(),
            'stability_rating': self.calculate_stability_rating(),
            'recommendations': self.generate_recommendations()
        }
        
        self.data['end_time'] = datetime.now().isoformat()
        
        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
        
        # Save JSON data temporarily
        json_file = self.output_file.replace('.xlsx', '.json')
        with open(json_file, 'w') as f:
            json.dump(self.data, f, indent=2)
        
        # Generate Excel report as primary output
        try:
            # Check if visualization data exists
            # Use the configured visualization output directory
            viz_data_file = os.path.join(self.viz_output_dir, 'visualization_data.json')
            self.get_logger().info(f'Looking for visualization data at: {viz_data_file}')
            
            if os.path.exists(viz_data_file):
                self.get_logger().info(f'Found visualization data: {viz_data_file}')
                # Generate graphs to folder
                try:
                    try:
                        from graph_generator import GraphGenerator
                    except ImportError:
                        # Try with full path import
                        import sys
                        script_dir = os.path.dirname(os.path.abspath(__file__))
                        sys.path.insert(0, script_dir)
                        from graph_generator import GraphGenerator
                    
                    # Get graph output directory from config or parameter
                    graph_output_dir = '/tmp/lidar_benchmark_graphs'  # Default to absolute path
                    if hasattr(self, 'config_data') and self.config_data:
                        graph_output_dir = self.config_data.get('benchmark', {}).get('graph_output_dir', graph_output_dir)
                    
                    # Ensure absolute path
                    if not os.path.isabs(graph_output_dir):
                        graph_output_dir = os.path.abspath(graph_output_dir)
                    
                    self.get_logger().info(f'Graph output directory: {graph_output_dir}')
                    
                    # Create parent directory if it doesn't exist
                    os.makedirs(graph_output_dir, exist_ok=True)
                    self.get_logger().info(f'Ensured graph output directory exists')
                    
                    # Generate graphs
                    self.get_logger().info('Starting graph generation...')
                    graph_gen = GraphGenerator(json_file, viz_data_file, graph_output_dir)
                    graph_path = graph_gen.generate_all_graphs()
                    self.get_logger().info(f'Graphs successfully saved to: {graph_path}')
                    
                except Exception as e:
                    self.get_logger().error(f'Failed to generate graphs: {str(e)}')
                    import traceback
                    self.get_logger().error(traceback.format_exc())
            else:
                self.get_logger().warning(f'Visualization data not found: {viz_data_file}')
            
            # Always generate Excel report (without images)
            try:
                from excel_report_generator import ExcelReportGenerator
            except ImportError:
                # Try with full path import
                import sys
                script_dir = os.path.dirname(os.path.abspath(__file__))
                sys.path.insert(0, script_dir)
                from excel_report_generator import ExcelReportGenerator
            
            generator = ExcelReportGenerator(json_file, self.output_file)
            excel_path = generator.generate()
            self.get_logger().info(f'Excel report saved to: {excel_path}')
            
            # Remove temporary JSON file if Excel was successful
            if os.path.exists(excel_path):
                os.remove(json_file)
        except Exception as e:
            self.get_logger().warning(f'Failed to generate Excel report: {str(e)}')
            self.get_logger().info(f'JSON report saved to: {json_file}')
        
        self.print_summary()
    
    def calculate_std(self, values):
        if not values:
            return 0
        mean = sum(values) / len(values)
        variance = sum((x - mean) ** 2 for x in values) / len(values)
        return variance ** 0.5
    
    def calculate_performance_rating(self):
        if not self.data['metrics']:
            return 'No data'
        
        avg_hz = self.data['metrics']['frequency']['average_hz']
        if avg_hz >= 20:
            return 'Excellent'
        elif avg_hz >= 10:
            return 'Good'
        elif avg_hz >= 5:
            return 'Fair'
        else:
            return 'Poor'
    
    def calculate_stability_rating(self):
        if not self.data['metrics']:
            return 'No data'
        
        avg_jitter = self.data['metrics']['jitter']['average_ms']
        if avg_jitter <= 10:
            return 'Excellent'
        elif avg_jitter <= 25:
            return 'Good'
        elif avg_jitter <= 50:
            return 'Fair'
        else:
            return 'Poor'
    
    def generate_recommendations(self):
        recommendations = []
        
        if self.data.get('metrics'):
            if self.data['metrics']['frequency']['average_hz'] < 10:
                recommendations.append('Consider optimizing point cloud processing to improve frequency')
            
            if self.data['metrics']['jitter']['average_ms'] > 50:
                recommendations.append('High jitter detected - check for system load or network issues')
            
            if self.data['metrics']['bandwidth']['average_mbps'] > 100:
                recommendations.append('High bandwidth usage - consider data compression or reduced resolution')
        
        if self.data.get('system_stats'):
            if self.data['system_stats']['cpu']['average_percent'] > 80:
                recommendations.append('High CPU usage - consider performance optimization or hardware upgrade')
            
            if self.data['system_stats']['memory']['average_percent'] > 80:
                recommendations.append('High memory usage - check for memory leaks or increase system RAM')
        
        return recommendations if recommendations else ['System performing within normal parameters']
    
    def print_summary(self):
        print("\n" + "="*60)
        print("LIDAR BENCHMARK SUMMARY")
        print("="*60)
        
        if self.data.get('metrics'):
            print(f"\nFrequency Performance:")
            print(f"  Average: {self.data['metrics']['frequency']['average_hz']:.2f} Hz")
            print(f"  Range: {self.data['metrics']['frequency']['min_hz']:.2f} - {self.data['metrics']['frequency']['max_hz']:.2f} Hz")
            
            print(f"\nJitter Analysis:")
            print(f"  Average: {self.data['metrics']['jitter']['average_ms']:.2f} ms")
            print(f"  Range: {self.data['metrics']['jitter']['min_ms']:.2f} - {self.data['metrics']['jitter']['max_ms']:.2f} ms")
            
            print(f"\nBandwidth Usage:")
            print(f"  Average: {self.data['metrics']['bandwidth']['average_mbps']:.2f} Mbps")
            
            print(f"\nThroughput Performance:")
            print(f"  Messages/sec: {self.data['metrics']['throughput']['average_messages_per_sec']:.2f}")
            print(f"  MB/sec: {self.data['metrics']['throughput']['average_mbytes_per_sec']:.2f}")
            print(f"  Points/sec: {self.data['metrics']['throughput']['average_kpoints_per_sec']:.2f}K")
            print(f"  Points/message: {self.data['metrics']['throughput']['average_points_per_message']:.0f}")
        
        if self.data.get('system_stats'):
            print(f"\nSystem Resources:")
            print(f"  CPU Average: {self.data['system_stats']['cpu']['average_percent']:.1f}%")
            print(f"  Memory Average: {self.data['system_stats']['memory']['average_percent']:.1f}%")
        
        print(f"\nOverall Ratings:")
        print(f"  Performance: {self.data['summary']['performance_rating']}")
        print(f"  Stability: {self.data['summary']['stability_rating']}")
        
        print("\nRecommendations:")
        for rec in self.data['summary']['recommendations']:
            print(f"  - {rec}")
        
        print("="*60 + "\n")


def main(args=None):
    rclpy.init(args=args)
    
    node = BenchmarkAnalyzer()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node:
            node.get_logger().info('Shutting down benchmark_analyzer...')
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()