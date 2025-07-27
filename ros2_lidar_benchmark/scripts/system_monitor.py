#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Float64MultiArray
import psutil
import threading
import time


class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')
        
        self.declare_parameter('monitor_rate', 1.0)
        self.declare_parameter('process_name', '')
        
        self.monitor_rate = self.get_parameter('monitor_rate').value
        self.process_name = self.get_parameter('process_name').value
        
        self.system_pub = self.create_publisher(
            Float64MultiArray,
            '/benchmark/system_resources',
            10
        )
        
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            10
        )
        
        self.timer = self.create_timer(
            1.0 / self.monitor_rate,
            self.monitor_system
        )
        
        self.cpu_percent_history = []
        self.memory_percent_history = []
        self.process = None
        
        if self.process_name:
            self.find_process()
        
        self.get_logger().info('System Monitor started')
        
    def find_process(self):
        for proc in psutil.process_iter(['pid', 'name']):
            if self.process_name in proc.info['name']:
                self.process = psutil.Process(proc.info['pid'])
                self.get_logger().info(f"Monitoring process: {proc.info['name']} (PID: {proc.info['pid']})")
                break
    
    def get_system_metrics(self):
        metrics = {}
        
        metrics['cpu_percent'] = psutil.cpu_percent(interval=0.1)
        metrics['cpu_count'] = psutil.cpu_count()
        
        memory = psutil.virtual_memory()
        metrics['memory_percent'] = memory.percent
        metrics['memory_used_gb'] = memory.used / (1024**3)
        metrics['memory_available_gb'] = memory.available / (1024**3)
        
        disk = psutil.disk_usage('/')
        metrics['disk_percent'] = disk.percent
        metrics['disk_free_gb'] = disk.free / (1024**3)
        
        try:
            temps = psutil.sensors_temperatures()
            if temps:
                cpu_temps = []
                for name, entries in temps.items():
                    for entry in entries:
                        if 'cpu' in name.lower() or 'cpu' in entry.label.lower():
                            cpu_temps.append(entry.current)
                if cpu_temps:
                    metrics['cpu_temp_c'] = sum(cpu_temps) / len(cpu_temps)
        except:
            pass
        
        if self.process and self.process.is_running():
            try:
                metrics['process_cpu_percent'] = self.process.cpu_percent(interval=0.1)
                metrics['process_memory_mb'] = self.process.memory_info().rss / (1024**2)
                metrics['process_threads'] = self.process.num_threads()
            except:
                self.process = None
        
        return metrics
    
    def monitor_system(self):
        metrics = self.get_system_metrics()
        
        self.cpu_percent_history.append(metrics['cpu_percent'])
        self.memory_percent_history.append(metrics['memory_percent'])
        if len(self.cpu_percent_history) > 60:
            self.cpu_percent_history.pop(0)
        if len(self.memory_percent_history) > 60:
            self.memory_percent_history.pop(0)
        
        metrics['cpu_avg_1min'] = sum(self.cpu_percent_history) / len(self.cpu_percent_history)
        metrics['memory_avg_1min'] = sum(self.memory_percent_history) / len(self.memory_percent_history)
        
        system_msg = Float64MultiArray()
        system_msg.data = [
            metrics.get('cpu_percent', 0.0),
            metrics.get('memory_percent', 0.0),
            metrics.get('cpu_avg_1min', 0.0),
            metrics.get('memory_avg_1min', 0.0),
            metrics.get('process_cpu_percent', 0.0),
            metrics.get('process_memory_mb', 0.0),
            metrics.get('cpu_temp_c', 0.0)
        ]
        self.system_pub.publish(system_msg)
        
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        status = DiagnosticStatus()
        status.name = "System Resources"
        status.level = DiagnosticStatus.OK
        
        if metrics['cpu_percent'] > 90:
            status.level = DiagnosticStatus.ERROR
            status.message = "CPU usage critical"
        elif metrics['cpu_percent'] > 70:
            status.level = DiagnosticStatus.WARN
            status.message = "High CPU usage"
        elif metrics['memory_percent'] > 90:
            status.level = DiagnosticStatus.ERROR
            status.message = "Memory usage critical"
        elif metrics['memory_percent'] > 70:
            status.level = DiagnosticStatus.WARN
            status.message = "High memory usage"
        else:
            status.message = "System resources normal"
        
        for key, value in metrics.items():
            kv = KeyValue()
            kv.key = key
            kv.value = f"{value:.2f}"
            status.values.append(kv)
        
        diag_array.status.append(status)
        self.diagnostics_pub.publish(diag_array)
        
        self.get_logger().debug(
            f"CPU: {metrics['cpu_percent']:.1f}%, "
            f"Memory: {metrics['memory_percent']:.1f}%, "
            f"Temp: {metrics.get('cpu_temp_c', 0):.1f}°C"
        )


def main(args=None):
    rclpy.init(args=args)
    
    node = SystemMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()