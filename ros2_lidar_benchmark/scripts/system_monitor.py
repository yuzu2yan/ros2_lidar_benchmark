#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Float64MultiArray, Empty
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
        
        # Cache for CPU percent calculation (needs two calls)
        self.process_cpu_cache = {}  # pid -> last_cpu_time tuple
        
        # Get CPU count for normalization
        self.cpu_count = psutil.cpu_count()
        
        if self.process_name:
            self.find_process()
        
        self._safe_log('info', 'System Monitor started')
        
        # Detect if running on Jetson
        self.is_jetson = self.detect_jetson()
        if self.is_jetson:
            self._safe_log('info', 'Jetson platform detected - temperature monitoring enabled')
        
        # Shutdown handling
        self.should_shutdown = False
        self.shutdown_sub = self.create_subscription(
            Empty,
            '/benchmark/shutdown',
            self.shutdown_callback,
            10
        )
    
    def _safe_log(self, level, message, *args, **kwargs):
        """Safely log a message, handling invalid ROS2 context"""
        try:
            logger = self.get_logger()
            if level == 'debug':
                logger.debug(message, *args, **kwargs)
            elif level == 'info':
                logger.info(message, *args, **kwargs)
            elif level == 'warn':
                logger.warn(message, *args, **kwargs)
            elif level == 'error':
                logger.error(message, *args, **kwargs)
        except Exception:
            # If ROS2 context is invalid, use print as fallback
            try:
                print(f"[{level.upper()}] {message}")
            except Exception:
                pass  # Ignore all logging errors
    
    def detect_jetson(self):
        """Detect if running on NVIDIA Jetson platform"""
        try:
            with open('/proc/device-tree/model', 'r') as f:
                model = f.read().lower()
                return 'jetson' in model or 'nvidia' in model
        except:
            return False
    
    def get_jetson_temperatures(self):
        """Get temperature readings from Jetson thermal zones"""
        if not self.is_jetson:
            return None
        
        temps = {}
        thermal_zones = {
            'cpu': '/sys/devices/virtual/thermal/thermal_zone0/temp',
            'gpu': '/sys/devices/virtual/thermal/thermal_zone1/temp',
            'aux': '/sys/devices/virtual/thermal/thermal_zone2/temp',
            'ao': '/sys/devices/virtual/thermal/thermal_zone3/temp',
            'pmic': '/sys/devices/virtual/thermal/thermal_zone4/temp',
            'tboard': '/sys/devices/virtual/thermal/thermal_zone5/temp',
            'tdiode': '/sys/devices/virtual/thermal/thermal_zone6/temp'
        }
        
        for zone_name, zone_path in thermal_zones.items():
            try:
                with open(zone_path, 'r') as f:
                    # Temperature is in millidegrees Celsius
                    temp_milli = float(f.read().strip())
                    temps[f'jetson_{zone_name}_temp_c'] = temp_milli / 1000.0
            except:
                continue
        
        # Calculate average temperature for main CPU temp metric
        if temps:
            temp_values = list(temps.values())
            temps['cpu_temp_c'] = sum(temp_values) / len(temp_values)
            
        return temps if temps else None
        
    def find_process(self):
        for proc in psutil.process_iter():
            try:
                if self.process_name in proc.name():
                    self.process = proc
                    self._safe_log('info', f"Monitoring process: {proc.name()} (PID: {proc.pid})")
                    break
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                continue
    
    def get_top_processes_by_memory(self, top_n=20):
        """Get top N processes by memory usage, including CPU usage"""
        processes = []
        try:
            # First pass: collect all processes with memory info (fast)
            for proc in psutil.process_iter():
                try:
                    mem_info = proc.memory_info()
                    if mem_info:
                        mem_mb = mem_info.rss / (1024**2)  # Convert to MB
                        processes.append({
                            'pid': proc.pid,
                            'name': proc.name(),
                            'memory_mb': mem_mb,
                            'proc': proc  # Keep reference for CPU calculation
                        })
                except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                    continue
            
            # Sort by memory usage (descending)
            processes.sort(key=lambda x: x['memory_mb'], reverse=True)
            
            # Get top N processes
            top_processes = processes[:top_n]
            
            # Second pass: get CPU usage only for top N processes (much faster)
            # Use cached CPU percent calculation for better performance
            for proc_info in top_processes:
                try:
                    proc = proc_info['proc']
                    pid = proc.pid
                    
                    # Use cached CPU percent calculation
                    # First call with interval=None returns 0.0, but sets up internal state
                    # Second call (if cached) or next call will return actual value
                    if pid not in self.process_cpu_cache:
                        # First time seeing this process, initialize
                        proc.cpu_percent(interval=None)
                        self.process_cpu_cache[pid] = None
                        cpu_percent = 0.0
                    else:
                        # Get CPU percent (non-blocking, uses cached calculation)
                        cpu_percent = proc.cpu_percent(interval=None)
                    
                    # Normalize CPU percent to 0-100% range (divide by number of cores)
                    # cpu_percent can exceed 100% on multi-core systems (e.g., 400% on 4 cores)
                    cpu_percent_normalized = cpu_percent / self.cpu_count if self.cpu_count > 0 else cpu_percent
                    proc_info['cpu_percent'] = cpu_percent_normalized
                    
                    # Store display name with PID to distinguish multiple instances of same process
                    proc_name = proc_info.get('name', '')
                    proc_pid = proc_info.get('pid', 0)
                    if proc_name:
                        proc_info['display_name'] = f"{proc_name} (PID:{proc_pid})"
                    elif proc_pid > 0:
                        proc_info['display_name'] = f"PID:{proc_pid}"
                    else:
                        proc_info['display_name'] = ''  # Empty if no process data
                except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                    proc_info['cpu_percent'] = 0.0
                    # Remove from cache if process no longer exists
                    self.process_cpu_cache.pop(pid, None)
                finally:
                    # Remove proc reference to avoid keeping process handles
                    proc_info.pop('proc', None)
            
            # Clean up cache for processes that no longer exist
            active_pids = {p['pid'] for p in top_processes}
            self.process_cpu_cache = {pid: val for pid, val in self.process_cpu_cache.items() if pid in active_pids}
            
            return top_processes
        except Exception as e:
            self._safe_log('warn', f'Error getting top processes: {e}')
            return []
    
    def get_system_metrics(self):
        metrics = {}
        
        try:
            metrics['cpu_percent'] = psutil.cpu_percent(interval=0.1)
            metrics['cpu_count'] = psutil.cpu_count()
            
            memory = psutil.virtual_memory()
            metrics['memory_percent'] = memory.percent
            metrics['memory_used_gb'] = memory.used / (1024**3)
            metrics['memory_available_gb'] = memory.available / (1024**3)
            
            disk = psutil.disk_usage('/')
            metrics['disk_percent'] = disk.percent
            metrics['disk_free_gb'] = disk.free / (1024**3)
            
            # Get top 20 processes by memory usage
            top_processes = self.get_top_processes_by_memory(20)
            # Ensure we always have exactly 20 processes (fill with empty if needed)
            for i in range(1, 21):
                if i <= len(top_processes):
                    proc = top_processes[i-1]
                    # Use display_name (with PID) if available, otherwise construct it
                    if 'display_name' in proc:
                        display_name = proc['display_name']
                    else:
                        proc_name = proc.get('name', '')
                        proc_pid = proc.get('pid', 0)
                        if proc_name:
                            display_name = f"{proc_name} (PID:{proc_pid})"
                        elif proc_pid > 0:
                            display_name = f"PID:{proc_pid}"
                        else:
                            display_name = ''  # Empty if no process data
                    metrics[f'top_process_{i}_name'] = display_name
                    metrics[f'top_process_{i}_pid'] = proc.get('pid', 0)
                    metrics[f'top_process_{i}_memory_mb'] = proc.get('memory_mb', 0.0)
                    metrics[f'top_process_{i}_cpu_percent'] = proc.get('cpu_percent', 0.0)
                else:
                    # Fill missing processes with empty values (not "unknown")
                    metrics[f'top_process_{i}_name'] = ''  # Empty string instead of 'unknown'
                    metrics[f'top_process_{i}_pid'] = 0
                    metrics[f'top_process_{i}_memory_mb'] = 0.0
                    metrics[f'top_process_{i}_cpu_percent'] = 0.0
            
            # Jetson temperature monitoring
            jetson_temps = self.get_jetson_temperatures()
            if jetson_temps:
                metrics.update(jetson_temps)
            else:
                # Fallback to psutil for non-Jetson systems
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
                except Exception:
                    pass
            
            if self.process and self.process.is_running():
                try:
                    metrics['process_cpu_percent'] = self.process.cpu_percent(interval=0.1)
                    metrics['process_memory_mb'] = self.process.memory_info().rss / (1024**2)
                    metrics['process_threads'] = self.process.num_threads()
                except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                    self.process = None
                    metrics['process_cpu_percent'] = 0.0
                    metrics['process_memory_mb'] = 0.0
                    metrics['process_threads'] = 0
        except Exception as e:
            self._safe_log('error', f'Error in get_system_metrics: {e}')
            # Return minimal metrics to prevent crash
            metrics['cpu_percent'] = 0.0
            metrics['memory_percent'] = 0.0
            metrics['cpu_temp_c'] = 0.0
            # Fill all top processes with zeros
            for i in range(1, 21):
                metrics[f'top_process_{i}_memory_mb'] = 0.0
                metrics[f'top_process_{i}_cpu_percent'] = 0.0
        
        return metrics
    
    def monitor_system(self):
        try:
            metrics = self.get_system_metrics()
        except Exception as e:
            self._safe_log('error', f'Error in get_system_metrics: {e}')
            return
        
        try:
            self.cpu_percent_history.append(metrics.get('cpu_percent', 0.0))
            self.memory_percent_history.append(metrics.get('memory_percent', 0.0))
            if len(self.cpu_percent_history) > 60:
                self.cpu_percent_history.pop(0)
            if len(self.memory_percent_history) > 60:
                self.memory_percent_history.pop(0)
            
            metrics['cpu_avg_1min'] = sum(self.cpu_percent_history) / len(self.cpu_percent_history)
            metrics['memory_avg_1min'] = sum(self.memory_percent_history) / len(self.memory_percent_history)
            
            system_msg = Float64MultiArray()
            # Ensure all values are float type
            base_data = [
                float(metrics.get('cpu_percent', 0.0)),
                float(metrics.get('memory_percent', 0.0)),
                float(metrics.get('cpu_avg_1min', 0.0)),
                float(metrics.get('memory_avg_1min', 0.0)),
                float(metrics.get('process_cpu_percent', 0.0)),
                float(metrics.get('process_memory_mb', 0.0)),
                float(metrics.get('cpu_temp_c', 0.0))
            ]
            
            # Add top 20 processes memory usage (MB) and CPU usage (%)
            # Ensure we always add exactly 40 values (20 processes * 2 metrics)
            # Also store process names in diagnostics for later retrieval
            process_names = []
            for i in range(1, 21):
                key_mem = f'top_process_{i}_memory_mb'
                key_cpu = f'top_process_{i}_cpu_percent'
                key_name = f'top_process_{i}_name'
                base_data.append(float(metrics.get(key_mem, 0.0)))
                base_data.append(float(metrics.get(key_cpu, 0.0)))
                # Store process name (empty string if not available)
                process_names.append(metrics.get(key_name, ''))
            
            # Add Jetson-specific temperatures if available
            if self.is_jetson:
                for zone in ['cpu', 'gpu', 'aux', 'ao', 'pmic', 'tboard', 'tdiode']:
                    key = f'jetson_{zone}_temp_c'
                    if key in metrics:
                        base_data.append(float(metrics[key]))
            
            system_msg.data = base_data
            
            # Log warning if data length is unexpected
            expected_min_len = 47  # 7 base + 20*2 (memory+CPU)
            if len(base_data) < expected_min_len:
                self._safe_log('warn', f'Unexpected data length: got {len(base_data)}, expected at least {expected_min_len}')
            
            self.system_pub.publish(system_msg)
            
            diag_array = DiagnosticArray()
            diag_array.header.stamp = self.get_clock().now().to_msg()
            
            status = DiagnosticStatus()
            status.name = "System Resources"
            status.level = DiagnosticStatus.OK
            
            cpu_percent = metrics.get('cpu_percent', 0.0)
            memory_percent = metrics.get('memory_percent', 0.0)
            
            if cpu_percent > 90:
                status.level = DiagnosticStatus.ERROR
                status.message = "CPU usage critical"
            elif cpu_percent > 70:
                status.level = DiagnosticStatus.WARN
                status.message = "High CPU usage"
            elif memory_percent > 90:
                status.level = DiagnosticStatus.ERROR
                status.message = "Memory usage critical"
            elif memory_percent > 70:
                status.level = DiagnosticStatus.WARN
                status.message = "High memory usage"
            else:
                status.message = "System resources normal"
            
            for key, value in metrics.items():
                try:
                    kv = KeyValue()
                    kv.key = str(key)
                    kv.value = f"{value:.2f}" if isinstance(value, (int, float)) else str(value)
                    status.values.append(kv)
                except Exception:
                    continue
            
            # Add process names as separate key-value pairs for easy retrieval
            for i, name in enumerate(process_names, 1):
                kv = KeyValue()
                kv.key = f'top_process_{i}_name'
                kv.value = name
                status.values.append(kv)
            
            diag_array.status.append(status)
            self.diagnostics_pub.publish(diag_array)
            
            self._safe_log('debug', f"CPU: {cpu_percent:.1f}%, Memory: {memory_percent:.1f}%, Temp: {metrics.get('cpu_temp_c', 0):.1f}?C")
        except Exception as e:
            self._safe_log('error', f'Error in monitor_system: {e}')
            # Don't raise exception to prevent process from dying
    
    def shutdown_callback(self, msg):
        """Handle shutdown signal"""
        self._safe_log('info', 'Received shutdown signal, stopping...')
        self.should_shutdown = True
        # Set flag to exit main loop
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    
    node = SystemMonitor()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.get_logger().info('Shutting down system_monitor...')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()