#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Empty
import json
import os
import time
import threading
from datetime import datetime, timezone


class LongTermDataRecorder(Node):
    """Records benchmark data continuously and writes daily cumulative snapshots"""

    def __init__(self):
        super().__init__('long_term_data_recorder')

        # Parameters
        self.declare_parameter('output_dir', '/tmp/lidar_benchmark_longterm')
        self.declare_parameter('duration_days', 14)
        self.declare_parameter('checkpoint_interval_days', 1)
        # Optional seconds-based overrides (for tests)
        self.declare_parameter('duration_seconds', 0)
        self.declare_parameter('checkpoint_interval_seconds', 0)
        self.declare_parameter('save_tick_seconds', 30.0)  # periodic current save
        self.declare_parameter('max_points', 0)  # 0 or negative => unlimited

        self.output_dir = self.get_parameter('output_dir').value
        self.duration_days = float(self.get_parameter('duration_days').value)
        self.checkpoint_interval_days = float(self.get_parameter('checkpoint_interval_days').value)
        self.duration_seconds = float(self.get_parameter('duration_seconds').value)
        self.checkpoint_interval_seconds = float(self.get_parameter('checkpoint_interval_seconds').value)
        self.save_tick_seconds = float(self.get_parameter('save_tick_seconds').value)
        self.max_points = int(self.get_parameter('max_points').value)

        os.makedirs(self.output_dir, exist_ok=True)
        self.snapshots_dir = os.path.join(self.output_dir, 'daily_snapshots')
        os.makedirs(self.snapshots_dir, exist_ok=True)

        # Data storage
        self.data = {
            'timestamps': [],
            'hz': [],
            'jitter': [],
            'bandwidth': [],
            'throughput': [],
            'cpu': [],
            'memory': [],
            'temperature': []
        }

        self.start_walltime = time.time()
        self.start_iso = datetime.now(timezone.utc).astimezone().strftime('%Y%m%d_%H%M%S')
        self.data_lock = threading.Lock()

        # Subscribers
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

        # Optional external shutdown
        self.shutdown_sub = self.create_subscription(
            Empty,
            '/benchmark/shutdown',
            self.shutdown_callback,
            10
        )

        # Timers
        self.periodic_save_timer = self.create_timer(self.save_tick_seconds, self.save_current)
        self.progress_check_timer = self.create_timer(1.0, self.progress_check)

        # Checkpoint schedule (prefer seconds params when > 0)
        if self.duration_seconds and self.duration_seconds > 0:
            self.total_duration_sec = self.duration_seconds
        else:
            self.total_duration_sec = self.duration_days * 24 * 3600

        if self.checkpoint_interval_seconds and self.checkpoint_interval_seconds > 0:
            self.checkpoint_interval_sec = self.checkpoint_interval_seconds
        else:
            self.checkpoint_interval_sec = self.checkpoint_interval_days * 24 * 3600

        # Initialize next checkpoint time
        self.next_checkpoint_sec = self.checkpoint_interval_sec
        self.checkpoint_count = 0

        self.get_logger().info(
            f'Long-term recorder started. Output: {self.output_dir}, duration_days={self.duration_days}, interval_days={self.checkpoint_interval_days}'
        )

    def _trim_if_needed(self):
        if self.max_points and self.max_points > 0:
            for key in ['timestamps', 'hz', 'jitter', 'bandwidth', 'throughput', 'cpu', 'memory', 'temperature']:
                if len(self.data[key]) > self.max_points:
                    # Keep the most recent max_points
                    self.data[key] = self.data[key][-self.max_points:]

    def metrics_callback(self, msg):
        with self.data_lock:
            elapsed = time.time() - self.start_walltime
            self.data['timestamps'].append(elapsed)

            if len(msg.data) >= 9:
                self.data['hz'].append(msg.data[0])
                self.data['jitter'].append(msg.data[1])
                self.data['bandwidth'].append(msg.data[2])
                self.data['throughput'].append(msg.data[8])  # kpoints_per_second
            else:
                # Fill to keep array lengths consistent
                self.data['hz'].append(float('nan'))
                self.data['jitter'].append(float('nan'))
                self.data['bandwidth'].append(float('nan'))
                self.data['throughput'].append(float('nan'))

            self._trim_if_needed()

    def system_callback(self, msg):
        with self.data_lock:
            if len(msg.data) >= 7:
                self.data['cpu'].append(msg.data[0])
                self.data['memory'].append(msg.data[1])
                self.data['temperature'].append(msg.data[6])
            else:
                self.data['cpu'].append(float('nan'))
                self.data['memory'].append(float('nan'))
                self.data['temperature'].append(float('nan'))

            self._trim_if_needed()

    def save_current(self):
        """Periodically save the current rolling data for live visualization."""
        with self.data_lock:
            if len(self.data['timestamps']) == 0:
                return
            out_file = os.path.join(self.output_dir, 'visualization_data.json')
            try:
                with open(out_file, 'w') as f:
                    json.dump(self.data, f, indent=2)
                self.get_logger().debug(
                    f'Saved current data points: {len(self.data["timestamps"])} -> {out_file}'
                )
            except Exception as e:
                self.get_logger().error(f'Failed to save current data: {e}')

    def _safe_stats(self, arr):
        import math
        vals = [v for v in arr if isinstance(v, (int, float)) and not (isinstance(v, float) and math.isnan(v))]
        if not vals:
            return 0.0, 0.0, 0.0, 0.0
        avg = sum(vals) / len(vals)
        mn = min(vals)
        mx = max(vals)
        # std
        var = sum((x - avg) ** 2 for x in vals) / len(vals)
        std = var ** 0.5
        return avg, mn, mx, std

    def _ratings_and_recs(self, avg_hz: float, avg_jitter_ms: float, avg_bw: float, cpu_avg: float, mem_avg: float):
        # Performance rating (aligned with benchmark_analyzer)
        if avg_hz >= 20:
            perf = 'Excellent'
        elif avg_hz >= 10:
            perf = 'Good'
        elif avg_hz >= 5:
            perf = 'Fair'
        else:
            perf = 'Poor'

        # Stability rating
        if avg_jitter_ms <= 10:
            stab = 'Excellent'
        elif avg_jitter_ms <= 25:
            stab = 'Good'
        elif avg_jitter_ms <= 50:
            stab = 'Fair'
        else:
            stab = 'Poor'

        recs = []
        if avg_hz < 10:
            recs.append('Consider optimizing point cloud processing to improve frequency')
        if avg_jitter_ms > 50:
            recs.append('High jitter detected - check for system load or network issues')
        if avg_bw > 100:
            recs.append('High bandwidth usage - consider data compression or reduced resolution')
        if cpu_avg > 80:
            recs.append('High CPU usage - consider performance optimization or hardware upgrade')
        if mem_avg > 80:
            recs.append('High memory usage - check for memory leaks or increase system RAM')
        if not recs:
            recs = ['System performing within normal parameters']
        return perf, stab, recs

    def _build_report_json(self, duration_seconds: float):
        # Compute metrics from cumulative data
        avg_hz, min_hz, max_hz, std_hz = self._safe_stats(self.data['hz'])
        avg_j, min_j, max_j, std_j = self._safe_stats(self.data['jitter'])
        avg_bw, min_bw, max_bw, _ = self._safe_stats(self.data['bandwidth'])
        avg_tp, _, max_tp, _ = self._safe_stats(self.data['throughput'])
        avg_cpu, min_cpu, max_cpu, _ = self._safe_stats(self.data['cpu'])
        avg_mem, min_mem, max_mem, _ = self._safe_stats(self.data['memory'])
        avg_temp, min_temp, max_temp, _ = self._safe_stats(self.data['temperature'])

        perf, stab, recs = self._ratings_and_recs(avg_hz, avg_j, avg_bw, avg_cpu, avg_mem)

        report = {
            'start_time': datetime.fromtimestamp(self.start_walltime).isoformat(),
            'end_time': datetime.now().isoformat(),
            'duration_seconds': duration_seconds,
            'metrics': {
                'frequency': {
                    'average_hz': avg_hz,
                    'min_hz': min_hz,
                    'max_hz': max_hz,
                    'std_hz': std_hz
                },
                'jitter': {
                    'average_ms': avg_j,
                    'min_ms': min_j,
                    'max_ms': max_j,
                    'std_ms': std_j
                },
                'bandwidth': {
                    'average_mbps': avg_bw,
                    'min_mbps': min_bw,
                    'max_mbps': max_bw
                },
                'throughput': {
                    'average_kpoints_per_sec': avg_tp,
                    'max_kpoints_per_sec': max_tp,
                    # The analyzer stores additional fields, keep minimal useful set
                },
                'samples': len(self.data['timestamps'])
            },
            'system_stats': {
                'cpu': {
                    'average_percent': avg_cpu,
                    'min_percent': min_cpu,
                    'max_percent': max_cpu
                },
                'memory': {
                    'average_percent': avg_mem,
                    'min_percent': min_mem,
                    'max_percent': max_mem
                },
                'temperature': {
                    'average_celsius': avg_temp,
                    'min_celsius': min_temp,
                    'max_celsius': max_temp
                },
                'samples': len(self.data['timestamps'])
            },
            'summary': {
                'performance_rating': perf,
                'stability_rating': stab,
                'recommendations': recs
            }
        }
        return report

    def save_checkpoint(self, day_index: int, final: bool = False):
        """Save cumulative snapshot outputs: Excel and graphs with unique names (no overwrite)."""
        with self.data_lock:
            if len(self.data['timestamps']) == 0:
                return

            tag = 'final' if final else f'day{day_index:02d}'
            ts = datetime.now(timezone.utc).astimezone().strftime('%Y%m%d_%H%M%S')

            # Create a dedicated snapshot directory
            snapshot_dir_name = f'snapshot_{self.start_iso}_until_{ts}_{tag}'
            snapshot_dir = os.path.join(self.snapshots_dir, snapshot_dir_name)
            os.makedirs(snapshot_dir, exist_ok=True)

            # Prepare report json for Excel/graphs
            try:
                duration_seconds = self.data['timestamps'][-1] if self.data['timestamps'] else 0.0
                report = self._build_report_json(duration_seconds)

                # Write temp JSON for generators
                tmp_json = os.path.join(snapshot_dir, f'report_{tag}.json')
                with open(tmp_json, 'w') as f:
                    json.dump(report, f, indent=2)

                # Excel generation
                try:
                    try:
                        from excel_report_generator import ExcelReportGenerator
                    except ImportError:
                        import sys
                        script_dir = os.path.dirname(os.path.abspath(__file__))
                        sys.path.insert(0, script_dir)
                        from excel_report_generator import ExcelReportGenerator

                    excel_name = f'report_{self.start_iso}_until_{ts}_{tag}.xlsx'
                    excel_path = os.path.join(snapshot_dir, excel_name)
                    gen = ExcelReportGenerator(tmp_json, excel_path)
                    gen.generate()
                except Exception as e:
                    self.get_logger().error(f'Excel generation failed: {e}')

                # Graphs generation
                try:
                    try:
                        from graph_generator import GraphGenerator
                    except ImportError:
                        import sys
                        script_dir = os.path.dirname(os.path.abspath(__file__))
                        sys.path.insert(0, script_dir)
                        from graph_generator import GraphGenerator

                    graphs_base = os.path.join(snapshot_dir, 'graphs')
                    os.makedirs(graphs_base, exist_ok=True)
                    viz_file = os.path.join(self.output_dir, 'visualization_data.json')
                    graph_gen = GraphGenerator(tmp_json, viz_file if os.path.exists(viz_file) else None, graphs_base)
                    graph_gen.generate_all_graphs()
                except Exception as e:
                    self.get_logger().error(f'Graph generation failed: {e}')

                # Remove temp JSON to comply with "Excel instead of JSON"
                try:
                    os.remove(tmp_json)
                except Exception:
                    pass

                self.get_logger().info(
                    f'Checkpoint outputs saved ({"final" if final else f"{day_index}d"}): {snapshot_dir}'
                )
            except Exception as e:
                self.get_logger().error(f'Failed to save checkpoint outputs: {e}')

    def progress_check(self):
        """Check elapsed time to trigger checkpoints and finalization."""
        elapsed = time.time() - self.start_walltime

        # Daily checkpoint
        if elapsed >= self.next_checkpoint_sec:
            self.checkpoint_count += 1
            self.save_checkpoint(self.checkpoint_count, final=False)
            self.next_checkpoint_sec += self.checkpoint_interval_sec

        # Finalization
        if elapsed >= self.total_duration_sec:
            # Save final snapshot and stop
            self.save_checkpoint(self.checkpoint_count, final=True)
            self.get_logger().info('Target duration reached. Stopping long-term recorder.')
            raise SystemExit

    def shutdown_callback(self, _msg: Empty):
        self.get_logger().info('Received shutdown signal. Saving final snapshot and exiting...')
        # Save final snapshot and exit
        try:
            self.save_checkpoint(self.checkpoint_count, final=True)
        finally:
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = LongTermDataRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.get_logger().info('Shutting down long_term_data_recorder...')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
