#!/usr/bin/env python3

import json
import os
import numpy as np

# Create test data
def create_test_data():
    """Create test visualization data"""
    duration = 60  # 60 seconds
    samples = 300  # 5 Hz sampling
    
    timestamps = np.linspace(0, duration, samples)
    
    # Generate synthetic data with some variation
    hz_base = 10.0
    hz_data = hz_base + np.sin(timestamps * 0.1) * 2 + np.random.normal(0, 0.5, samples)
    
    jitter_base = 5.0
    jitter_data = jitter_base + np.sin(timestamps * 0.2) * 2 + np.random.normal(0, 0.3, samples)
    
    bandwidth_base = 50.0
    bandwidth_data = bandwidth_base + np.sin(timestamps * 0.15) * 10 + np.random.normal(0, 2, samples)
    
    throughput_base = 100.0
    throughput_data = throughput_base + np.sin(timestamps * 0.1) * 20 + np.random.normal(0, 5, samples)
    
    cpu_base = 30.0
    cpu_data = cpu_base + np.sin(timestamps * 0.05) * 10 + np.random.normal(0, 2, samples)
    
    memory_base = 40.0
    memory_data = memory_base + np.sin(timestamps * 0.08) * 5 + np.random.normal(0, 1, samples)
    
    temp_base = 45.0
    temp_data = temp_base + timestamps * 0.1 + np.random.normal(0, 0.5, samples)  # Gradual increase
    
    viz_data = {
        'timestamps': timestamps.tolist(),
        'hz': hz_data.tolist(),
        'jitter': jitter_data.tolist(),
        'bandwidth': bandwidth_data.tolist(),
        'throughput': throughput_data.tolist(),
        'cpu': cpu_data.tolist(),
        'memory': memory_data.tolist(),
        'temperature': temp_data.tolist()
    }
    
    # Create test benchmark data
    benchmark_data = {
        'summary': {
            'analysis_duration': duration,
            'performance_rating': 'Good',
            'stability_rating': 'Good'
        },
        'lidar_metrics': {
            'average_hz': float(np.mean(hz_data)),
            'min_hz': float(np.min(hz_data)),
            'max_hz': float(np.max(hz_data)),
            'average_jitter_ms': float(np.mean(jitter_data)),
            'min_jitter_ms': float(np.min(jitter_data)),
            'max_jitter_ms': float(np.max(jitter_data)),
            'average_bandwidth_mbps': float(np.mean(bandwidth_data)),
            'min_bandwidth_mbps': float(np.min(bandwidth_data)),
            'max_bandwidth_mbps': float(np.max(bandwidth_data)),
            'total_messages': len(timestamps) * 10,
            'total_mb': float(np.sum(bandwidth_data) * duration / 8)
        },
        'system_resources': {
            'average_cpu_percent': float(np.mean(cpu_data)),
            'average_memory_percent': float(np.mean(memory_data)),
            'max_temperature_c': float(np.max(temp_data))
        },
        'analysis': {
            'performance_rating': 'Good',
            'stability_rating': 'Good',
            'recommendations': [
                'Performance is within acceptable range',
                'Consider monitoring temperature during extended runs',
                'Network bandwidth utilization is optimal'
            ]
        }
    }
    
    return viz_data, benchmark_data

def main():
    print("Creating test data...")
    viz_data, benchmark_data = create_test_data()
    
    # Save test data
    test_dir = '/tmp/test_excel_graphs'
    os.makedirs(test_dir, exist_ok=True)
    
    viz_file = os.path.join(test_dir, 'visualization_data.json')
    bench_file = os.path.join(test_dir, 'benchmark_report.json')
    
    with open(viz_file, 'w') as f:
        json.dump(viz_data, f, indent=2)
    print(f"Saved visualization data to: {viz_file}")
    
    with open(bench_file, 'w') as f:
        json.dump(benchmark_data, f, indent=2)
    print(f"Saved benchmark data to: {bench_file}")
    
    # Generate Excel report
    print("\nGenerating Excel report with graphs...")
    output_file = os.path.join(test_dir, 'test_report_with_graphs.xlsx')
    
    import sys
    sys.path.append('/Users/yuzu/ros2_lidar_benchmark/ros2_lidar_benchmark/scripts')
    
    from excel_report_enhanced import EnhancedExcelReport
    
    generator = EnhancedExcelReport(bench_file, viz_file)
    generator.generate_excel_with_graphs(output_file)
    
    print(f"\nTest complete! Check the output at: {output_file}")

if __name__ == '__main__':
    main()