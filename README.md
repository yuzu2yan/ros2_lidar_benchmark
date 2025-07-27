# ROS 2 LiDAR Benchmark Package

A comprehensive performance benchmarking tool for LiDAR data processing in ROS 2, designed for NVIDIA Jetson and other platforms.

## Features

- **Real-time Performance Metrics**
  - Publishing frequency (Hz) with statistics
  - Jitter analysis (ms) for stability assessment
  - Bandwidth usage (Mbps) monitoring
  - Point cloud throughput (K points/sec)
  
- **System Resource Monitoring**
  - CPU and memory usage tracking
  - Process-specific resource monitoring
  - NVIDIA Jetson temperature monitoring (7 thermal zones)
  
- **Automated Analysis & Reporting**
  - Real-time visualization dashboard (6 graphs)
  - JSON format detailed reports
  - Excel reports with comprehensive analysis
  - Performance ratings and recommendations

## Requirements

- ROS 2 (Humble/Iron/Rolling)
- Python 3.8+
- Python packages:
  - psutil
  - matplotlib
  - numpy
  - pandas
  - openpyxl

## Installation

```bash
# Clone the repository
cd ~/ros2_ws/src
git clone <repository-url>

# Install dependencies
sudo apt update
sudo apt install python3-psutil python3-matplotlib python3-numpy
pip3 install pandas openpyxl

# Build the package
cd ~/ros2_ws
colcon build --packages-select ros2_lidar_benchmark
source install/setup.bash

# Or use the setup script
./setup_benchmark.sh
```

## Usage

### Quick Start

1. **Start tcpreplay** (in a separate terminal):
```bash
sudo tcpreplay -i eth0 -l 0 lidar_data.pcap
```

2. **Run the benchmark**:
```bash
# Default configuration (60 seconds, with visualization)
ros2 launch ros2_lidar_benchmark benchmark.launch.py

# Headless mode (no visualization)
ros2 launch ros2_lidar_benchmark benchmark_headless.launch.py

# Custom duration
ros2 launch ros2_lidar_benchmark benchmark.launch.py analysis_duration:=300.0
```

### Configuration

The package uses a YAML configuration file for easy customization:

```yaml
# config/benchmark_config.yaml
benchmark:
  analysis_duration: 60.0  # Measurement duration in seconds
  
topics:
  input_topic: "/lidar/points"      # Input topic from tcpreplay
  output_topic: "/benchmark/points" # Internal benchmark topic
```

#### Using Custom Configuration

```bash
# Use custom config file
ros2 launch ros2_lidar_benchmark benchmark.launch.py \
  config_file:=/path/to/custom_config.yaml

# Override specific parameters
ros2 launch ros2_lidar_benchmark benchmark.launch.py \
  input_topic:=/velodyne_points \
  analysis_duration:=120.0
```

## Metrics Analyzed

### LiDAR Performance Metrics
- **Frequency**: Message publishing rate (Hz)
- **Jitter**: Inter-message timing variation (ms)
- **Bandwidth**: Network bandwidth usage (Mbps)
- **Throughput**: 
  - Messages per second
  - Megabytes per second
  - Points per second (K points/sec)
  - Points per message

### System Resources
- **CPU Usage**: System-wide and per-process
- **Memory Usage**: RAM utilization
- **Temperature**: CPU, GPU, and other thermal zones (Jetson only)

### Performance Ratings
- **Performance Rating**: Excellent/Good/Fair/Poor based on frequency
- **Stability Rating**: Based on jitter measurements
- **Automated Recommendations**: Optimization suggestions

## Output Files

1. **Excel Report** (`/tmp/lidar_benchmark_report.xlsx`)
   - Summary sheet with overview and ratings
   - Detailed metrics sheet with all statistics
   - System resources sheet including temperatures
   - Raw data sheet with complete JSON data
   - Automatically generated after benchmark completion

2. **Visualization Plots** (`/tmp/lidar_benchmark/`)
   - PNG images of performance graphs
   - Saved when visualization is enabled

## Running Individual Nodes

```bash
# Point cloud receiver
ros2 run ros2_lidar_benchmark pointcloud_receiver.py

# Metrics collector
ros2 run ros2_lidar_benchmark metrics_collector.py

# System monitor
ros2 run ros2_lidar_benchmark system_monitor.py

# Visualizer
ros2 run ros2_lidar_benchmark visualizer.py

# Analyzer (runs for specified duration)
ros2 run ros2_lidar_benchmark benchmark_analyzer.py

# Generate Excel from existing JSON
ros2 run ros2_lidar_benchmark excel_report_generator.py \
  --json-file /tmp/lidar_benchmark_report.json
```

## Performance Targets

### Good Performance
- Frequency: > 10 Hz (stable)
- Jitter: < 25 ms
- CPU Usage: < 70%
- Temperature: < 70°C
- Throughput: > 80% of sensor specification

### Needs Improvement
- Frequency: < 5 Hz or unstable
- Jitter: > 50 ms
- CPU Usage: > 80%
- Temperature: > 80°C
- Throughput: < 50% of expected

## Troubleshooting

### No Point Cloud Data Received
- Verify tcpreplay is running correctly
- Check topic names: `ros2 topic list`
- Monitor data flow: `ros2 topic hz /vlp16/points_filtered`
- If data is visible with `ros2 topic echo` but not in the benchmark:
  - Run the diagnostic script: `python3 fix_data_reception.py`
  - Try different RMW implementation:
    ```bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ros2 launch ros2_lidar_benchmark benchmark.launch.py
    ```
  - Or use the RMW-aware launch file:
    ```bash
    ros2 launch ros2_lidar_benchmark benchmark_with_rmw.launch.py rmw_implementation:=rmw_cyclonedds_cpp
    ```

### Visualization Not Showing
- Check X11 forwarding for SSH connections
- Verify DISPLAY environment variable
- Use headless mode for remote systems

### High Resource Usage
- Reduce `window_size` parameter in config
- Decrease `analysis_duration`
- Use headless mode to save resources
