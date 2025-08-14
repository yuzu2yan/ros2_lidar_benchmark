# ROS 2 LiDAR Benchmark Package

A comprehensive performance benchmarking tool for LiDAR data processing in ROS 2, designed for NVIDIA Jetson Orin Nano Super and other platforms. This package provides automated performance analysis with real-time visualization and detailed Excel reporting.

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
  - Real-time visualization dashboard (optional, default OFF)
  - Excel reports with comprehensive analysis
  - Separate graph folder with 10 different visualizations
  - Performance ratings and recommendations
  - Automatic termination after analysis duration

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

# Or use the setup script (if available)
# ./setup_benchmark.sh
```

## Usage

### Quick Start

1. **Output point cloud data as ROS2 topics** (in a separate terminal):

2. **Check for any running benchmark nodes from previous sessions**:
```bash
# Check if any benchmark nodes are still running
ros2 node list | grep -E "(pointcloud_receiver|metrics_collector|system_monitor|benchmark)"

# If nodes are found, clean them up before starting
pkill -f pointcloud_receiver
pkill -f metrics_collector
pkill -f system_monitor
pkill -f benchmark_analyzer

# Or restart the ROS2 daemon
ros2 daemon stop
ros2 daemon start
```

3. **Run the benchmark**:
```bash
# Default configuration (5 minutes, no visualization)
ros2 launch ros2_lidar_benchmark benchmark.launch.py

# With visualization enabled
ros2 launch ros2_lidar_benchmark benchmark.launch.py enable_visualization:=true

# Custom duration (e.g., 2 minutes)
ros2 launch ros2_lidar_benchmark benchmark.launch.py analysis_duration:=120.0
```

### Configuration

The package uses a YAML configuration file for easy customization:

```yaml
# config/benchmark_config.yaml
benchmark:
  analysis_duration: 300.0          # Measurement duration (default: 5 minutes)
  enable_visualization: false       # Real-time visualization (default: OFF)
  report_file: "/tmp/lidar_benchmark_report.xlsx"
  graph_output_dir: "/tmp/lidar_benchmark_graphs"
  
topics:
  input_topic: "/vlp16/points_filtered"  # Input topic (e.g., from r2r_multi_lidar_filter)
  output_topic: "/benchmark/points"      # Internal benchmark topic
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

2. **Graph Folder** (`/tmp/lidar_benchmark_graphs/benchmark_YYYYMMDD_HHMMSS/`)
   - 10 different PNG graphs:
     - Frequency over time
     - Jitter analysis
     - Bandwidth usage
     - System resources
     - Temperature monitoring
     - Throughput metrics
     - Distribution histograms
     - Combined overview
   - Metadata.json with graph generation details

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

### Node Cleanup Issues

If you experience increasing Hz rates or duplicate messages between runs:

1. **Check for leftover nodes:**
```bash
ros2 node list | grep -E "(pointcloud_receiver|metrics_collector|system_monitor|benchmark)"
```

2. **Clean up any remaining nodes:**
```bash
# Kill specific benchmark nodes
pkill -f pointcloud_receiver
pkill -f metrics_collector  
pkill -f system_monitor
pkill -f benchmark_analyzer

# Or restart ROS2 daemon completely
ros2 daemon stop
ros2 daemon start
```

3. **Verify cleanup:**
```bash
# Should show no benchmark-related nodes
ros2 node list
```

**Note:** The benchmark now includes automatic shutdown signals to all nodes when analysis completes, but manual cleanup may still be needed if the launch is interrupted (Ctrl+C) or encounters errors.

## Troubleshooting

### No Point Cloud Data Received
- Verify tcpreplay is running correctly
- Check topic names: `ros2 topic list`
- Monitor data flow: `ros2 topic hz /vlp16/points_filtered`
- Check QoS settings - the package uses sensor_data QoS profile (BEST_EFFORT reliability)

### Visualization Not Showing
- Visualization is OFF by default - enable with `enable_visualization:=true`
- Check X11 forwarding for SSH connections
- Verify DISPLAY environment variable
- Graphs are always saved to folder regardless of visualization setting

### High Resource Usage
- Reduce `window_size` parameter in config
- Decrease `analysis_duration`
- Keep visualization disabled (default) to save resources

### Excel Report Issues
- Ensure pandas and openpyxl are installed: `pip3 install pandas openpyxl`
- Check write permissions for output directory
- Report is generated automatically after analysis completes
