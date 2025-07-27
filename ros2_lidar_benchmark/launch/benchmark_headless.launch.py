from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
import yaml


def generate_launch_description():
    """Launch file for headless benchmark (no visualization)"""
    
    # Find package share directory
    pkg_share = FindPackageShare('ros2_lidar_benchmark')
    
    # Default config file path
    default_config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'benchmark_config.yaml'
    ])
    
    # Declare config file argument
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to the configuration file'
    )
    
    # Load configuration
    config_file = LaunchConfiguration('config_file')
    
    # Read config file (for default values)
    config_path = os.path.join(
        FindPackageShare('ros2_lidar_benchmark').find('ros2_lidar_benchmark'),
        'config',
        'benchmark_config.yaml'
    )
    
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    # Extract default values from config
    default_input_topic = config['topics']['input_topic']
    default_output_topic = config['topics']['output_topic']
    default_analysis_duration = str(config['benchmark']['analysis_duration'])
    default_report_file = config['benchmark']['report_file']
    
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value=default_input_topic,
        description='Input point cloud topic from tcpreplay'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value=default_output_topic,
        description='Output point cloud topic for benchmarking'
    )
    
    analysis_duration_arg = DeclareLaunchArgument(
        'analysis_duration',
        default_value=default_analysis_duration,
        description='Duration for benchmark analysis in seconds'
    )
    
    report_file_arg = DeclareLaunchArgument(
        'report_file',
        default_value=default_report_file,
        description='Output file for benchmark report'
    )
    
    pointcloud_receiver_node = Node(
        package='ros2_lidar_benchmark',
        executable='pointcloud_receiver.py',
        name='pointcloud_receiver',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic')
        }],
        output='screen'
    )
    
    metrics_collector_node = Node(
        package='ros2_lidar_benchmark',
        executable='metrics_collector.py',
        name='metrics_collector',
        parameters=[{
            'topic_to_monitor': LaunchConfiguration('output_topic'),
            'window_size': 100,
            'publish_rate': 2.0
        }],
        output='screen'
    )
    
    system_monitor_node = Node(
        package='ros2_lidar_benchmark',
        executable='system_monitor.py',
        name='system_monitor',
        parameters=[{
            'monitor_rate': 1.0,
            'process_name': ''
        }],
        output='screen'
    )
    
    analyzer_node = Node(
        package='ros2_lidar_benchmark',
        executable='benchmark_analyzer.py',
        name='benchmark_analyzer',
        parameters=[{
            'output_file': LaunchConfiguration('report_file'),
            'analysis_duration': LaunchConfiguration('analysis_duration')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        config_file_arg,
        input_topic_arg,
        output_topic_arg,
        analysis_duration_arg,
        report_file_arg,
        pointcloud_receiver_node,
        metrics_collector_node,
        system_monitor_node,
        analyzer_node
    ])