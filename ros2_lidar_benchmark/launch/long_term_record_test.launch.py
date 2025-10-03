from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
import yaml


def generate_launch_description():
    """Test: 5 minutes duration, snapshots every 1 minute."""

    pkg_share = FindPackageShare('ros2_lidar_benchmark')

    default_config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'benchmark_config.yaml'
    ])

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to the configuration file'
    )

    # Load default topics
    config_path = os.path.join(
        FindPackageShare('ros2_lidar_benchmark').find('ros2_lidar_benchmark'),
        'config',
        'benchmark_config.yaml'
    )
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    default_input_topic = config['topics']['input_topic']
    default_output_topic = config['topics']['output_topic']
    default_use_best_effort = config['topics'].get('use_best_effort_qos', True)

    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value=default_input_topic,
        description='Input point cloud topic'
    )

    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value=default_output_topic,
        description='Output point cloud topic for benchmarking'
    )

    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/tmp/lidar_benchmark_longterm_test',
        description='Base output directory for test run'
    )

    # Bridge input -> output like benchmark.launch.py
    pointcloud_receiver_node = Node(
        package='ros2_lidar_benchmark',
        executable='pointcloud_receiver.py',
        name='pointcloud_receiver',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'use_best_effort_qos': default_use_best_effort,
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

    long_term_recorder_node = Node(
        package='ros2_lidar_benchmark',
        executable='long_term_data_recorder.py',
        name='long_term_data_recorder_test',
        parameters=[{
            # Seconds-based test config: 300s total, checkpoint every 60s
            'duration_seconds': 300,
            'checkpoint_interval_seconds': 60,
            'save_tick_seconds': 10.0,
            'max_points': 0,
            'output_dir': LaunchConfiguration('output_dir'),
        }],
        output='screen'
    )

    return LaunchDescription([
        config_file_arg,
        input_topic_arg,
        output_topic_arg,
        output_dir_arg,
        pointcloud_receiver_node,
        metrics_collector_node,
        system_monitor_node,
        long_term_recorder_node,
    ])
