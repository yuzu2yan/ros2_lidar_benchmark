from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
import yaml


def generate_launch_description():
    """Launch long-term recorder with daily cumulative checkpoints."""

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

    # Load default values from config for topics
    config_path = os.path.join(
        FindPackageShare('ros2_lidar_benchmark').find('ros2_lidar_benchmark'),
        'config',
        'benchmark_config.yaml'
    )
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    default_input_topic = config['topics']['input_topic']
    default_output_topic = config['topics']['output_topic']

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

    duration_days_arg = DeclareLaunchArgument(
        'duration_days',
        default_value='14',
        description='Total recording duration in days'
    )

    checkpoint_days_arg = DeclareLaunchArgument(
        'checkpoint_interval_days',
        default_value='1',
        description='Interval in days for cumulative snapshots'
    )

    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/tmp/lidar_benchmark_longterm',
        description='Base output directory for long-term recorder'
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
        name='long_term_data_recorder',
        parameters=[{
            'output_dir': LaunchConfiguration('output_dir'),
            'duration_days': LaunchConfiguration('duration_days'),
            'checkpoint_interval_days': LaunchConfiguration('checkpoint_interval_days'),
            'save_tick_seconds': 30.0,
            'max_points': 0
        }],
        output='screen'
    )

    return LaunchDescription([
        config_file_arg,
        input_topic_arg,
        output_topic_arg,
        duration_days_arg,
        checkpoint_days_arg,
        output_dir_arg,
        metrics_collector_node,
        system_monitor_node,
        long_term_recorder_node,
    ])

