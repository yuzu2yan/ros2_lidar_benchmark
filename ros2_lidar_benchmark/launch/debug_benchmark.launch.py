from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo
import os
import yaml
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Read config file
    config_path = os.path.join(
        FindPackageShare('ros2_lidar_benchmark').find('ros2_lidar_benchmark'),
        'config',
        'benchmark_config.yaml'
    )
    
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    input_topic = config['topics']['input_topic']
    output_topic = config['topics']['output_topic']
    use_best_effort = config['topics'].get('use_best_effort_qos', True)
    
    return LaunchDescription([
        LogInfo(msg=f"Starting benchmark with input_topic={input_topic}, output_topic={output_topic}"),
        
        # Pointcloud receiver only
        Node(
            package='ros2_lidar_benchmark',
            executable='pointcloud_receiver.py',
            name='pointcloud_receiver',
            parameters=[{
                'input_topic': input_topic,
                'output_topic': output_topic,
                'use_best_effort_qos': use_best_effort
            }],
            output='screen',
            emulate_tty=True
        ),
        
        # Metrics collector only
        Node(
            package='ros2_lidar_benchmark',
            executable='metrics_collector.py',
            name='metrics_collector',
            parameters=[{
                'topic_to_monitor': output_topic,
                'window_size': 100,
                'publish_rate': 2.0
            }],
            output='screen',
            emulate_tty=True
        )
    ])