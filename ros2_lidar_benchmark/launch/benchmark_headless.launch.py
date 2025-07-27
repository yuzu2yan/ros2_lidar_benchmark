from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch file for headless benchmark (no visualization)"""
    
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/lidar/points',
        description='Input point cloud topic from tcpreplay'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/benchmark/points',
        description='Output point cloud topic for benchmarking'
    )
    
    analysis_duration_arg = DeclareLaunchArgument(
        'analysis_duration',
        default_value='60.0',
        description='Duration for benchmark analysis in seconds'
    )
    
    report_file_arg = DeclareLaunchArgument(
        'report_file',
        default_value='/tmp/lidar_benchmark_report.json',
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
        input_topic_arg,
        output_topic_arg,
        analysis_duration_arg,
        report_file_arg,
        pointcloud_receiver_node,
        metrics_collector_node,
        system_monitor_node,
        analyzer_node
    ])