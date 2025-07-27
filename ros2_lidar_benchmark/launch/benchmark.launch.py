from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
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
    
    enable_viz_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable real-time visualization'
    )
    
    analysis_duration_arg = DeclareLaunchArgument(
        'analysis_duration',
        default_value='60.0',
        description='Duration for benchmark analysis in seconds'
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
    
    visualizer_node = Node(
        package='ros2_lidar_benchmark',
        executable='visualizer.py',
        name='benchmark_visualizer',
        condition=IfCondition(LaunchConfiguration('enable_visualization')),
        parameters=[{
            'window_seconds': 60,
            'update_rate': 2.0,
            'save_plots': True,
            'output_dir': '/tmp/lidar_benchmark'
        }],
        output='screen'
    )
    
    analyzer_node = Node(
        package='ros2_lidar_benchmark',
        executable='benchmark_analyzer.py',
        name='benchmark_analyzer',
        parameters=[{
            'output_file': '/tmp/lidar_benchmark_report.json',
            'analysis_duration': LaunchConfiguration('analysis_duration')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        input_topic_arg,
        output_topic_arg,
        enable_viz_arg,
        analysis_duration_arg,
        pointcloud_receiver_node,
        metrics_collector_node,
        system_monitor_node,
        visualizer_node,
        analyzer_node
    ])