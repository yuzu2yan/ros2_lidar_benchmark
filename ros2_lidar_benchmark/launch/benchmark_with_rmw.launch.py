import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, EmitEvent, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.events import Shutdown
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_lidar_benchmark')
    config_file = os.path.join(pkg_share, 'config', 'benchmark_config.yaml')
    
    # Declare arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the benchmark configuration file'
    )
    
    analysis_duration_arg = DeclareLaunchArgument(
        'analysis_duration',
        default_value='60.0',
        description='Duration of analysis in seconds'
    )
    
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/vlp16/points_filtered',
        description='Input topic name'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/benchmark/points',
        description='Output topic name for benchmark'
    )
    
    show_viz_arg = DeclareLaunchArgument(
        'show_viz',
        default_value='true',
        description='Show visualization'
    )
    
    use_best_effort_arg = DeclareLaunchArgument(
        'use_best_effort_qos',
        default_value='true',
        description='Use best effort QoS for sensor data'
    )
    
    rmw_implementation_arg = DeclareLaunchArgument(
        'rmw_implementation',
        default_value='rmw_cyclonedds_cpp',
        description='RMW implementation to use (rmw_cyclonedds_cpp or rmw_fastrtps_cpp)'
    )
    
    # Set RMW implementation
    rmw_impl = LaunchConfiguration('rmw_implementation')
    
    # Common parameters
    common_params = {
        'config_file': LaunchConfiguration('config_file'),
        'input_topic': LaunchConfiguration('input_topic'),
        'output_topic': LaunchConfiguration('output_topic'),
        'use_best_effort_qos': LaunchConfiguration('use_best_effort_qos')
    }
    
    # Node configurations with environment variable
    env_vars = {'RMW_IMPLEMENTATION': LaunchConfiguration('rmw_implementation')}
    
    pointcloud_receiver = Node(
        package='ros2_lidar_benchmark',
        executable='pointcloud_receiver.py',
        name='pointcloud_receiver',
        parameters=[common_params],
        output='screen',
        additional_env=env_vars
    )
    
    metrics_collector = Node(
        package='ros2_lidar_benchmark',
        executable='metrics_collector.py',
        name='metrics_collector',
        parameters=[{
            'topic_to_monitor': LaunchConfiguration('output_topic'),
            'use_best_effort_qos': LaunchConfiguration('use_best_effort_qos')
        }],
        output='screen',
        additional_env=env_vars
    )
    
    system_monitor = Node(
        package='ros2_lidar_benchmark',
        executable='system_monitor.py',
        name='system_monitor',
        output='screen',
        additional_env=env_vars
    )
    
    visualizer = Node(
        package='ros2_lidar_benchmark',
        executable='visualizer.py',
        name='visualizer',
        condition=IfCondition(LaunchConfiguration('show_viz')),
        output='screen',
        additional_env=env_vars
    )
    
    benchmark_analyzer = Node(
        package='ros2_lidar_benchmark',
        executable='benchmark_analyzer.py',
        name='benchmark_analyzer',
        parameters=[{
            'analysis_duration': LaunchConfiguration('analysis_duration'),
            'config_file': LaunchConfiguration('config_file')
        }],
        output='screen',
        additional_env=env_vars
    )
    
    # Print RMW info at startup
    print_rmw_info = ExecuteProcess(
        cmd=['echo', 'Using RMW_IMPLEMENTATION:', LaunchConfiguration('rmw_implementation')],
        output='screen'
    )
    
    # Delayed shutdown based on analysis duration
    delayed_shutdown = TimerAction(
        period=LaunchConfiguration('analysis_duration'),
        actions=[
            EmitEvent(event=Shutdown(reason='Analysis completed'))
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        analysis_duration_arg,
        input_topic_arg,
        output_topic_arg,
        show_viz_arg,
        use_best_effort_arg,
        rmw_implementation_arg,
        print_rmw_info,
        pointcloud_receiver,
        metrics_collector,
        system_monitor,
        visualizer,
        benchmark_analyzer,
        delayed_shutdown
    ])