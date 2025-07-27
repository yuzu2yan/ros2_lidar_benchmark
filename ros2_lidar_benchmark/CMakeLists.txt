cmake_minimum_required(VERSION 3.8)
project(ros2_lidar_benchmark)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(message_filters REQUIRED)

install(DIRECTORY
  launch
  config
  scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/pointcloud_receiver.py
  scripts/metrics_collector.py
  scripts/system_monitor.py
  scripts/visualizer.py
  scripts/benchmark_analyzer.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()