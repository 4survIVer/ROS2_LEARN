cmake_minimum_required(VERSION 3.8)
project(dds_visualizer)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Include directories
include_directories(include)

# Executable 1: Minimal DDS Monitor
add_executable(minimal_dds_monitor 
  src/minimal_dds_monitor.cpp
)
ament_target_dependencies(minimal_dds_monitor
  rclcpp
  std_msgs
)

# Executable 2: Main DDS Monitor
add_executable(dds_discovery_monitor 
  src/dds_discovery_monitor.cpp
  src/dds_utils.cpp
)
ament_target_dependencies(dds_discovery_monitor
  rclcpp
  std_msgs
  visualization_msgs
)

# Executable 3: QoS Demo
add_executable(qos_demo 
  src/qos_demo.cpp
)
ament_target_dependencies(qos_demo
  rclcpp
  std_msgs
)

# Install all executables
install(TARGETS
  minimal_dds_monitor
  dds_discovery_monitor
  qos_demo
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

