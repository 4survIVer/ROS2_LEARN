# Minimum CMake version required
cmake_minimum_required(VERSION 3.8)

# Project name and version
project(cpp_oops_package)

# Check compiler compatibility
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# =============================================
# EXAMPLE 1: Class Basics with Publisher
# =============================================
add_executable(class_basics_example 
  src/fundamentals/01_class_basics.cpp
)

# Link ROS2 libraries to the executable
ament_target_dependencies(class_basics_example
  rclcpp
  std_msgs
)

# =============================================
# Install the executables
# =============================================
install(TARGETS
  class_basics_example
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME/launch}
)

# Export dependencies for other packages
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs  
  sensor_msgs
)

# Finalize package configuration
ament_package()
