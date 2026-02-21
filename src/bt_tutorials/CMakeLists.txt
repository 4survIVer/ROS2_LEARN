cmake_minimum_required(VERSION 3.8)
project(bt_tutorials)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

# Add include directories
include_directories(
  include
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
)

# ============================================
# Example 1: Basic Nodes
# ============================================
add_executable(example1_basic_nodes
  src/example1_basic_nodes.cpp
)
ament_target_dependencies(example1_basic_nodes
  rclcpp
  std_msgs
  behaviortree_cpp_v3
)

# ============================================
# Example 2: Control Flow (ISOLATED)
# ============================================
add_executable(example2_control_flow
  src/example2_control_flow.cpp
)
ament_target_dependencies(example2_control_flow
  rclcpp
  std_msgs
  behaviortree_cpp_v3
)

# ============================================
# Example 3: Decorators (ISOLATED)
# ============================================
add_executable(example3_decorators
  src/example3_decorators.cpp
)
ament_target_dependencies(example3_decorators
  rclcpp
  std_msgs
  behaviortree_cpp_v3
)

# ============================================
# Example 4: Blackboard and Ports
# ============================================
add_executable(example4_blackboard_ports
  src/example4_blackboard_ports.cpp
)
ament_target_dependencies(example4_blackboard_ports
  rclcpp
  std_msgs
  behaviortree_cpp_v3
)

# ============================================
# Example 5: SubTrees and XML Files
# ============================================
add_executable(example5_subtrees
  src/example5_subtrees.cpp
)
ament_target_dependencies(example5_subtrees
  rclcpp
  std_msgs
  behaviortree_cpp_v3
)
# ============================================
# Example 6: Robot Delivery System
# ============================================
add_executable(example6_robot_delivery
  src/example6_robot_delivery.cpp
)
ament_target_dependencies(example6_robot_delivery
  rclcpp
  std_msgs
  behaviortree_cpp_v3
)

# Install XML tree files
install(DIRECTORY
  trees
  DESTINATION share/${PROJECT_NAME}
)

# ============================================
# Install executables
# ============================================
install(TARGETS
  example1_basic_nodes
  example2_control_flow
  example3_decorators
  example4_blackboard_ports
  example5_subtrees
  example6_robot_delivery
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================
# Install launch files if directory exists
# ============================================
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# ============================================
# Export dependencies
# ============================================
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  behaviortree_cpp_v3
)

# ============================================
# Build testing
# ============================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
