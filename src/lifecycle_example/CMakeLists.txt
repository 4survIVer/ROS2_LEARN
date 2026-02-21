cmake_minimum_required(VERSION 3.8)
project(lifecycle_example)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)  # Enforce the standard
  set(CMAKE_CXX_EXTENSIONS OFF)  
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Declare the executable and link the libraries
add_executable(lifecycle_example_node src/lifecycle_node.cpp)

# Link libraries
ament_target_dependencies(lifecycle_example_node
  rclcpp
  rclcpp_lifecycle
)

# Install executable
install(TARGETS
  lifecycle_example_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

