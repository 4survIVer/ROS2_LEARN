###############################################################################
# ROS 2 PACKAGE CMAKELISTS.TXT
# Comprehensive template with detailed explanations
###############################################################################

#############################################
# SECTION 1: CMAKE MINIMUM VERSION AND PROJECT
#############################################

# Set the minimum required CMake version
# ROS 2 Humble requires at least CMake 3.8
cmake_minimum_required(VERSION 3.8)

# Define the project name - MUST match package.xml <name> tag
project(ros2_learning_pkg)

#############################################
# SECTION 2: COMPILER AND LANGUAGE STANDARDS
#############################################

# Check if C++ standard is not already set, then default to C++17
# Modern ROS 2 packages typically use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)  # Enforce the standard
  set(CMAKE_CXX_EXTENSIONS OFF)        # Disable compiler-specific extensions
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
#############################################
# SECTION 3: FIND DEPENDENCIES
#############################################

# Essential ROS 2 package infrastructure
find_package(ament_cmake REQUIRED)

# ROS 2 C++ client library - core dependency for all ROS 2 C++ nodes
find_package(rclcpp REQUIRED)

# Standard messages package - for std_msgs message types
find_package(std_msgs REQUIRED)


find_package(geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

  # CUSTOM MESSAGE GENERATION
find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp_action REQUIRED)

# NOTE: Add other dependencies here as needed, for example:
# find_package(geometry_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)

#############################################
# SECTION 4: INCLUDE DIRECTORIES
#############################################

# Add the include directory to the compiler's search path
# This allows #include "ros2_learning_pkg/header_file.hpp" to work
include_directories(include)


# ===========================================
# CUSTOM INTERFACES GENERATION
# ===========================================
# ===========================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StudentInfo.msg"
  "msg/VectorStamped.msg"
  "srv/StudentGrade.srv"
  "action/ProcessStudentSubmission.action"
  DEPENDENCIES std_msgs builtin_interfaces
)


#############################################
# SECTION 5: CREATE EXECUTABLES (NODES)
#############################################

# Build the publisher node executable
# Syntax: add_executable(executable_name source_files)
add_executable(simple_publisher src/topics/simple_publisher.cpp)

# Specify dependencies for the publisher node
# This ensures proper include directories and linking
ament_target_dependencies(simple_publisher 
  rclcpp 
  std_msgs
)

# Build the subscriber node executable
add_executable(simple_subscriber src/topics/simple_subscriber.cpp)

# Specify dependencies for the subscriber node
ament_target_dependencies(simple_subscriber 
  rclcpp 
  std_msgs
)

# NEW: Custom message publisher
add_executable(custom_message_publisher src/topics/custom_message_publisher.cpp)
ament_target_dependencies(custom_message_publisher 
  rclcpp
  std_msgs
  builtin_interfaces
  geometry_msgs
)
#target_link_libraries(custom_message_publisher
#  ${PROJECT_NAME}__rosidl_typesupport_cpp
#  ${PROJECT_NAME}__rosidl_typesupport_fastrtps_cpp
#  ${PROJECT_NAME}__rosidl_typesupport_introspection_cpp
#)
target_link_libraries(custom_message_publisher
  ${PROJECT_NAME}__rosidl_generator_c
  ${PROJECT_NAME}__rosidl_typesupport_c
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# NEW: Custom message subscriber
add_executable(custom_message_subscriber src/topics/custom_message_subscriber.cpp)
ament_target_dependencies(custom_message_subscriber 
  rclcpp
  std_msgs
  builtin_interfaces
  geometry_msgs
)
#target_link_libraries(custom_message_subscriber
#  ${PROJECT_NAME}__rosidl_typesupport_cpp
#  ${PROJECT_NAME}__rosidl_typesupport_fastrtps_cpp
#  ${PROJECT_NAME}__rosidl_typesupport_introspection_cpp
#)
target_link_libraries(custom_message_subscriber
  ${PROJECT_NAME}__rosidl_generator_c
  ${PROJECT_NAME}__rosidl_typesupport_c
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# NEW: Multiple Publishers/Subscribers Examples
add_executable(sensor_publisher_1 src/topics/sensor_publisher_1.cpp)
ament_target_dependencies(sensor_publisher_1 rclcpp std_msgs)
  
add_executable(sensor_publisher_2 src/topics/sensor_publisher_2.cpp)
ament_target_dependencies(sensor_publisher_2 rclcpp std_msgs)
  
add_executable(display_subscriber_1 src/topics/display_subscriber_1.cpp)
ament_target_dependencies(display_subscriber_1 rclcpp std_msgs)
  
add_executable(alert_subscriber_2 src/topics/alert_subscriber_2.cpp)
ament_target_dependencies(alert_subscriber_2 rclcpp std_msgs)

add_executable(add_two_ints_server src/services/add_two_ints_server.cpp)
ament_target_dependencies(add_two_ints_server rclcpp example_interfaces)
  
add_executable(add_two_ints_client src/services/add_two_ints_client.cpp)
ament_target_dependencies(add_two_ints_client rclcpp example_interfaces)

# NEW: Student Grade Service Server
add_executable(student_grade_server src/services/student_grade_server.cpp)
ament_target_dependencies(student_grade_server 
  rclcpp 
  std_msgs
)
target_link_libraries(student_grade_server
  ${PROJECT_NAME}__rosidl_generator_c
  ${PROJECT_NAME}__rosidl_typesupport_c
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)


# NEW: Student Grade Service Client  
add_executable(student_grade_client src/services/student_grade_client.cpp)
ament_target_dependencies(student_grade_client 
  rclcpp 
  std_msgs
)
target_link_libraries(student_grade_client
  ${PROJECT_NAME}__rosidl_generator_c
  ${PROJECT_NAME}__rosidl_typesupport_c
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)
# Advanced Grade Client with multiple service patterns
add_executable(advanced_grade_client src/services/advanced_grade_client.cpp)
ament_target_dependencies(advanced_grade_client 
  rclcpp 
  std_msgs
)
target_link_libraries(advanced_grade_client
  ${PROJECT_NAME}__rosidl_generator_c
  ${PROJECT_NAME}__rosidl_typesupport_c
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)
# Advanced Grade Server with simulated delays
add_executable(advanced_grade_server src/services/advanced_grade_server.cpp)
ament_target_dependencies(advanced_grade_server 
  rclcpp 
  std_msgs
)
target_link_libraries(advanced_grade_server
  ${PROJECT_NAME}__rosidl_generator_c
  ${PROJECT_NAME}__rosidl_typesupport_c
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)
# Action Server - Student Submission Processing
add_executable(student_submission_server src/actions/student_submission_server.cpp)
ament_target_dependencies(student_submission_server 
  rclcpp 
  rclcpp_action
)
target_link_libraries(student_submission_server
  ${PROJECT_NAME}__rosidl_generator_c
  ${PROJECT_NAME}__rosidl_typesupport_c
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# Action Client - Student Submission Processing
add_executable(student_submission_client src/actions/student_submission_client.cpp)
ament_target_dependencies(student_submission_client 
  rclcpp 
  rclcpp_action
)
target_link_libraries(student_submission_client
  ${PROJECT_NAME}__rosidl_generator_c
  ${PROJECT_NAME}__rosidl_typesupport_c
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# Circuit Breaker Pattern
add_executable(circuit_breaker_client src/services/circuit_breaker_client.cpp)
ament_target_dependencies(circuit_breaker_client 
  rclcpp 
  std_msgs
)
target_link_libraries(circuit_breaker_client
  ${PROJECT_NAME}__rosidl_generator_c
  ${PROJECT_NAME}__rosidl_typesupport_c
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)
# Load Balancing Pattern
add_executable(load_balancing_client src/services/load_balancing_client.cpp)
ament_target_dependencies(load_balancing_client 
  rclcpp 
  std_msgs
)
target_link_libraries(load_balancing_client
  ${PROJECT_NAME}__rosidl_generator_c
  ${PROJECT_NAME}__rosidl_typesupport_c
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

#advanced_submission_server
#advanced_submission_client

# Advanced Action Server with Goal Management
add_executable(advanced_submission_server src/actions/advanced_submission_server.cpp)
ament_target_dependencies(advanced_submission_server 
  rclcpp 
  rclcpp_action
)
target_link_libraries(advanced_submission_server
  ${PROJECT_NAME}__rosidl_generator_c
  ${PROJECT_NAME}__rosidl_typesupport_c
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# Advanced Action Client
add_executable(advanced_submission_client src/actions/advanced_submission_client.cpp)
ament_target_dependencies(advanced_submission_client 
  rclcpp 
  rclcpp_action
)
target_link_libraries(advanced_submission_client
  ${PROJECT_NAME}__rosidl_generator_c
  ${PROJECT_NAME}__rosidl_typesupport_c
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

#############################################
# SECTION 6: INSTALLATION CONFIGURATION
#############################################
# This section defines what gets installed and where
# Critical for making your package usable by others

# Install executables to lib/<package_name> directory
# This follows ROS 2 convention and makes nodes runnable with 'ros2 run'
install(TARGETS
  simple_publisher
  simple_subscriber
  custom_message_publisher
  custom_message_subscriber
  sensor_publisher_1
  sensor_publisher_2
  display_subscriber_1
  alert_subscriber_2
  add_two_ints_server
  add_two_ints_client
  student_grade_server
  student_grade_client
  advanced_grade_client
  advanced_grade_server
  student_submission_server
  student_submission_client
  circuit_breaker_client
  load_balancing_client
  advanced_submission_server
  advanced_submission_client
  ARCHIVE DESTINATION lib/${PROJECT_NAME}    # Static libraries (.a)
  LIBRARY DESTINATION lib/${PROJECT_NAME}    # Shared libraries (.so)
  RUNTIME DESTINATION lib/${PROJECT_NAME}    # Executables (.exe on Windows)
)

# Install header files to include directory
# Makes headers available to other packages that depend on this one
install(DIRECTORY include/ 
  DESTINATION include
)

# Install launch files to share/<package_name>/launch
# Allows using: ros2 launch ros2_learning_pkg your_launch_file.launch.py
install(DIRECTORY launch/ 
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install configuration files to share/<package_name>/config
# For YAML config files, parameter files, etc.
install(DIRECTORY config/ 
  DESTINATION share/${PROJECT_NAME}/config
)

#############################################
# SECTION 7: EXPORT DEPENDENCIES AND CONFIGURATION
#############################################
# This section tells ROS 2 what your package provides and needs

# Export dependencies so other packages can find what this package needs
# Critical for the ROS 2 build system dependency resolution
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  example_interfaces
  rosidl_default_runtime
  builtin_interfaces
)

# Export include directories so dependent packages can find headers
ament_export_include_directories(include)

# If you had libraries, you'd export them too:
# ament_export_libraries(your_library_name)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
#############################################
# SECTION 8: FINAL PACKAGE COMMAND
#############################################
# MUST BE THE LAST LINE IN THE FILE
# Generates necessary configuration files for ROS 2
ament_package()
