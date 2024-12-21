cmake_minimum_required(VERSION 3.8)
project(lidar_object_detection)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Enable warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Build the node as a shared library
add_library(${PROJECT_NAME}_node SHARED
  src/euclidean_cluster_node.cpp
  lib/euclidean_cluster.cpp
)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_components
  sensor_msgs
  pcl_conversions
  PCL
)

# Register the node as a component
rclcpp_components_register_nodes(${PROJECT_NAME}_node "lidar_object_detection::euclidean_cluster::EuclideanClusterNode")

# Install the library
install(
  TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include files
install(
  DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(
  DIRECTORY cfg
  DESTINATION share/${PROJECT_NAME}/cfg
)

# Ament package generation
ament_package()
