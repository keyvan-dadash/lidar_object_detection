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
find_package(vision_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Euclidean Clustering library
add_library(${PROJECT_NAME}_euclidean_cluster_lib SHARED
  lib/euclidean_cluster.cpp
  lib/cluster.cpp
)

ament_target_dependencies(${PROJECT_NAME}_euclidean_cluster_lib
  rclcpp
  rclcpp_components
  sensor_msgs
  vision_msgs
  pcl_conversions
  PCL
)

target_include_directories(${PROJECT_NAME}_euclidean_cluster_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# DBSCAN Clustering library
add_library(${PROJECT_NAME}_dbscan_cluster_lib SHARED
  lib/dbScan.cpp
  lib/cluster.cpp
  lib/OctreeGenerator.cpp
)

ament_target_dependencies(${PROJECT_NAME}_dbscan_cluster_lib
  rclcpp
  rclcpp_components
  sensor_msgs
  vision_msgs
  pcl_conversions
  PCL
)

target_include_directories(${PROJECT_NAME}_dbscan_cluster_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Euclidean Clustering node
add_library(${PROJECT_NAME}_euclidean_cluster_node_core SHARED
  src/euclidean_cluster_node.cpp
)

target_link_libraries(${PROJECT_NAME}_euclidean_cluster_node_core
  ${PROJECT_NAME}_euclidean_cluster_lib
)

ament_target_dependencies(${PROJECT_NAME}_euclidean_cluster_node_core
  rclcpp
  rclcpp_components
  sensor_msgs
  vision_msgs
  pcl_conversions
  PCL
)

rclcpp_components_register_node(${PROJECT_NAME}_euclidean_cluster_node_core
  PLUGIN "lidar_object_detection::euclidean_cluster::EuclideanClusterNode"
  EXECUTABLE euclidean_cluster_node
)

# DBSCAN Clustering node
add_library(${PROJECT_NAME}_dbscan_cluster_node_core SHARED
  src/dbscan_cluster_node.cpp
)

target_link_libraries(${PROJECT_NAME}_dbscan_cluster_node_core
  ${PROJECT_NAME}_dbscan_cluster_lib
)

ament_target_dependencies(${PROJECT_NAME}_dbscan_cluster_node_core
  rclcpp
  rclcpp_components
  sensor_msgs
  vision_msgs
  pcl_conversions
  PCL
)

rclcpp_components_register_node(${PROJECT_NAME}_dbscan_cluster_node_core
  PLUGIN "lidar_object_detection::dbscan::DBScanClusterNode"
  EXECUTABLE dbscan_cluster_node
)

# Install libraries and nodes
install(
  TARGETS
    ${PROJECT_NAME}_euclidean_cluster_lib
    ${PROJECT_NAME}_dbscan_cluster_lib
    ${PROJECT_NAME}_euclidean_cluster_node_core
    ${PROJECT_NAME}_dbscan_cluster_node_core
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

