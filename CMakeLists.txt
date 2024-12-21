cmake_minimum_required(VERSION 3.8)
project(lidar_object_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
  include
  ${PCL_COMMON_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  lib/euclidean_cluster.cpp
)

target_link_libraries(${PROJECT_NAME}_lib ${PCL_LIBRARIES})

ament_target_dependencies(${PROJECT_NAME}_lib rclcpp std_msgs sensor_msgs)


ament_auto_add_executable(lidar_object_detection src/lidar_object_detection.cpp src/euclidean_cluster_node.cpp)

target_link_libraries(lidar_object_detection ${PROJECT_NAME}_lib ${PCL_LIBRARIES})

ament_target_dependencies(lidar_object_detection rclcpp std_msgs sensor_msgs PCL pcl_conversions)

# Install the shared library
install(TARGETS ${PROJECT_NAME}_lib 
  EXPORT export_${PROJECT_NAME}_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(lidar_object_detection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(lidar_object_detection PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS lidar_object_detection
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY cfg
  DESTINATION share/${PROJECT_NAME}/cfg
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

