cmake_minimum_required(VERSION 3.8)
project(ssl_slam_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include 
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# メインノード: SSL特徴抽出
add_executable(ssl_feature_extraction_node src/ssl_feature_extraction_node.cpp)
ament_target_dependencies(ssl_feature_extraction_node 
  rclcpp sensor_msgs pcl_conversions PCL)
install(TARGETS ssl_feature_extraction_node DESTINATION lib/${PROJECT_NAME})

# オドメトリ推定ノード
add_executable(odom_estimation_node_ros2 
  src/odom_estimation_node_ros2.cpp
  src/odomEstimationClass.cpp
  src/lidarOptimization.cpp
  src/lidar.cpp
)
ament_target_dependencies(odom_estimation_node_ros2 
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros pcl_conversions PCL
)
target_link_libraries(odom_estimation_node_ros2 ${CERES_LIBRARIES})
install(TARGETS odom_estimation_node_ros2 DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch_ros2/
  DESTINATION share/${PROJECT_NAME}/launch_ros2
)

# マッピングノード
add_executable(laser_mapping_node_ros2 
  src/laser_mapping_node_ros2.cpp
  src/laserMappingClass.cpp
  src/lidar.cpp
)
ament_target_dependencies(laser_mapping_node_ros2 
  rclcpp sensor_msgs nav_msgs geometry_msgs pcl_conversions PCL
)
install(TARGETS laser_mapping_node_ros2 DESTINATION lib/${PROJECT_NAME})

ament_package()