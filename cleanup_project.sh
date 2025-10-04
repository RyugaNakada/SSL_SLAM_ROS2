#!/bin/bash
# SSL_SLAM ROS2プロジェクト整理スクリプト

cd ~/ros2_ws/src/ssl_slam_ros2

echo "=== SSL_SLAM ROS2 プロジェクト整理 ==="
echo ""

# バックアップディレクトリ作成
mkdir -p ~/ssl_slam_backup_$(date +%Y%m%d)
BACKUP_DIR=~/ssl_slam_backup_$(date +%Y%m%d)

echo "バックアップディレクトリ: $BACKUP_DIR"

# 1. includeディレクトリの整理
echo ""
echo "1. includeディレクトリの整理"
# laserProcessingClass.hをバックアップして削除
cp include/laserProcessingClass.h $BACKUP_DIR/
rm include/laserProcessingClass.h
echo "  - laserProcessingClass.h を削除"

# 2. srcディレクトリの整理
echo ""
echo "2. srcディレクトリの整理"

# テストノード削除
cp src/simple_laser_node.cpp $BACKUP_DIR/
cp src/enhanced_laser_node.cpp $BACKUP_DIR/
cp src/pcl_processing_node_fixed.cpp $BACKUP_DIR/
rm src/simple_laser_node.cpp
rm src/enhanced_laser_node.cpp
rm src/pcl_processing_node_fixed.cpp
rm src/pcl_processing_node.cp
echo "  - テストノード削除完了"

# 不要なROS1/試作ファイル削除
cp src/laserProcessingClass.cpp $BACKUP_DIR/
cp src/laserProcessingNode.cpp $BACKUP_DIR/
cp src/laserProcessingNode_ros2.cpp $BACKUP_DIR/
rm src/laserProcessingClass.cpp
rm src/laserProcessingNode.cpp
rm src/laserProcessingNode_ros2.cpp
echo "  - 不要なROS1ファイル削除完了"

# OctoMap関連は保留（コメントアウト）
# cp src/octoMappingClass.cpp $BACKUP_DIR/
# cp src/octoMappingNode.cpp $BACKUP_DIR/
# rm src/octoMappingClass.cpp
# rm src/octoMappingNode.cpp

# src/.vscodeディレクトリ削除
rm -rf src/.vscode
echo "  - src/.vscode削除完了"

# 3. 古いログ削除
echo ""
echo "3. 古いログ削除"
cd ~/ros2_ws
find log/ -type d -name "build_2024-*" -exec rm -rf {} + 2>/dev/null
find log/ -type d -name "build_2025-09-*" -exec rm -rf {} + 2>/dev/null
echo "  - 古いログ削除完了"

# 4. CMakeLists.txt更新
echo ""
echo "4. CMakeLists.txt更新"
cd ~/ros2_ws/src/ssl_slam_ros2

# バックアップ
cp CMakeLists.txt $BACKUP_DIR/

# 新しいCMakeLists.txt作成
cat > CMakeLists.txt << 'EOF'
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

ament_package()
EOF

echo "  - CMakeLists.txt更新完了"

# 5. 整理結果表示
echo ""
echo "=== 整理完了 ==="
echo ""
echo "残っているファイル:"
echo ""
echo "include/"
ls -1 include/*.h
echo ""
echo "src/"
ls -1 src/*.cpp
echo ""
echo "バックアップ場所: $BACKUP_DIR"
echo ""
echo "次のステップ:"
echo "1. cd ~/ros2_ws"
echo "2. colcon build --packages-select ssl_slam_ros2"
echo "3. source install/setup.bash"
