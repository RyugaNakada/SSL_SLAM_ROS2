cd ~/ros2_ws/src/ssl_slam_ros2
cat > README.md << 'EOF'
# SSL_SLAM ROS2

Intel RealSense L515を使用したSSL_SLAM（Lightweight 3-D Localization and Mapping for Solid-State LiDAR）のROS2移植版

## 開発環境
- Ubuntu 22.04 WSL2
- ROS2 Humble
- Intel RealSense L515

## 完成機能
- [x] スキャンライン分割
- [x] 曲率計算・特徴点検出
- [x] オドメトリ推定（Ceres Solver）
- [ ] マッピングノード
- [ ] ループクロージャ

## 実行方法
- bash# 特徴抽出ノード
- ros2 run ssl_slam_ros2 ssl_feature_extraction_node

- # オドメトリ推定ノード
- ros2 run ssl_slam_ros2 odom_estimation_node_ros2

## 依存関係
- ROS2 Humble
- PCL (Point Cloud Library)
- Ceres Solver
- Eigen3
- librealsense v2.56.5

## 参考文献
Original: https://github.com/wh200720041/ssl_slam
Paper: "Lightweight 3-D Localization and Mapping for Solid-State LiDAR" (IEEE RA-L 2021)
EOF

## コミット・プッシュ
git add README.md
git commit -m "Add README.md with project documentation"
git push

## ビルド方法
```bash
cd ~/ros2_ws
colcon build --packages-select ssl_slam_ros2
source install/setup.bash
