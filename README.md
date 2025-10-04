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

## ビルド方法
```bash
cd ~/ros2_ws
colcon build --packages-select ssl_slam_ros2
source install/setup.bash
