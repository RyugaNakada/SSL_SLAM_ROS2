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
- [x] オドメトリ軌跡の可視化（Path）
- [x] マッピングノード
- [x] カラー画像取得・表示
- [x] ループクロージャー（ICP/GICP）

## ビルド方法
```bash
cd ~/ros2_ws
colcon build --packages-select ssl_slam_ros2
source install/setup.bash
```

## 実行方法

### 全機能統合起動（推奨）
```bash
ros2 launch ssl_slam_ros2 ssl_slam_complete.launch.py
```

### RViz付き起動
```bash
ros2 launch ssl_slam_ros2 ssl_slam_with_rviz.launch.py
```

### 個別ノード起動
```bash
# 特徴抽出ノード
ros2 run ssl_slam_ros2 ssl_feature_extraction_node

# オドメトリ推定ノード
ros2 run ssl_slam_ros2 odom_estimation_node_ros2

# マッピングノード
ros2 run ssl_slam_ros2 laser_mapping_node_ros2

# カラー画像ノード
ros2 run ssl_slam_ros2 color_image_node

# ループクロージャーノード
ros2 run ssl_slam_ros2 loop_closure_node
```

## 主要トピック

### Subscribe
- `/camera/depth/color/points` - RealSense深度点群
- `/camera/color/image_raw` - RGBカラー画像

### Publish
- `/odom` - オドメトリ情報（nav_msgs/Odometry）
- `/odom_path` - オドメトリ軌跡（nav_msgs/Path）
- `/map` - グローバルマップ点群
- `/edge_points` - エッジ特徴点
- `/surf_points` - 平面特徴点
- `/ssl_slam/color_image` - カラー画像（リレー）
- `/keyframe_cloud` - キーフレーム点群
- `/loop_closure_markers` - ループクロージャー可視化マーカー

## パラメータ

### オドメトリノード
- `scan_period`: スキャン周期（デフォルト: 0.1）
- `max_dis`: 最大検出距離（デフォルト: 9.0m）
- `min_dis`: 最小検出距離（デフォルト: 0.2m）
- `map_resolution`: マップ解像度（デフォルト: 0.05m）

### マッピングノード
- `map_resolution`: マップ解像度（デフォルト: 0.4m）
- `displacement_threshold`: 更新距離閾値（デフォルト: 0.3m）
- `angular_threshold`: 更新角度閾値（デフォルト: 20度）

### ループクロージャーノード
- `keyframe_distance_threshold`: キーフレーム距離閾値（デフォルト: 1.0m）
- `loop_search_radius`: ループ検索半径（デフォルト: 5.0m）
- `icp_fitness_threshold`: ICP適合度閾値（デフォルト: 0.3）

### カラー画像ノード
- `save_images`: 画像保存の有効化（デフォルト: false）
- `save_directory`: 保存ディレクトリ（デフォルト: /tmp/ssl_slam_images）
- `save_interval`: 保存間隔（フレーム数、デフォルト: 10）

## 依存関係
- ROS2 Humble
- PCL (Point Cloud Library)
- Ceres Solver
- Eigen3
- OpenCV
- cv_bridge
- librealsense v2.56.5

## 参考文献
- Original: https://github.com/wh200720041/ssl_slam
- Paper: "Lightweight 3-D Localization and Mapping for Solid-State LiDAR" (IEEE RA-L 2021)
