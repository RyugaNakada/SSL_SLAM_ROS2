// マッピングノード

// Author of SSL_SLAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
// ROS2 port by: ry223

// C++ standard libraries
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <memory>

// ROS2 libraries
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// PCL libraries
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Local libraries
#include "laserMappingClass.h"
#include "lidar.h"

class LaserMappingNode : public rclcpp::Node
{
public:
    LaserMappingNode() : Node("laser_mapping_node")
    {
        RCLCPP_INFO(this->get_logger(), "Laser Mapping Node Started");
        
        // パラメータ宣言と取得
        this->declare_parameter<double>("scan_period", 0.1);
        this->declare_parameter<double>("map_resolution", 0.4);
        this->declare_parameter<double>("displacement_threshold", 0.3);
        this->declare_parameter<double>("angular_threshold", 20.0);
        this->declare_parameter<bool>("use_timestamp_check", false);  // 追加: タイムスタンプチェックの有効/無効
        
        double scan_period = this->get_parameter("scan_period").as_double();
        double map_resolution = this->get_parameter("map_resolution").as_double();
        displacement_threshold_ = this->get_parameter("displacement_threshold").as_double();
        angular_threshold_ = this->get_parameter("angular_threshold").as_double();
        use_timestamp_check_ = this->get_parameter("use_timestamp_check").as_bool();
        
        RCLCPP_INFO(this->get_logger(), "Parameters - scan_period: %.2f, map_resolution: %.2f",
                   scan_period, map_resolution);
        RCLCPP_INFO(this->get_logger(), "Update thresholds - displacement: %.2fm, angular: %.1f°",
                   displacement_threshold_, angular_threshold_);
        RCLCPP_INFO(this->get_logger(), "Timestamp check: %s", use_timestamp_check_ ? "enabled" : "disabled");
        
        // LiDARパラメータ設定
        lidar_param_.setScanPeriod(scan_period);
        
        // マッピングクラス初期化
        laser_mapping_.init(map_resolution);
        
        // Subscriberの作成
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&LaserMappingNode::odom_handler, this, std::placeholders::_1));
            
        pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/color/points", 10,
            std::bind(&LaserMappingNode::pointcloud_handler, this, std::placeholders::_1));
        
        // Publisherの作成
        map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map", 2);
        
        // 処理スレッドの開始
        process_thread_ = std::thread(&LaserMappingNode::process_loop, this);
        
        // 初期姿勢
        last_pose_ = Eigen::Isometry3d::Identity();
        update_count_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "Laser Mapping Node initialization complete");
    }
    
    ~LaserMappingNode()
    {
        if (process_thread_.joinable())
        {
            process_thread_.join();
        }
    }

private:
    // マッピングクラス
    LaserMappingClass laser_mapping_;
    
    // LiDARパラメータ
    lidar::Lidar lidar_param_;
    
    // データキュー
    std::mutex mutex_lock_;
    std::queue<nav_msgs::msg::Odometry::SharedPtr> odom_buf_;
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> pointcloud_buf_;
    
    // 姿勢情報
    Eigen::Isometry3d last_pose_;
    int update_count_;
    
    // 更新閾値
    double displacement_threshold_;
    double angular_threshold_;
    bool use_timestamp_check_;  // 追加
    
    // ROS2通信
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
    
    // 処理スレッド
    std::thread process_thread_;
    
    // オドメトリコールバック
    void odom_handler(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_lock_);
        odom_buf_.push(msg);
    }
    
    // 点群コールバック
    void pointcloud_handler(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_lock_);
        pointcloud_buf_.push(msg);
    }
    
    // メイン処理ループ
    void process_loop()
    {
        while (rclcpp::ok())
        {
            if (!odom_buf_.empty() && !pointcloud_buf_.empty())
            {
                mutex_lock_.lock();
                
                // タイムスタンプチェック（オプション）
                if (use_timestamp_check_)
                {
                    double pc_time = rclcpp::Time(pointcloud_buf_.front()->header.stamp).seconds();
                    double odom_time = rclcpp::Time(odom_buf_.front()->header.stamp).seconds();
                    double time_threshold = 0.5 * lidar_param_.scan_period;
                    
                    // 点群が古すぎる場合は破棄
                    if (pc_time < odom_time - time_threshold)
                    {
                        pointcloud_buf_.pop();
                        RCLCPP_WARN(this->get_logger(), 
                                   "Pointcloud timestamp too old, discarded. Time diff: %.3fs", 
                                   odom_time - pc_time);
                        mutex_lock_.unlock();
                        continue;
                    }
                    
                    // オドメトリが古すぎる場合は破棄
                    if (odom_time < pc_time - time_threshold)
                    {
                        odom_buf_.pop();
                        RCLCPP_WARN(this->get_logger(), 
                                   "Odometry timestamp too old, discarded. Time diff: %.3fs",
                                   pc_time - odom_time);
                        mutex_lock_.unlock();
                        continue;
                    }
                }
                
                // データ取得
                auto pointcloud_msg = pointcloud_buf_.front();
                auto odom_msg = odom_buf_.front();
                
                pointcloud_buf_.pop();
                odom_buf_.pop();
                mutex_lock_.unlock();
                
                // 点群データ変換
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZRGB>());
                pcl::PointCloud<pcl::PointXYZ> pointcloud_xyz;
                
                pcl::fromROSMsg(*pointcloud_msg, pointcloud_xyz);
                pcl::copyPointCloud(pointcloud_xyz, *pointcloud_in);
                
                // 現在姿勢の取得
                Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
                
                // 回転(クォータニオン)
                Eigen::Quaterniond q(
                    odom_msg->pose.pose.orientation.w,
                    odom_msg->pose.pose.orientation.x,
                    odom_msg->pose.pose.orientation.y,
                    odom_msg->pose.pose.orientation.z
                );
                current_pose.rotate(q);
                
                // 並進
                current_pose.pretranslate(Eigen::Vector3d(
                    odom_msg->pose.pose.position.x,
                    odom_msg->pose.pose.position.y,
                    odom_msg->pose.pose.position.z
                ));
                
                // 移動量・回転量の計算
                update_count_++;
                Eigen::Isometry3d delta_transform = last_pose_.inverse() * current_pose;
                double displacement = delta_transform.translation().squaredNorm();
                
                // オイラー角から回転量計算(Z軸回転)
                Eigen::Vector3d euler = delta_transform.linear().eulerAngles(2, 1, 0);
                double angular_change = std::abs(euler[0] * 180.0 / M_PI);
                
                if (angular_change > 90.0)
                    angular_change = std::abs(180.0 - angular_change);
                
                // 閾値チェックしてマップ更新
                if (displacement > displacement_threshold_ || angular_change > angular_threshold_)
                {
                    RCLCPP_INFO(this->get_logger(), 
                               "Updating map - displacement: %.2fm, angular: %.1f°",
                               std::sqrt(displacement), angular_change);
                    
                    last_pose_ = current_pose;
                    laser_mapping_.updateCurrentPointsToMap(pointcloud_in, current_pose);
                    
                    // マップパブリッシュ
                    publish_map(pointcloud_msg->header.stamp);
                }
            }
            
            // 10ms待機
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    
    // マップのパブリッシュ
    void publish_map(const builtin_interfaces::msg::Time& stamp)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_cloud = laser_mapping_.getMap();
        
        if (map_cloud->points.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Map is empty, skipping publish");
            return;
        }
        
        sensor_msgs::msg::PointCloud2 map_msg;
        pcl::toROSMsg(*map_cloud, map_msg);
        map_msg.header.stamp = stamp;
        map_msg.header.frame_id = "map";
        
        map_publisher_->publish(map_msg);
        
        RCLCPP_INFO(this->get_logger(), "Map published - %zu points", map_cloud->points.size());
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<LaserMappingNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}