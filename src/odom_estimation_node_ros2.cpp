// オドメトリ推定ノード

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
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>

// PCL libraries
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Local libraries
#include "lidar.h"
#include "odomEstimationClass.h"

class OdomEstimationNode : public rclcpp::Node
{
public:
    OdomEstimationNode() : Node("odom_estimation_node")
    {
        RCLCPP_INFO(this->get_logger(), "Odom Estimation Node Started");
        
        // パラメータ宣言と取得
        this->declare_parameter<double>("scan_period", 0.1);
        this->declare_parameter<double>("max_dis", 9.0);
        this->declare_parameter<double>("min_dis", 0.2);
        this->declare_parameter<double>("map_resolution", 0.05);
        
        double scan_period = this->get_parameter("scan_period").as_double();
        double max_dis = this->get_parameter("max_dis").as_double();
        double min_dis = this->get_parameter("min_dis").as_double();
        double map_resolution = this->get_parameter("map_resolution").as_double();
        
        RCLCPP_INFO(this->get_logger(), "Parameters - scan_period: %.2f, max_dis: %.2f, min_dis: %.2f, map_resolution: %.4f",
                   scan_period, max_dis, min_dis, map_resolution);
        
        // LiDARパラメータ設定
        lidar_param.setScanPeriod(scan_period);
        lidar_param.setMaxDistance(max_dis);
        lidar_param.setMinDistance(min_dis);
        
        // オドメトリ推定クラスの初期化
        odom_estimation_.init(lidar_param, map_resolution);
        
        // Subscriberの作成
        edge_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/edge_points", 10,
            std::bind(&OdomEstimationNode::edge_handler, this, std::placeholders::_1));
            
        surf_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/surf_points", 10,
            std::bind(&OdomEstimationNode::surf_handler, this, std::placeholders::_1));
        
        // Publisherの作成
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        
        // TF broadcasterの初期化
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        // 処理スレッドの開始
        process_thread_ = std::thread(&OdomEstimationNode::process_loop, this);
        
        RCLCPP_INFO(this->get_logger(), "Odom Estimation Node initialization complete");
    }
    
    ~OdomEstimationNode()
    {
        if (process_thread_.joinable())
        {
            process_thread_.join();
        }
    }

private:
    // オドメトリ推定クラス
    OdomEstimationClass odom_estimation_;
    
    // LiDARパラメータ
    lidar::Lidar lidar_param;
    
    // データキュー
    std::mutex mutex_lock_;
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> edge_buf_;
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> surf_buf_;
    
    // ROS2通信
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr edge_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr surf_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // 処理スレッド
    std::thread process_thread_;
    
    // Edge特徴点コールバック
    void edge_handler(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_lock_);
        edge_buf_.push(msg);
    }
    
    // Surface特徴点コールバック
    void surf_handler(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_lock_);
        surf_buf_.push(msg);
    }
    
    // メイン処理ループ
    void process_loop()
    {
        while (rclcpp::ok())
        {
            if (!edge_buf_.empty() && !surf_buf_.empty())
            {
                mutex_lock_.lock();
                
                // 最も古いメッセージを取得
                auto edge_msg = edge_buf_.front();
                auto surf_msg = surf_buf_.front();
                
                // タイムスタンプの同期チェック
                double time_diff = std::abs(rclcpp::Time(edge_msg->header.stamp).seconds() - 
                                           rclcpp::Time(surf_msg->header.stamp).seconds());
                
                if (time_diff > 0.2)
                {
                    // 時刻が大きくずれている場合、古い方を破棄
                    if (rclcpp::Time(edge_msg->header.stamp) < rclcpp::Time(surf_msg->header.stamp))
                    {
                        edge_buf_.pop();
                        RCLCPP_WARN(this->get_logger(), "Edge message dropped due to time mismatch");
                    }
                    else
                    {
                        surf_buf_.pop();
                        RCLCPP_WARN(this->get_logger(), "Surface message dropped due to time mismatch");
                    }
                    mutex_lock_.unlock();
                    continue;
                }
                
                // キューから削除
                edge_buf_.pop();
                surf_buf_.pop();
                mutex_lock_.unlock();
                
                // 点群データに変換
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr edge_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr surf_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
                
                pcl::PointCloud<pcl::PointXYZ> edge_cloud_xyz;
                pcl::PointCloud<pcl::PointXYZ> surf_cloud_xyz;
                
                pcl::fromROSMsg(*edge_msg, edge_cloud_xyz);
                pcl::fromROSMsg(*surf_msg, surf_cloud_xyz);
                
                // PointXYZ -> PointXYZRGB変換
                pcl::copyPointCloud(edge_cloud_xyz, *edge_cloud);
                pcl::copyPointCloud(surf_cloud_xyz, *surf_cloud);
                
                RCLCPP_DEBUG(this->get_logger(), "Processing - Edge: %zu points, Surf: %zu points",
                            edge_cloud->points.size(), surf_cloud->points.size());
                
                // オドメトリ更新
                odom_estimation_.updatePointsToMap(edge_cloud, surf_cloud);
                
                // オドメトリパブリッシュ
                publish_odometry(edge_msg->header.stamp);
            }
            
            // 10ms待機
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    
    // オドメトリのパブリッシュ
    void publish_odometry(const builtin_interfaces::msg::Time& stamp)
    {
        // オドメトリメッセージ作成
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = stamp;
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = "base_link";
        
        // 位置
        Eigen::Isometry3d odom = odom_estimation_.odom;
        odom_msg.pose.pose.position.x = odom.translation().x();
        odom_msg.pose.pose.position.y = odom.translation().y();
        odom_msg.pose.pose.position.z = odom.translation().z();
        
        // 姿勢（クォータニオン）
        Eigen::Quaterniond q(odom.rotation());
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();
        
        // パブリッシュ
        odom_publisher_->publish(odom_msg);
        
        // TF送信
        publish_tf(stamp, odom);
        
        RCLCPP_DEBUG(this->get_logger(), "Odom published - pos: [%.2f, %.2f, %.2f]",
                    odom.translation().x(), odom.translation().y(), odom.translation().z());
    }
    
    // TFのパブリッシュ
    void publish_tf(const builtin_interfaces::msg::Time& stamp, const Eigen::Isometry3d& odom)
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        
        transform_stamped.header.stamp = stamp;
        transform_stamped.header.frame_id = "map";
        transform_stamped.child_frame_id = "base_link";
        
        // 位置
        transform_stamped.transform.translation.x = odom.translation().x();
        transform_stamped.transform.translation.y = odom.translation().y();
        transform_stamped.transform.translation.z = odom.translation().z();
        
        // 姿勢
        Eigen::Quaterniond q(odom.rotation());
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();
        
        // TF送信
        tf_broadcaster_->sendTransform(transform_stamped);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<OdomEstimationNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}