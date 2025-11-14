// ループクロージャーノード
// ROS2 port by: ry223

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "loopClosureClass.h"

#include <memory>
#include <mutex>
#include <queue>
#include <thread>

class LoopClosureNode : public rclcpp::Node
{
public:
    LoopClosureNode() : Node("loop_closure_node")
    {
        RCLCPP_INFO(this->get_logger(), "Loop Closure Node Started");

        // パラメータ宣言
        this->declare_parameter<double>("keyframe_distance_threshold", 1.0);
        this->declare_parameter<double>("loop_search_radius", 5.0);
        this->declare_parameter<double>("icp_fitness_threshold", 0.3);

        double keyframe_distance = this->get_parameter("keyframe_distance_threshold").as_double();
        double search_radius = this->get_parameter("loop_search_radius").as_double();
        double icp_threshold = this->get_parameter("icp_fitness_threshold").as_double();

        RCLCPP_INFO(this->get_logger(), "Parameters:");
        RCLCPP_INFO(this->get_logger(), "  - Keyframe distance: %.2fm", keyframe_distance);
        RCLCPP_INFO(this->get_logger(), "  - Search radius: %.2fm", search_radius);
        RCLCPP_INFO(this->get_logger(), "  - ICP threshold: %.3f", icp_threshold);

        // ループクロージャークラス初期化
        loop_closure_.init(keyframe_distance, search_radius, icp_threshold);

        // Subscriber作成
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&LoopClosureNode::odom_callback, this, std::placeholders::_1));

        pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/color/points", 10,
            std::bind(&LoopClosureNode::pointcloud_callback, this, std::placeholders::_1));

        // Publisher作成
        loop_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/loop_closure_markers", 10);

        keyframe_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/keyframe_cloud", 2);

        // 処理スレッド開始
        process_thread_ = std::thread(&LoopClosureNode::process_loop, this);

        RCLCPP_INFO(this->get_logger(), "Loop Closure Node initialization complete");
    }

    ~LoopClosureNode()
    {
        if (process_thread_.joinable())
        {
            process_thread_.join();
        }
    }

private:
    // ループクロージャークラス
    LoopClosureClass loop_closure_;

    // データキュー
    std::mutex mutex_lock_;
    std::queue<nav_msgs::msg::Odometry::SharedPtr> odom_buf_;
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> pointcloud_buf_;

    // ROS2通信
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr loop_marker_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_cloud_publisher_;

    // 処理スレッド
    std::thread process_thread_;

    // コールバック
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_lock_);
        odom_buf_.push(msg);
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
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

                auto odom_msg = odom_buf_.front();
                auto pc_msg = pointcloud_buf_.front();

                odom_buf_.pop();
                pointcloud_buf_.pop();
                mutex_lock_.unlock();

                // 点群変換
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>());
                pcl::PointCloud<pcl::PointXYZ> pc_xyz;
                pcl::fromROSMsg(*pc_msg, pc_xyz);
                pcl::copyPointCloud(pc_xyz, *pointcloud);

                // 姿勢取得
                Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
                Eigen::Quaterniond q(
                    odom_msg->pose.pose.orientation.w,
                    odom_msg->pose.pose.orientation.x,
                    odom_msg->pose.pose.orientation.y,
                    odom_msg->pose.pose.orientation.z
                );
                pose.rotate(q);
                pose.pretranslate(Eigen::Vector3d(
                    odom_msg->pose.pose.position.x,
                    odom_msg->pose.pose.position.y,
                    odom_msg->pose.pose.position.z
                ));

                double timestamp = rclcpp::Time(odom_msg->header.stamp).seconds();

                // キーフレーム追加
                if (loop_closure_.addKeyFrame(timestamp, pose, pointcloud))
                {
                    // キーフレーム点群をパブリッシュ
                    publish_keyframe_cloud(pc_msg->header.stamp);

                    // ループ検出
                    Eigen::Isometry3d loop_transform;
                    int loop_from, loop_to;

                    if (loop_closure_.detectAndCloseLoop(loop_transform, loop_from, loop_to))
                    {
                        RCLCPP_INFO(this->get_logger(),
                                   "Loop closure detected: %d <-> %d", loop_from, loop_to);

                        // マーカーをパブリッシュ
                        publish_loop_markers(loop_from, loop_to);
                    }
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    // キーフレーム点群のパブリッシュ
    void publish_keyframe_cloud(const builtin_interfaces::msg::Time& stamp)
    {
        auto& keyframes = loop_closure_.getKeyFrames();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

        // 全キーフレームの点群を結合
        for (const auto& kf : keyframes)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::transformPointCloud(*(kf.pointcloud), *transformed, kf.pose.cast<float>());
            *combined_cloud += *transformed;
        }

        if (!combined_cloud->empty())
        {
            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl::toROSMsg(*combined_cloud, cloud_msg);
            cloud_msg.header.stamp = stamp;
            cloud_msg.header.frame_id = "map";
            keyframe_cloud_publisher_->publish(cloud_msg);
        }
    }

    // ループマーカーのパブリッシュ
    void publish_loop_markers(int from_id, int to_id)
    {
        auto& keyframes = loop_closure_.getKeyFrames();

        visualization_msgs::msg::MarkerArray marker_array;

        // ラインマーカー作成
        visualization_msgs::msg::Marker line_marker;
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = this->now();
        line_marker.ns = "loop_closure";
        line_marker.id = from_id * 1000 + to_id;
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;

        line_marker.scale.x = 0.1;  // 線の太さ
        line_marker.color.r = 1.0;
        line_marker.color.g = 0.0;
        line_marker.color.b = 0.0;
        line_marker.color.a = 1.0;

        geometry_msgs::msg::Point p1, p2;
        p1.x = keyframes[from_id].pose.translation().x();
        p1.y = keyframes[from_id].pose.translation().y();
        p1.z = keyframes[from_id].pose.translation().z();

        p2.x = keyframes[to_id].pose.translation().x();
        p2.y = keyframes[to_id].pose.translation().y();
        p2.z = keyframes[to_id].pose.translation().z();

        line_marker.points.push_back(p1);
        line_marker.points.push_back(p2);

        marker_array.markers.push_back(line_marker);

        loop_marker_publisher_->publish(marker_array);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LoopClosureNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
