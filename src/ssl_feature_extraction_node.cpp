// 特徴抽出ノード

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <cmath>
#include <vector>
#include <algorithm>

class SSLFeatureExtractionNode : public rclcpp::Node
{
public:
    SSLFeatureExtractionNode() : Node("ssl_feature_extraction_node")
    {
        RCLCPP_INFO(this->get_logger(), "SSL Feature Extraction Node Started");
        
        // L515の仕様に基づく設定
        vertical_fov_ = 55.0;
        horizontal_fov_ = 70.0;
        vertical_scans_ = 480;
        horizontal_scans_ = 640;
        
        // スキャンライン分割パラメータ
        scanline_threshold_ = 0.1;
        min_points_per_scanline_ = 10;
        
        // 特徴抽出パラメータの初期化
        edge_threshold_ = 1.0;
        surf_threshold_ = 0.1;
        curvature_region_ = 5;
        max_edge_points_per_scan_ = 2;
        max_surf_points_per_scan_ = 4;
        
        RCLCPP_INFO(this->get_logger(), "Scanline segmentation parameters initialized");
        RCLCPP_INFO(this->get_logger(), "Feature extraction parameters initialized");
        
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/color/points", 10,
            std::bind(&SSLFeatureExtractionNode::pointcloud_callback, this, std::placeholders::_1));
        edge_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/edge_points", 10);
        surf_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/surf_points", 10);
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Processing SSL feature extraction");
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        coordinate_transform(cloud);
        extract_features(cloud);
    }

    // L515スキャンライン分割用パラメータ
    double vertical_fov_;
    double horizontal_fov_;
    int vertical_scans_;
    int horizontal_scans_;
    double scanline_threshold_;
    size_t min_points_per_scanline_;

    // 特徴抽出パラメータ
    double edge_threshold_;
    double surf_threshold_;
    int curvature_region_;
    int max_edge_points_per_scan_;
    int max_surf_points_per_scan_;

    void coordinate_transform(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        for (int i = 0; i < (int) cloud->points.size(); i++)
        {
            double new_x = cloud->points[i].z;
            double new_y = -cloud->points[i].x;
            double new_z = -cloud->points[i].y;
            cloud->points[i].x = new_x;
            cloud->points[i].y = new_y;
            cloud->points[i].z = new_z;
        }
    }

    void extract_features(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        RCLCPP_INFO(this->get_logger(), "Starting feature extraction from %zu points", cloud->points.size());
        
        // スキャンライン分割
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> scanline_clouds;
        scanline_segmentation(cloud, scanline_clouds);
        
        // 統計情報出力
        if (!scanline_clouds.empty())
        {
            size_t total_points = 0;
            for (const auto& scanline : scanline_clouds)
            {
                total_points += scanline->points.size();
            }
            
            RCLCPP_INFO(this->get_logger(), 
                       "Scanline segmentation: %zu lines, %zu total points, avg %.1f points/line",
                       scanline_clouds.size(), total_points, 
                       static_cast<double>(total_points) / scanline_clouds.size());
        }
        
        // 特徴点抽出
        pcl::PointCloud<pcl::PointXYZ>::Ptr edge_points(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr surf_points(new pcl::PointCloud<pcl::PointXYZ>());
        
        extract_features_from_scanlines(scanline_clouds, edge_points, surf_points);
        
        // ROS2メッセージに変換してパブリッシュ
        if (!edge_points->points.empty())
        {
            sensor_msgs::msg::PointCloud2 edge_msg;
            pcl::toROSMsg(*edge_points, edge_msg);
            edge_msg.header.stamp = this->now();
            edge_msg.header.frame_id = "camera_depth_optical_frame";
            edge_publisher_->publish(edge_msg);
            
            RCLCPP_INFO(this->get_logger(), "Published %zu edge points", edge_points->points.size());
        }
        
        if (!surf_points->points.empty())
        {
            sensor_msgs::msg::PointCloud2 surf_msg;
            pcl::toROSMsg(*surf_points, surf_msg);
            surf_msg.header.stamp = this->now();
            surf_msg.header.frame_id = "camera_depth_optical_frame";
            surf_publisher_->publish(surf_msg);
            
            RCLCPP_INFO(this->get_logger(), "Published %zu surface points", surf_points->points.size());
        }
    }

    // スキャンライン分割のメイン関数
    void scanline_segmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                              std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& scanline_clouds)
    {
        RCLCPP_INFO(this->get_logger(), "Starting scanline segmentation for %zu points", 
                   input_cloud->points.size());
        
        if (input_cloud->isOrganized())
        {
            organized_scanline_segmentation(input_cloud, scanline_clouds);
        }
        else
        {
            angle_based_scanline_segmentation(input_cloud, scanline_clouds);
        }
        
        RCLCPP_INFO(this->get_logger(), "Segmentation complete: %zu scanlines created", 
                   scanline_clouds.size());
    }

    // 組織化された点群用の分割
    void organized_scanline_segmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& scanline_clouds)
    {
        scanline_clouds.clear();
        
        for (size_t row = 0; row < input_cloud->height; ++row)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr scanline(new pcl::PointCloud<pcl::PointXYZ>());
            scanline->header = input_cloud->header;
            
            for (size_t col = 0; col < input_cloud->width; ++col)
            {
                const auto& point = input_cloud->at(col, row);
                
                if (is_valid_point(point))
                {
                    scanline->points.push_back(point);
                }
            }
            
            if (scanline->points.size() >= min_points_per_scanline_)
            {
                scanline->width = scanline->points.size();
                scanline->height = 1;
                scanline->is_dense = true;
                scanline_clouds.push_back(scanline);
            }
        }
    }

    // 角度ベース分割（非組織化点群用）
    void angle_based_scanline_segmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                          std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& scanline_clouds)
    {
        scanline_clouds.clear();
        
        // 将来の角度ベース分割拡張時に使用予定
        // double vertical_angle_resolution = vertical_fov_ / vertical_scans_ * M_PI / 180.0;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr scanline(new pcl::PointCloud<pcl::PointXYZ>());
        
        for (const auto& point : input_cloud->points)
        {
            if (is_valid_point(point))
            {
                scanline->points.push_back(point);
            }
        }
        
        if (scanline->points.size() >= min_points_per_scanline_)
        {
            scanline->width = scanline->points.size();
            scanline->height = 1;
            scanline->is_dense = true;
            scanline_clouds.push_back(scanline);
        }
    }

    // 曲率計算のメイン関数
    void calculate_curvature(const pcl::PointCloud<pcl::PointXYZ>::Ptr& scanline,
                            std::vector<double>& curvatures,
                            std::vector<int>& point_labels)
    {
        size_t cloud_size = scanline->points.size();
        curvatures.resize(cloud_size, 0.0);
        point_labels.resize(cloud_size, 0);
        
        for (size_t i = curvature_region_; i < cloud_size - curvature_region_; ++i)
        {
            double diff_x = 0.0, diff_y = 0.0, diff_z = 0.0;
            
            for (int j = 1; j <= curvature_region_; ++j)
            {
                diff_x += scanline->points[i - j].x + scanline->points[i + j].x;
                diff_y += scanline->points[i - j].y + scanline->points[i + j].y;
                diff_z += scanline->points[i - j].z + scanline->points[i + j].z;
            }
            
            diff_x -= 2 * curvature_region_ * scanline->points[i].x;
            diff_y -= 2 * curvature_region_ * scanline->points[i].y;
            diff_z -= 2 * curvature_region_ * scanline->points[i].z;
            
            curvatures[i] = diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
        }
    }

    // Edge特徴点抽出
    void extract_edge_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr& scanline,
                            const std::vector<double>& curvatures,
                            std::vector<int>& point_labels,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr& edge_points)
    {
        std::vector<std::pair<double, size_t>> curvature_idx;
        
        for (size_t i = curvature_region_; i < scanline->points.size() - curvature_region_; ++i)
        {
            if (point_labels[i] == 0)
            {
                curvature_idx.push_back(std::make_pair(curvatures[i], i));
            }
        }
        
        std::sort(curvature_idx.begin(), curvature_idx.end(),
                  [](const std::pair<double, size_t>& a, const std::pair<double, size_t>& b) {
                      return a.first > b.first;
                  });
        
        int edge_count = 0;
        for (const auto& pair : curvature_idx)
        {
            if (edge_count >= max_edge_points_per_scan_) break;
            
            size_t idx = pair.second;
            double curvature = pair.first;
            
            if (curvature > edge_threshold_)
            {
                point_labels[idx] = 1;
                edge_points->points.push_back(scanline->points[idx]);
                edge_count++;
                
                for (int j = 1; j <= curvature_region_; ++j)
                {
                    if (idx >= static_cast<size_t>(j) && idx + j < point_labels.size())
                    {
                        point_labels[idx - j] = -1;
                        point_labels[idx + j] = -1;
                    }
                }
            }
        }
    }

    // Surface特徴点抽出
    void extract_surface_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr& scanline,
                               const std::vector<double>& curvatures,
                               std::vector<int>& point_labels,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& surf_points)
    {
        std::vector<std::pair<double, size_t>> curvature_idx;
        
        for (size_t i = curvature_region_; i < scanline->points.size() - curvature_region_; ++i)
        {
            if (point_labels[i] == 0)
            {
                curvature_idx.push_back(std::make_pair(curvatures[i], i));
            }
        }
        
        std::sort(curvature_idx.begin(), curvature_idx.end(),
                  [](const std::pair<double, size_t>& a, const std::pair<double, size_t>& b) {
                      return a.first < b.first;
                  });
        
        int surf_count = 0;
        for (const auto& pair : curvature_idx)
        {
            if (surf_count >= max_surf_points_per_scan_) break;
            
            size_t idx = pair.second;
            double curvature = pair.first;
            
            if (curvature < surf_threshold_)
            {
                point_labels[idx] = 2;
                surf_points->points.push_back(scanline->points[idx]);
                surf_count++;
            }
        }
    }

    // 全スキャンラインから特徴点抽出
    void extract_features_from_scanlines(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& scanline_clouds,
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr& all_edge_points,
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr& all_surf_points)
    {
        all_edge_points->clear();
        all_surf_points->clear();
        
        for (const auto& scanline : scanline_clouds)
        {
            if (scanline->points.size() < 2 * static_cast<size_t>(curvature_region_)) continue;
            
            std::vector<double> curvatures;
            std::vector<int> point_labels;
            calculate_curvature(scanline, curvatures, point_labels);
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr edge_points(new pcl::PointCloud<pcl::PointXYZ>());
            extract_edge_points(scanline, curvatures, point_labels, edge_points);
            *all_edge_points += *edge_points;
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr surf_points(new pcl::PointCloud<pcl::PointXYZ>());
            extract_surface_points(scanline, curvatures, point_labels, surf_points);
            *all_surf_points += *surf_points;
        }
        
        RCLCPP_INFO(this->get_logger(), 
                   "Extracted features - Edge: %zu, Surface: %zu", 
                   all_edge_points->points.size(), all_surf_points->points.size());
    }

    // ユーティリティ関数
    bool is_valid_point(const pcl::PointXYZ& point) const
    {
        return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
               point.x != 0.0 && point.y != 0.0 && point.z != 0.0;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr edge_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surf_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SSLFeatureExtractionNode>());
    rclcpp::shutdown();
    return 0;
}