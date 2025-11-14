// ループクロージャーのC++ヘッダーファイル
// ROS2 port by: ry223

#ifndef _LOOP_CLOSURE_CLASS_H_
#define _LOOP_CLOSURE_CLASS_H_

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

//eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

//std
#include <vector>
#include <deque>

// キーフレーム構造体
struct KeyFrame
{
    int id;
    double timestamp;
    Eigen::Isometry3d pose;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud;

    KeyFrame() : id(0), timestamp(0.0)
    {
        pose = Eigen::Isometry3d::Identity();
        pointcloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    }
};

class LoopClosureClass
{
public:
    LoopClosureClass();

    // 初期化
    void init(double keyframe_distance_threshold, double loop_search_radius, double icp_fitness_threshold);

    // キーフレームの追加
    bool addKeyFrame(double timestamp, const Eigen::Isometry3d& pose,
                     const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud);

    // ループ検出と修正
    bool detectAndCloseLoop(Eigen::Isometry3d& loop_transform, int& loop_id_from, int& loop_id_to);

    // キーフレーム取得
    std::vector<KeyFrame>& getKeyFrames() { return keyframes_; }

private:
    // パラメータ
    double keyframe_distance_threshold_;  // キーフレーム追加の距離閾値
    double loop_search_radius_;           // ループ検索半径
    double icp_fitness_threshold_;        // ICP適合度閾値

    // キーフレームデータ
    std::vector<KeyFrame> keyframes_;
    Eigen::Isometry3d last_keyframe_pose_;
    int keyframe_count_;

    // ループクロージャー検出
    pcl::VoxelGrid<pcl::PointXYZRGB> downSizeFilter_;

    // ループ候補検索
    bool searchLoopCandidates(const KeyFrame& current_frame, std::vector<int>& candidate_ids);

    // ICP マッチング
    bool performICP(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& source,
                   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& target,
                   Eigen::Isometry3d& transform,
                   double& fitness_score);
};

#endif // _LOOP_CLOSURE_CLASS_H_
