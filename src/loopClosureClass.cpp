// ループクロージャーのクラス実装
// ROS2 port by: ry223

#include "loopClosureClass.h"
#include <iostream>

LoopClosureClass::LoopClosureClass()
    : keyframe_count_(0)
{
    last_keyframe_pose_ = Eigen::Isometry3d::Identity();
}

void LoopClosureClass::init(double keyframe_distance_threshold,
                            double loop_search_radius,
                            double icp_fitness_threshold)
{
    keyframe_distance_threshold_ = keyframe_distance_threshold;
    loop_search_radius_ = loop_search_radius;
    icp_fitness_threshold_ = icp_fitness_threshold;

    // ダウンサンプリングフィルタ設定
    downSizeFilter_.setLeafSize(0.2, 0.2, 0.2);

    std::cout << "[LoopClosure] Initialized with:" << std::endl;
    std::cout << "  - Keyframe distance threshold: " << keyframe_distance_threshold_ << "m" << std::endl;
    std::cout << "  - Loop search radius: " << loop_search_radius_ << "m" << std::endl;
    std::cout << "  - ICP fitness threshold: " << icp_fitness_threshold_ << std::endl;
}

bool LoopClosureClass::addKeyFrame(double timestamp,
                                   const Eigen::Isometry3d& pose,
                                   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud)
{
    // 最初のキーフレーム、または距離閾値を超えた場合に追加
    if (keyframes_.empty())
    {
        KeyFrame kf;
        kf.id = keyframe_count_++;
        kf.timestamp = timestamp;
        kf.pose = pose;

        // ダウンサンプリング
        downSizeFilter_.setInputCloud(pointcloud);
        downSizeFilter_.filter(*(kf.pointcloud));

        keyframes_.push_back(kf);
        last_keyframe_pose_ = pose;

        std::cout << "[LoopClosure] Added first keyframe (id: " << kf.id << ")" << std::endl;
        return true;
    }

    // 前のキーフレームからの距離計算
    double distance = (pose.translation() - last_keyframe_pose_.translation()).norm();

    if (distance > keyframe_distance_threshold_)
    {
        KeyFrame kf;
        kf.id = keyframe_count_++;
        kf.timestamp = timestamp;
        kf.pose = pose;

        // ダウンサンプリング
        downSizeFilter_.setInputCloud(pointcloud);
        downSizeFilter_.filter(*(kf.pointcloud));

        keyframes_.push_back(kf);
        last_keyframe_pose_ = pose;

        std::cout << "[LoopClosure] Added keyframe (id: " << kf.id
                  << ", total: " << keyframes_.size() << ")" << std::endl;
        return true;
    }

    return false;
}

bool LoopClosureClass::searchLoopCandidates(const KeyFrame& current_frame,
                                            std::vector<int>& candidate_ids)
{
    candidate_ids.clear();

    // 現在のフレーム位置
    Eigen::Vector3d current_pos = current_frame.pose.translation();

    // 全キーフレームを検索（最近のものを除く）
    int skip_recent = 30;  // 最近30フレームはスキップ
    for (size_t i = 0; i < keyframes_.size(); i++)
    {
        // 最近のフレームはスキップ
        if ((int)keyframes_.size() - (int)i < skip_recent)
            continue;

        // 距離計算
        double distance = (keyframes_[i].pose.translation() - current_pos).norm();

        // 検索半径内の候補を追加
        if (distance < loop_search_radius_)
        {
            candidate_ids.push_back(i);
        }
    }

    return !candidate_ids.empty();
}

bool LoopClosureClass::performICP(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& source,
                                  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& target,
                                  Eigen::Isometry3d& transform,
                                  double& fitness_score)
{
    // GICP（Generalized ICP）を使用
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> gicp;

    gicp.setMaxCorrespondenceDistance(1.0);
    gicp.setMaximumIterations(100);
    gicp.setTransformationEpsilon(1e-6);
    gicp.setEuclideanFitnessEpsilon(1e-6);

    gicp.setInputSource(source);
    gicp.setInputTarget(target);

    pcl::PointCloud<pcl::PointXYZRGB> aligned;
    gicp.align(aligned);

    fitness_score = gicp.getFitnessScore();

    if (gicp.hasConverged() && fitness_score < icp_fitness_threshold_)
    {
        Eigen::Matrix4f transformation = gicp.getFinalTransformation();
        transform = Eigen::Isometry3d(transformation.cast<double>());
        return true;
    }

    return false;
}

bool LoopClosureClass::detectAndCloseLoop(Eigen::Isometry3d& loop_transform,
                                          int& loop_id_from,
                                          int& loop_id_to)
{
    // 最低限のキーフレーム数が必要
    if (keyframes_.size() < 50)
        return false;

    // 最新のキーフレーム
    const KeyFrame& current_frame = keyframes_.back();

    // ループ候補検索
    std::vector<int> candidate_ids;
    if (!searchLoopCandidates(current_frame, candidate_ids))
        return false;

    std::cout << "[LoopClosure] Found " << candidate_ids.size()
              << " loop candidates for keyframe " << current_frame.id << std::endl;

    // 各候補についてICP実行
    for (int candidate_id : candidate_ids)
    {
        const KeyFrame& candidate_frame = keyframes_[candidate_id];

        Eigen::Isometry3d icp_transform;
        double fitness_score;

        if (performICP(current_frame.pointcloud, candidate_frame.pointcloud,
                      icp_transform, fitness_score))
        {
            std::cout << "[LoopClosure] Loop detected! Current: " << current_frame.id
                      << " <-> Candidate: " << candidate_frame.id
                      << " (fitness: " << fitness_score << ")" << std::endl;

            loop_transform = icp_transform;
            loop_id_from = current_frame.id;
            loop_id_to = candidate_frame.id;
            return true;
        }
    }

    return false;
}
