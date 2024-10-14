// MIT License

// Copyright (c) 2024 Tiziano Guadagnino, Benedikt Mersch, Ignacio Vizzo, Cyrill
// Stachniss.

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#pragma once

#include <Eigen/Core>
#include <kiss_icp/core/Threshold.hpp>
#include <kiss_icp/core/VoxelHashMap.hpp>
#include <kiss_icp/pipeline/KissICP.hpp>
#include <sophus/se3.hpp>
#include <tuple>
#include <vector>

#include "kinematic_icp/registration/Registration.hpp"

namespace kinematic_icp::pipeline {

using Config = kiss_icp::pipeline::KISSConfig;

class KinematicICP {
public:
    using Vector3dVector = std::vector<Eigen::Vector3d>;
    using Vector3dVectorTuple = std::tuple<Vector3dVector, Vector3dVector>;

    explicit KinematicICP(const kiss_icp::pipeline::KISSConfig &config)
        : registration_(
              config.max_num_iterations, config.convergence_criterion, config.max_num_threads),
          config_(config),
          local_map_(config.voxel_size, config.max_range, config.max_points_per_voxel),
          adaptive_threshold_(config.initial_threshold, config.min_motion_th, config.max_range) {}

    Vector3dVectorTuple RegisterFrame(const std::vector<Eigen::Vector3d> &frame,
                                      const std::vector<double> &timestamps,
                                      const Sophus::SE3d &lidar_to_base,
                                      const Sophus::SE3d &relative_odometry);

    inline void SetPose(const Sophus::SE3d &pose) {
        last_pose_ = pose;
        local_map_.Clear();
        adaptive_threshold_ = kiss_icp::AdaptiveThreshold(config_.initial_threshold,
                                                          config_.min_motion_th, config_.max_range);
    };

    std::vector<Eigen::Vector3d> LocalMap() const { return local_map_.Pointcloud(); };

    const kiss_icp::VoxelHashMap &VoxelMap() const { return local_map_; };
    kiss_icp::VoxelHashMap &VoxelMap() { return local_map_; };

    const Sophus::SE3d &pose() const { return last_pose_; }
    Sophus::SE3d &pose() { return last_pose_; }

private:
    Sophus::SE3d last_pose_;
    // Kinematic module
    KinematicRegistration registration_;
    // KISS-ICP pipeline modules
    kiss_icp::pipeline::KISSConfig config_;
    kiss_icp::VoxelHashMap local_map_;
    kiss_icp::AdaptiveThreshold adaptive_threshold_;
};

}  // namespace kinematic_icp::pipeline
