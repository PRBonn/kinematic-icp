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
#include <cmath>
#include <kiss_icp/core/Preprocessing.hpp>
#include <sophus/se3.hpp>
#include <tuple>
#include <vector>

#include "kinematic_icp/correspondence_threshold/CorrespondenceThreshold.hpp"
#include "kinematic_icp/local_map/SparseVoxelGrid.hpp"
#include "kinematic_icp/registration/Registration.hpp"

namespace kinematic_icp::pipeline {

struct Config {
    // Preprocessing
    double max_range = 100.0;
    double min_range = 0.0;
    // Mapping parameters
    double voxel_size = 1.0;
    unsigned int max_points_per_voxel = 20;
    // Derived parameter, will be computed from other parts of the configuration
    constexpr double map_resolution() const { return voxel_size / std::sqrt(max_points_per_voxel); }
    // Correspondence threshold parameters
    bool use_adaptive_threshold = true;
    double fixed_threshold = 1.0;  // <-- Ignored if use_adaptive_threshold = true

    // Registration Parameters
    int max_num_iterations = 10;
    double convergence_criterion = 0.001;
    int max_num_threads = 1;
    bool use_adaptive_odometry_regularization = true;
    double fixed_regularization = 0.0;  // <-- Ignored if use_adaptive_threshold = true

    // Motion compensation
    bool deskew = false;
};

class KinematicICP {
public:
    using Vector3dVector = std::vector<Eigen::Vector3d>;
    using Vector3dVectorTuple = std::tuple<Vector3dVector, Vector3dVector>;

    explicit KinematicICP(const Config &config)
        : registration_(config.max_num_iterations,
                        config.convergence_criterion,
                        config.max_num_threads,
                        config.use_adaptive_odometry_regularization,
                        config.fixed_regularization),
          correspondence_threshold_(config.map_resolution(),
                                    config.max_range,
                                    config.use_adaptive_threshold,
                                    config.fixed_threshold),
          config_(config),
          local_map_(config.voxel_size, config.max_range, config.max_points_per_voxel),
          preprocessor_(config.max_range, config.min_range, config.deskew, config.max_num_threads) {
    }

    Vector3dVectorTuple RegisterFrame(const std::vector<Eigen::Vector3d> &frame,
                                      const std::vector<double> &timestamps,
                                      const Sophus::SE3d &lidar_to_base,
                                      const Sophus::SE3d &relative_odometry);

    inline void SetPose(const Sophus::SE3d &pose) {
        last_pose_ = pose;
        local_map_.Clear();
        correspondence_threshold_.Reset();
    };

    std::vector<Eigen::Vector3d> LocalMap() const { return local_map_.Pointcloud(); };

    const SparseVoxelGrid &VoxelMap() const { return local_map_; };
    SparseVoxelGrid &VoxelMap() { return local_map_; };

    const Sophus::SE3d &pose() const { return last_pose_; }
    Sophus::SE3d &pose() { return last_pose_; }

protected:
    Sophus::SE3d last_pose_;
    // Kinematic Modules
    KinematicRegistration registration_;
    CorrespondenceThreshold correspondence_threshold_;
    Config config_;
    SparseVoxelGrid local_map_;
    // Kiss Module
    kiss_icp::Preprocessor preprocessor_;
};

}  // namespace kinematic_icp::pipeline
