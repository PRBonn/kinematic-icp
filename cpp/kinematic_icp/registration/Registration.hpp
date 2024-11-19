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
#include <kiss_icp/core/VoxelHashMap.hpp>
#include <sophus/se3.hpp>
#include <vector>

namespace kinematic_icp {

struct KinematicRegistration {
    explicit KinematicRegistration(const int max_num_iteration,
                                   const double convergence_criterion,
                                   const int max_num_threads,
                                   const bool use_adaptive_odometry_regularization,
                                   const double fixed_regularization);

    Sophus::SE3d ComputeRobotMotion(const std::vector<Eigen::Vector3d> &frame,
                                    const kiss_icp::VoxelHashMap &voxel_map,
                                    const Sophus::SE3d &last_robot_pose,
                                    const Sophus::SE3d &relative_wheel_odometry,
                                    const double max_correspondence_distance);

    int max_num_iterations_;
    double convergence_criterion_;
    int max_num_threads_;
    bool use_adaptive_odometry_regularization_;
    double fixed_regularization_;
};
}  // namespace kinematic_icp
