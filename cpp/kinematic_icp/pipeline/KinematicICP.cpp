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
#include "KinematicICP.hpp"

#include <Eigen/Core>
#include <kiss_icp/core/Deskew.hpp>
#include <kiss_icp/core/Preprocessing.hpp>
#include <kiss_icp/core/Registration.hpp>
#include <kiss_icp/core/VoxelHashMap.hpp>
#include <vector>

namespace {
auto transform_points(const std::vector<Eigen::Vector3d> &points, const Sophus::SE3d &pose) {
    std::vector<Eigen::Vector3d> points_transformed(points.size());
    std::transform(points.cbegin(), points.cend(), points_transformed.begin(),
                   [&](const auto &point) { return pose * point; });
    return points_transformed;
}

auto Voxelize(const std::vector<Eigen::Vector3d> &frame, const double voxel_size) {
    const std::vector<Eigen::Vector3d> &frame_downsample =
        kiss_icp::VoxelDownsample(frame, voxel_size * 0.5);
    const std::vector<Eigen::Vector3d> &source =
        kiss_icp::VoxelDownsample(frame_downsample, voxel_size * 1.5);
    return std::make_tuple(source, frame_downsample);
}
}  // namespace
namespace kinematic_icp::pipeline {

KinematicICP::Vector3dVectorTuple KinematicICP::RegisterFrame(
    const std::vector<Eigen::Vector3d> &frame,
    const std::vector<double> &timestamps,
    const Sophus::SE3d &lidar_to_base,
    const Sophus::SE3d &relative_odometry) {
    const auto &deskew_frame = [&]() -> std::vector<Eigen::Vector3d> {
        if (!config_.deskew || timestamps.empty()) return frame;
        return kiss_icp::DeSkewScan(frame, timestamps,
                                    lidar_to_base.inverse() * relative_odometry * lidar_to_base);
    }();
    const auto &deskew_frame_in_base = transform_points(deskew_frame, lidar_to_base);
    // Preprocess the input cloud
    const auto &cropped_frame =
        kiss_icp::Preprocess(deskew_frame_in_base, config_.max_range, config_.min_range);

    // Voxelize
    const auto &[source, frame_downsample] = Voxelize(cropped_frame, config_.voxel_size);

    // Get adaptive_threshold
    const double &tau = adaptive_threshold_.ComputeThreshold();

    // Run ICP
    const auto new_pose = registration_.ComputeRobotMotion(source,             // frame
                                                           local_map_,         // voxel_map
                                                           last_pose_,         // last_pose
                                                           relative_odometry,  // robot_motion
                                                           tau);  // max_correspondence_dist

    // Compute the difference between the prediction and the actual estimate
    const auto model_deviation = (last_pose_ * relative_odometry).inverse() * new_pose;

    // Update step: threshold, local map and the last pose
    adaptive_threshold_.UpdateModelError(model_deviation);
    local_map_.Update(frame_downsample, new_pose);
    last_pose_ = new_pose;

    // Return the (deskew) input raw scan (frame) and the points used for
    // registration (source)
    return {deskew_frame_in_base, source};
}
}  // namespace kinematic_icp::pipeline
