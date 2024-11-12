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

#include "kinematic_icp/preprocessing/Preprocessing.hpp"
#include "kinematic_icp/preprocessing/StampedPointCloud.hpp"

namespace kinematic_icp::pipeline {

KinematicICP::StampedPointCloudTuple KinematicICP::RegisterFrame(
    const StampedPointCloud &stamped_frame,
    const Sophus::SE3d &lidar_to_base,
    const Sophus::SE3d &relative_odometry) {
    // Preprocess the input cloud
    const auto &cropped_frame =
        ClipMinMaxRange(stamped_frame, config_.max_range, config_.min_range);

    const auto &mapping_frame = VoxelDownsample(cropped_frame, config_.voxel_size * 0.5);
    const auto &registration_frame = VoxelDownsample(mapping_frame, config_.voxel_size * 1.5);
    const auto &deskewed_registration_frame =
        config_.deskew ? DeSkew(registration_frame, relative_odometry) : registration_frame;
    // Get adaptive_threshold
    const double &tau = correspondence_threshold_.ComputeThreshold();

    // Run ICP
    const auto new_pose = registration_.ComputeRobotMotion(deskewed_registration_frame,  // frame
                                                           local_map_,         // voxel_map
                                                           last_pose_,         // last_pose
                                                           relative_odometry,  // robot_motion
                                                           tau);  // max_correspondence_dist

    // Compute the difference between the prediction and the actual estimate
    const auto model_deviation = (last_pose_ * relative_odometry).inverse() * new_pose;

    // Update step: threshold, local map and the last pose
    correspondence_threshold_.UpdateModelError(model_deviation);
    const auto estimated_relative_motion = last_pose_.inverse() * new_pose;
    const auto &deskewed_mapping_frame =
        config_.deskew ? DeSkew(mapping_frame, estimated_relative_motion) : mapping_frame;
    local_map_.Update(deskewed_mapping_frame, new_pose);
    last_pose_ = new_pose;

    // Return the (deskew) input raw scan (frame) and the points used for
    // registration (source)
    return {deskewed_mapping_frame, deskewed_registration_frame};
}
}  // namespace kinematic_icp::pipeline
