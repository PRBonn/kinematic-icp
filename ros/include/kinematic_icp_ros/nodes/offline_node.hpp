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

#include <memory>
#include <string>

// ROS
#include <filesystem>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <sophus/se3.hpp>

#include "kinematic_icp_ros/server/LidarOdometryServer.hpp"
#include "kinematic_icp_ros/utils/RosbagUtils.hpp"

namespace kinematic_icp_ros {

class OfflineNode {
public:
    OfflineNode() = delete;
    explicit OfflineNode(const rclcpp::NodeOptions &options);

    void writePosesInTumFormat();
    void Run();

private:
    // Common for offline/online nodes
    std::string pcl_topic_;
    std::shared_ptr<LidarOdometryServer> odometry_server_;
    rclcpp::Node::SharedPtr node_;
    // store data for the experiments
    using PoseWithStamp = std::pair<double, Sophus::SE3d>;
    std::vector<PoseWithStamp> poses_with_timestamps_;
    std::filesystem::path output_pose_file_;

    // Offline node specifics
    BagMultiplexer bag_multiplexer_;
};

}  // namespace kinematic_icp_ros
