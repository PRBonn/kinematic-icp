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
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// ROS
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

#include "kinematic_icp_ros/nodes/offline_node.hpp"
#include "kinematic_icp_ros/server/LidarOdometryServer.hpp"
#include "kinematic_icp_ros/utils/RosbagUtils.hpp"
#include "kinematic_icp_ros/utils/indicators.hpp"

namespace {
std::filesystem::path generateOutputFilename(const std::string &bag_filename) {
    size_t last_dot = bag_filename.find_last_of(".");
    std::string output_file = bag_filename.substr(0, last_dot);
    output_file += "_kinematic_icp_poses_tum.txt";
    std::filesystem::path output_path(output_file);
    return output_path.filename();
}
}  // namespace

namespace kinematic_icp_ros {

OfflineNode::OfflineNode(const rclcpp::NodeOptions &options) {
    node_ = rclcpp::Node::make_shared("kinematic_icp_offline_node", options);
    pcl_topic_ = node_->declare_parameter<std::string>("input");
    odometry_server_ = std::make_shared<LidarOdometryServer>(node_);

    auto bag_filename = node_->declare_parameter<std::string>("bag_filename");
    const auto poses_filename = generateOutputFilename(bag_filename);
    output_pose_file_ = std::filesystem::path(node_->declare_parameter<std::string>("output_dir"));
    output_pose_file_ /= poses_filename;
    auto tf_bridge = std::make_shared<BufferableBag::TFBridge>(node_);
    bag_multiplexer_.AddBag(BufferableBag(bag_filename, tf_bridge, pcl_topic_));
}

void OfflineNode::writePosesInTumFormat() {
    std::ofstream file(output_pose_file_);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << output_pose_file_ << std::endl;
        return;
    }

    RCLCPP_INFO_STREAM(node_->get_logger(), "Saving poses in TUM format in " << output_pose_file_);

    // Iterate over the poses and timestamps
    for (const auto &[timestamp, pose] : poses_with_timestamps_) {
        const Eigen::Vector3d &translation = pose.translation();
        const Eigen::Quaterniond &quaternion = pose.so3().unit_quaternion();
        // Write the timestamp, position, and quaternion to the file
        file << std::fixed << std::setprecision(6) << timestamp << " " << translation.x() << " "
             << translation.y() << " " << translation.z() << " " << quaternion.x() << " "
             << quaternion.y() << " " << quaternion.z() << " " << quaternion.w() << "\n";
    }

    // Close the file
    file.close();
}

void OfflineNode::Run() {
    indicators::ProgressBar bar{indicators::option::BarWidth{100},
                                indicators::option::Start{"["},
                                indicators::option::Fill{"="},
                                indicators::option::Lead{">"},
                                indicators::option::Remainder{" "},
                                indicators::option::End{"]"},
                                indicators::option::ForegroundColor{indicators::Color::cyan},
                                indicators::option::ShowPercentage{true},
                                indicators::option::ShowElapsedTime{true},
                                indicators::option::ShowRemainingTime{true},
                                indicators::option::Stream{std::cout},
                                indicators::option::MaxProgress{bag_multiplexer_.message_count()}};
    // Deserialize the next pointcloud message from the bagfiles
    auto GetNextMsg = [this] {
        const rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pcl2_serializer;
        const auto pc_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        const auto &msg = bag_multiplexer_.GetNextMessage();
        rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
        pcl2_serializer.deserialize_message(&serialized_msg, pc_msg.get());
        return pc_msg;
    };

    // This is the main blocking loop that this simulates the subscription form the online node,
    // but instead of fetching the data from a topic, it does it from the provided bagfiles
    while (rclcpp::ok() && bag_multiplexer_.IsMessageAvailable()) {
        const auto msg = GetNextMsg();
        odometry_server_->RegisterFrame(msg);
        const auto &stamp = odometry_server_->current_stamp_;
        poses_with_timestamps_.emplace_back(stamp.sec + stamp.nanosec * 1e-9,
                                            odometry_server_->kinematic_icp_->pose());
        bar.tick();
    }
    bar.mark_as_completed();
}

}  // namespace kinematic_icp_ros

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto offline_node = kinematic_icp_ros::OfflineNode(rclcpp::NodeOptions());
    offline_node.Run();
    offline_node.writePosesInTumFormat();
    rclcpp::shutdown();
    return 0;
}
