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
#include <memory>
#include <rclcpp/logging.hpp>
#include <string>

// ROS
#include <laser_geometry/laser_geometry.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "kinematic_icp_ros/nodes/online_node.hpp"
#include "kinematic_icp_ros/server/LidarOdometryServer.hpp"

namespace kinematic_icp_ros {

OnlineNode ::OnlineNode(const rclcpp::NodeOptions &options) {
    node_ = rclcpp::Node::make_shared("kinematic_icp_online_node", options);
    lidar_topic_ = node_->declare_parameter<std::string>("lidar_topic");
    odometry_server_ = std::make_shared<LidarOdometryServer>(node_);
    const bool use_2d_lidar = node_->declare_parameter<bool>("use_2d_lidar");
    if (use_2d_lidar) {
        RCLCPP_INFO_STREAM(node_->get_logger(),
                           "Started in 2D scanner mode with topic: " << lidar_topic_);
        laser_scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
            lidar_topic_, rclcpp::SystemDefaultsQoS(),
            [&](const sensor_msgs::msg::LaserScan::ConstSharedPtr &msg) {
                const sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar_msg = [&]() {
                    auto projected_scan = std::make_shared<sensor_msgs::msg::PointCloud2>();
                    laser_projector_.projectLaser(*msg, *projected_scan, -1.0,
                                                  laser_geometry::channel_option::Timestamp);
                    return projected_scan;
                }();
                odometry_server_->RegisterFrame(lidar_msg);
            });
    } else {
        RCLCPP_INFO_STREAM(node_->get_logger(),
                           "Started in 3D Lidar mode with topic: " << lidar_topic_);
        pointcloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic_, rclcpp::SystemDefaultsQoS(),
            [&](const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
                odometry_server_->RegisterFrame(msg);
            });
    }
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr OnlineNode::get_node_base_interface() {
    return node_->get_node_base_interface();
}

}  // namespace kinematic_icp_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kinematic_icp_ros::OnlineNode)
