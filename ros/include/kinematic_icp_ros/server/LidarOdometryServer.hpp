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

#include "kinematic_icp/pipeline/KinematicICP.hpp"

// ROS 2 C
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// ROS 2 C++
#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sophus/se3.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>

// STL
#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace kinematic_icp_ros {

class LidarOdometryServer {
public:
    /// LidarOdometryServer constructor
    LidarOdometryServer() = delete;
    explicit LidarOdometryServer(rclcpp::Node::SharedPtr node);

    /// Register new frame
    void RegisterFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
    std::unique_ptr<kinematic_icp::pipeline::KinematicICP> kinematic_icp_;
    builtin_interfaces::msg::Time current_stamp_;

private:
    /// Stream the estimated pose to ROS
    void PublishOdometryMsg(const Sophus::SE3d &pose, const Sophus::SE3d::Tangent &velocity);

    /// Stream the debugging point clouds for visualization (if required)
    void PublishClouds(const std::vector<Eigen::Vector3d> frame,
                       const std::vector<Eigen::Vector3d> keypoints);

    // Temporal initializaiton strattegy until we convert the odometry server to life cycle
    void InitializePoseAndExtrinsic(const std::string &lidar_frame_id);

    /// Tools for broadcasting TFs.
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
    std::chrono::milliseconds tf_timeout_;
    bool publish_odom_tf_;
    bool invert_odom_tf_;
    bool publish_debug_clouds_;
    Sophus::SE3d sensor_to_base_footprint_;

    /// Data publishers.
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr frame_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr kpoints_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr voxel_grid_pub_;

    /// SetPose service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_pose_srv_;

    /// Internal messages, models the current state that will be published by this node.
    nav_msgs::msg::Odometry odom_msg_;
    geometry_msgs::msg::TransformStamped tf_msg_;

    /// Global/map coordinate frame.
    std::string lidar_odom_frame_{"odom_lidar"};
    std::string wheel_odom_frame_{"odom"};
    std::string base_frame_{"base_link"};
    // TF frame initialization flag
    bool initialize_odom_node;

    rclcpp::Node::SharedPtr node_;
};

}  // namespace kinematic_icp_ros
