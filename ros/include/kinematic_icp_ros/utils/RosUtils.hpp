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

// tf2
#include <tf2_ros/buffer.h>

#include <Eigen/Core>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

// ROS 2
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sophus/se3.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace tf2 {

inline geometry_msgs::msg::Transform sophusToTransform(const Sophus::SE3d &T) {
    geometry_msgs::msg::Transform t;
    t.translation.x = T.translation().x();
    t.translation.y = T.translation().y();
    t.translation.z = T.translation().z();

    Eigen::Quaterniond q(T.so3().unit_quaternion());
    t.rotation.x = q.x();
    t.rotation.y = q.y();
    t.rotation.z = q.z();
    t.rotation.w = q.w();

    return t;
}

inline geometry_msgs::msg::Pose sophusToPose(const Sophus::SE3d &T) {
    geometry_msgs::msg::Pose t;
    t.position.x = T.translation().x();
    t.position.y = T.translation().y();
    t.position.z = T.translation().z();

    Eigen::Quaterniond q(T.so3().unit_quaternion());
    t.orientation.x = q.x();
    t.orientation.y = q.y();
    t.orientation.z = q.z();
    t.orientation.w = q.w();

    return t;
}

inline Sophus::SE3d transformToSophus(const geometry_msgs::msg::TransformStamped &transform) {
    const auto &t = transform.transform;
    return Sophus::SE3d(
        Sophus::SE3d::QuaternionType(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z),
        Sophus::SE3d::Point(t.translation.x, t.translation.y, t.translation.z));
}

inline Sophus::SE3d poseToSophus(const geometry_msgs::msg::Pose &pose) {
    const auto &T = pose;
    return Sophus::SE3d(Sophus::SE3d::QuaternionType(T.orientation.w, T.orientation.x,
                                                     T.orientation.y, T.orientation.z),
                        Sophus::SE3d::Point(T.position.x, T.position.y, T.position.z));
}

}  // namespace tf2

namespace kinematic_icp_ros::utils {
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;
using Header = std_msgs::msg::Header;

inline Sophus::SE3d LookupTransform(
    const std::string &target_frame,
    const std::string &source_frame,
    const std::unique_ptr<tf2_ros::Buffer> &tf2_buffer,
    const rclcpp::Time &time = tf2_ros::toRclcpp(tf2::TimePointZero)) {
    try {
        auto tf = tf2_buffer->lookupTransform(target_frame, source_frame, time);
        return tf2::transformToSophus(tf);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(rclcpp::get_logger("LookupTransform"), "%s", ex.what());
        return {};
    }
}

inline Sophus::SE3d LookupDeltaTransform(const std::string &target_frame,
                                         const rclcpp::Time &target_time,
                                         const std::string &source_frame,
                                         const rclcpp::Time &source_time,
                                         const std::string &fixed_frame,
                                         const rclcpp::Duration timeout,
                                         const std::unique_ptr<tf2_ros::Buffer> &tf2_buffer) {
    try {
        auto tf = tf2_buffer->lookupTransform(target_frame, target_time, source_frame, source_time,
                                              fixed_frame, timeout);
        return tf2::transformToSophus(tf);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(rclcpp::get_logger("LookupTransform"), "%s", ex.what());
        return {};
    }
}

std::optional<PointField> GetTimestampField(const PointCloud2::ConstSharedPtr msg);

std::vector<double> NormalizeTimestamps(const std::vector<double> &timestamps);

auto ExtractTimestampsFromMsg(const PointCloud2::ConstSharedPtr msg,
                              const PointField &timestamp_field);

std::vector<double> GetTimestamps(const PointCloud2::ConstSharedPtr msg);

std::vector<Eigen::Vector3d> PointCloud2ToEigen(const PointCloud2::ConstSharedPtr msg,
                                                const Sophus::SE3d &T = {});

std::unique_ptr<PointCloud2> EigenToPointCloud2(const std::vector<Eigen::Vector3d> &points,
                                                const Header &header);

visualization_msgs::msg::Marker VoxelsToMarker(const std::vector<Eigen::Vector3i> &voxels,
                                               const double voxel_size,
                                               const Header &header);
}  // namespace kinematic_icp_ros::utils
