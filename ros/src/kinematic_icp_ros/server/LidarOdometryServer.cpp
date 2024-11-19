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
#include "kinematic_icp_ros/server/LidarOdometryServer.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <sophus/se3.hpp>
#include <utility>

#include "kinematic_icp/pipeline/KinematicICP.hpp"
#include "kinematic_icp_ros/utils/RosUtils.hpp"

// ROS 2 headers
#include <rcl/time.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace {
// Time Type aliases to simplify code
using milliseconds = std::chrono::milliseconds;
using seconds = std::chrono::duration<long double>;
using std::chrono::duration_cast;
}  // namespace

namespace kinematic_icp_ros {

using namespace utils;

LidarOdometryServer::LidarOdometryServer(rclcpp::Node::SharedPtr node) : node_(node) {
    lidar_odom_frame_ = node->declare_parameter<std::string>("lidar_odom_frame", lidar_odom_frame_);
    wheel_odom_frame_ = node->declare_parameter<std::string>("wheel_odom_frame", wheel_odom_frame_);
    base_frame_ = node->declare_parameter<std::string>("base_frame", base_frame_);
    publish_odom_tf_ = node->declare_parameter<bool>("publish_odom_tf", publish_odom_tf_);
    invert_odom_tf_ = node->declare_parameter<bool>("invert_odom_tf", invert_odom_tf_);
    tf_timeout_ =
        duration_cast<milliseconds>(seconds(node->declare_parameter<double>("tf_timeout", 0.0)));

    kinematic_icp::pipeline::Config config;
    // Preprocessing
    config.max_range = node->declare_parameter<double>("max_range", config.max_range);
    config.min_range = node->declare_parameter<double>("min_range", config.min_range);
    // Mapping parameters
    config.voxel_size = node->declare_parameter<double>("voxel_size", config.voxel_size);
    config.max_points_per_voxel =
        node->declare_parameter<int>("max_points_per_voxel", config.max_points_per_voxel);
    // Correspondence threshold parameters
    config.use_adaptive_threshold =
        node->declare_parameter<bool>("use_adaptive_threshold", config.use_adaptive_threshold);
    config.fixed_threshold =
        node->declare_parameter<double>("fixed_threshold", config.fixed_threshold);
    // Registration Parameters
    config.max_num_iterations =
        node->declare_parameter<int>("max_num_iterations", config.max_num_iterations);
    config.convergence_criterion =
        node->declare_parameter<double>("convergence_criterion", config.convergence_criterion);
    config.max_num_threads =
        node->declare_parameter<int>("max_num_threads", config.max_num_threads);
    config.use_adaptive_odometry_regularization = node->declare_parameter<bool>(
        "use_adaptive_odometry_regularization", config.use_adaptive_odometry_regularization);
    config.fixed_regularization =
        node->declare_parameter<double>("fixed_regularization", config.fixed_regularization);
    // Motion compensation
    config.deskew = node->declare_parameter<bool>("deskew", config.deskew);
    if (config.max_range < config.min_range) {
        RCLCPP_WARN(node_->get_logger(),
                    "[WARNING] max_range is smaller than min_range, settng min_range to 0.0");
        config.min_range = 0.0;
    }

    // Construct the main KISS-ICP odometry node
    kinematic_icp_ = std::make_unique<kinematic_icp::pipeline::KinematicICP>(config);

    // Initialize publishers
    rclcpp::QoS qos((rclcpp::SystemDefaultsQoS().keep_last(1).durability_volatile()));
    odom_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>("lidar_odometry", qos);
    frame_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("frame", qos);
    kpoints_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("keypoints", qos);
    map_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("local_map", qos);
    voxel_grid_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("voxel_grid", qos);

    set_pose_srv_ = node_->create_service<std_srvs::srv::Trigger>(
        "set_pose", [&](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            const auto pose = LookupTransform(wheel_odom_frame_, base_frame_, tf2_buffer_);
            RCLCPP_WARN_STREAM(node_->get_logger(), "Resetting KISS-ICP pose:\n"
                                                        << pose.matrix() << "\n");
            kinematic_icp_->SetPose(pose);
            response->success = true;
        });

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

    // Initialize the odometry and tf msg. Since tf by design does not allow for having two frames
    // in the tree with a different parent, to publish the odom_lidar frame, we need to add a child
    // frame (we can't provide odom_lidar -> base_footprint, which is what we estimate internally).
    // Therefore we publish the inverted transformation (base_footprint -> odom_lidar), which turns
    // to be an implemantation detail as tf lookups are transparent and one can query for the
    // odom_lidar -> base_footprint transform, the tf will invert it again on the fly.
    if (invert_odom_tf_) {
        tf_msg_.header.frame_id = base_frame_;
        tf_msg_.child_frame_id = lidar_odom_frame_;
    } else {
        tf_msg_.header.frame_id = lidar_odom_frame_;
        tf_msg_.child_frame_id = base_frame_;
    }

    // fixed covariancecovariance
    const auto position_covariance = node->declare_parameter<double>("position_covariance", 0.1);
    const auto orientation_covariance =
        node->declare_parameter<double>("orientation_covariance", 0.1);
    odom_msg_.header.frame_id = lidar_odom_frame_;
    odom_msg_.child_frame_id = base_frame_;
    odom_msg_.pose.covariance.fill(0.0);
    odom_msg_.pose.covariance[0] = position_covariance;
    odom_msg_.pose.covariance[7] = position_covariance;
    odom_msg_.pose.covariance[35] = orientation_covariance;
    odom_msg_.twist.covariance.fill(0);
    odom_msg_.twist.covariance[0] = position_covariance;
    odom_msg_.twist.covariance[7] = position_covariance;
    odom_msg_.twist.covariance[35] = orientation_covariance;
}

void LidarOdometryServer::InitializePoseAndExtrinsic(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    if (!tf2_buffer_->_frameExists(wheel_odom_frame_) || !tf2_buffer_->_frameExists(base_frame_) ||
        !tf2_buffer_->_frameExists(msg->header.frame_id)) {
        return;
    }

    // Independent from the service, start the ROS node from a known state
    const auto pose = LookupTransform(wheel_odom_frame_, base_frame_, tf2_buffer_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "Resetting KISS-ICP pose:\n" << pose.matrix() << "\n");
    kinematic_icp_->SetPose(pose);

    try {
        sensor_to_base_footprint_ = tf2::transformToSophus(
            tf2_buffer_->lookupTransform(base_frame_, msg->header.frame_id, tf2::TimePointZero));
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
        return;
    }

    // Initialization finished
    RCLCPP_INFO(node_->get_logger(), "KISS-ICP ROS 2 odometry node initialized");
    timestamps_handler_.last_processed_stamp_ = msg->header.stamp;
    initialize_odom_node = true;
}

void LidarOdometryServer::RegisterFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    if (!initialize_odom_node) {
        InitializePoseAndExtrinsic(msg);
    }

    // Buffer the last state This will be used for computing the veloicty
    const auto last_pose = kinematic_icp_->pose();
    const auto &[begin_odom_query, end_odom_query, timestamps] =
        timestamps_handler_.ProcessTimestamps(msg);

    // Get the initial guess from the wheel odometry
    const auto delta =
        LookupDeltaTransform(base_frame_, begin_odom_query, base_frame_, end_odom_query,
                             wheel_odom_frame_, tf_timeout_, tf2_buffer_);

    // Run kinematic ICP
    if (delta.log().norm() > 1e-3) {
        const auto &extrinsic = sensor_to_base_footprint_;
        const auto points = PointCloud2ToEigen(msg, {});
        const auto &[frame, kpoints] =
            kinematic_icp_->RegisterFrame(points, timestamps, extrinsic, delta);
        PublishClouds(frame, kpoints);
    }

    // Compute velocities, use the elapsed time between the current msg and the last received
    const double elapsed_time =
        timestamps_handler_.toTime(end_odom_query) - timestamps_handler_.toTime(begin_odom_query);
    const Sophus::SE3d::Tangent delta_twist = (last_pose.inverse() * kinematic_icp_->pose()).log();
    const Sophus::SE3d::Tangent velocity = delta_twist / elapsed_time;

    // Spit the current estimated pose to ROS pc_out_msgs handling the desired target frame
    PublishOdometryMsg(kinematic_icp_->pose(), velocity);
}

void LidarOdometryServer::PublishOdometryMsg(const Sophus::SE3d &pose,
                                             const Sophus::SE3d::Tangent &velocity) {
    // Broadcast over the tf tree
    if (publish_odom_tf_) {
        tf_msg_.transform = [&]() {
            if (invert_odom_tf_) return tf2::sophusToTransform(pose.inverse());
            return tf2::sophusToTransform(pose);
        }();
        tf_msg_.header.stamp = timestamps_handler_.last_processed_stamp_;
        tf_broadcaster_->sendTransform(tf_msg_);
    }

    // publish odometry msg
    odom_msg_.pose.pose = tf2::sophusToPose(pose);
    odom_msg_.twist.twist.linear.x = velocity[0];
    odom_msg_.twist.twist.angular.z = velocity[5];
    odom_msg_.header.stamp = timestamps_handler_.last_processed_stamp_;
    odom_publisher_->publish(odom_msg_);
}

void LidarOdometryServer::PublishClouds(const std::vector<Eigen::Vector3d> frame,
                                        const std::vector<Eigen::Vector3d> keypoints) {
    // For re-publishing the input frame and keypoints, we do it in the LiDAR coordinate frames
    std_msgs::msg::Header lidar_header;
    lidar_header.frame_id = base_frame_;
    lidar_header.stamp = timestamps_handler_.last_processed_stamp_;

    // The internal map representation is in the lidar_odom_frame_
    std_msgs::msg::Header map_header;
    map_header.frame_id = lidar_odom_frame_;
    map_header.stamp = timestamps_handler_.last_processed_stamp_;

    // Check for subscriptions before publishing to avoid unnecesary CPU usage
    if (frame_publisher_->get_subscription_count() > 0) {
        frame_publisher_->publish(std::move(EigenToPointCloud2(frame, lidar_header)));
    }
    if (kpoints_publisher_->get_subscription_count() > 0) {
        kpoints_publisher_->publish(std::move(EigenToPointCloud2(keypoints, lidar_header)));
    }
    if (map_publisher_->get_subscription_count() > 0) {
        map_publisher_->publish(
            std::move(EigenToPointCloud2(kinematic_icp_->LocalMap(), map_header)));
    }
}

}  // namespace kinematic_icp_ros
