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
#include <tf2/time.h>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tuple>
#include <vector>

namespace kinematic_icp_ros::utils {

using StampType = builtin_interfaces::msg::Time;

struct TimeStampHandler {
    std::tuple<StampType, StampType, std::vector<double>> ProcessTimestamps(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
    // From double to ROS TimeStamp
    inline StampType toStamp(const double &time_in_seconds) const {
        return rclcpp::Time(tf2::durationFromSec(time_in_seconds).count());
    };
    inline double toTime(const StampType &stamp) const {
        return rclcpp::Time(stamp).nanoseconds() * 1e-9;
    };

    StampType last_processed_stamp_;
};

}  // namespace kinematic_icp_ros::utils
