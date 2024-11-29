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
#include "kinematic_icp_ros/utils/TimeStampHandler.hpp"

#include <tf2/time.h>

#include <optional>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vector>

namespace {

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;
using Header = std_msgs::msg::Header;

std::optional<PointField> GetTimestampField(const PointCloud2::ConstSharedPtr msg) {
    PointField timestamp_field;
    for (const auto &field : msg->fields) {
        if ((field.name == "t" || field.name == "timestamp" || field.name == "time" ||
             field.name == "stamps")) {
            timestamp_field = field;
        }
    }
    if (timestamp_field.count) return timestamp_field;
    RCLCPP_WARN_ONCE(rclcpp::get_logger("kinematic_icp_ros"),
                     "Field 't', 'timestamp', 'time', or 'stamps'  does not exist. "
                     "Disabling scan deskewing");
    return {};
}

auto ExtractTimestampsFromMsg(const PointCloud2::ConstSharedPtr msg,
                              const PointField &timestamp_field) {
    auto number_of_digits_decimal_part = [](const auto &stamp) {
        const uint64_t number_of_seconds = static_cast<uint64_t>(std::round(stamp));
        return number_of_seconds > 0 ? std::floor(std::log10(number_of_seconds) + 1) : 1;
    };
    auto extract_timestamps =
        [&]<typename T>(sensor_msgs::PointCloud2ConstIterator<T> &&it) -> std::vector<double> {
        const size_t n_points = msg->height * msg->width;
        std::vector<double> timestamps;
        timestamps.reserve(n_points);
        for (size_t i = 0; i < n_points; ++i, ++it) {
            double stampd = static_cast<double>(*it);
            // If the number of digits is greater than 10 (which is the maximum number of digits
            // that can be represented with a 32 bits integer), the stamp is in nanoseconds instead
            // of seconds, perform conversion
            if (number_of_digits_decimal_part(stampd) > 10) {
                stampd *= 1e-9;
            }
            timestamps.emplace_back(stampd);
        }
        return timestamps;
    };

    // According to the type of the timestamp == type, return a PointCloud2ConstIterator<type>
    using sensor_msgs::PointCloud2ConstIterator;
    if (timestamp_field.datatype == PointField::UINT32) {
        return extract_timestamps(PointCloud2ConstIterator<uint32_t>(*msg, timestamp_field.name));
    } else if (timestamp_field.datatype == PointField::FLOAT32) {
        return extract_timestamps(PointCloud2ConstIterator<float>(*msg, timestamp_field.name));
    } else if (timestamp_field.datatype == PointField::FLOAT64) {
        return extract_timestamps(PointCloud2ConstIterator<double>(*msg, timestamp_field.name));
    }

    // timestamp type not supported, please open an issue :)
    throw std::runtime_error("timestamp field type not supported");
}

std::vector<double> GetTimestamps(const PointCloud2::ConstSharedPtr msg) {
    auto timestamp_field = GetTimestampField(msg);
    if (!timestamp_field.has_value()) return {};

    // Extract timestamps from cloud_msg
    std::vector<double> timestamps = ExtractTimestampsFromMsg(msg, timestamp_field.value());

    return timestamps;
}
}  // namespace

namespace kinematic_icp_ros::utils {

std::tuple<StampType, StampType, std::vector<double>> TimeStampHandler::ProcessTimestamps(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    std::vector<double> timestamps = GetTimestamps(msg);
    const auto &[min_it, max_it] = std::minmax_element(timestamps.cbegin(), timestamps.cend());
    const StampType msg_stamp = msg->header.stamp;
    const StampType begin_stamp = last_processed_stamp_;
    StampType end_stamp = msg_stamp;
    if (max_it != timestamps.cend()) {
        const double &max_stamp_in_seconds = *max_it;
        const double &min_stamp_in_seconds = *min_it;
        const double msg_stamp_in_seconds = this->toTime(msg_stamp);

        // Check if stamping happens and the beginning or the end of scan
        const bool is_stamped_at_the_beginning =
            std::abs(msg_stamp_in_seconds - max_stamp_in_seconds) > 1e-8;
        if (is_stamped_at_the_beginning) {
            // begin-stamping -> add scan duration to the stamp
            const auto scan_duration =
                tf2::durationFromSec(max_stamp_in_seconds - min_stamp_in_seconds);
            end_stamp = StampType(rclcpp::Time(end_stamp) + scan_duration);
        }

        // Normalize timestamps
        std::transform(timestamps.cbegin(), timestamps.cend(), timestamps.begin(),
                       [&](const auto &timestamp) {
                           return (timestamp - min_stamp_in_seconds) /
                                  (max_stamp_in_seconds - min_stamp_in_seconds);
                       });
    }
    last_processed_stamp_ = end_stamp;
    return std::make_tuple(begin_stamp, end_stamp, timestamps);
}

}  // namespace kinematic_icp_ros::utils
