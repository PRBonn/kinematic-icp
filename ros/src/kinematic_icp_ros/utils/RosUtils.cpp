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
#include "kinematic_icp_ros/utils/RosUtils.hpp"

#include <cmath>
#include <cstdint>

namespace kinematic_icp_ros::utils {

std::optional<PointField> GetTimestampField(const PointCloud2::ConstSharedPtr msg) {
    PointField timestamp_field;
    for (const auto &field : msg->fields) {
        if ((field.name == "t" || field.name == "timestamp" || field.name == "time")) {
            timestamp_field = field;
        }
    }
    if (timestamp_field.count) return timestamp_field;
    RCLCPP_WARN_ONCE(rclcpp::get_logger("kinematic_icp_ros"),
                     "Field 't', 'timestamp', or 'time'  does not exist. "
                     "Disabling scan deskewing");
    return {};
}

// Normalize timestamps from 0.0 to 1.0
std::vector<double> NormalizeTimestamps(const std::vector<double> &timestamps) {
    const auto [min_it, max_it] = std::minmax_element(timestamps.cbegin(), timestamps.cend());
    const double min_timestamp = min_it != timestamps.cend() ? *min_it : 0.0;
    const double max_timestamp = max_it != timestamps.cend() ? *max_it : 1.0;

    std::vector<double> timestamps_normalized(timestamps.size());
    std::transform(timestamps.cbegin(), timestamps.cend(), timestamps_normalized.begin(),
                   [&](const auto &timestamp) {
                       return (timestamp - min_timestamp) / (max_timestamp - min_timestamp);
                   });
    return timestamps_normalized;
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

std::vector<Eigen::Vector3d> PointCloud2ToEigen(const PointCloud2::ConstSharedPtr msg,
                                                const Sophus::SE3d &T) {
    std::vector<Eigen::Vector3d> points;
    points.reserve(msg->height * msg->width);
    sensor_msgs::PointCloud2ConstIterator<float> iter_xyz(*msg, "x");
    for (size_t i = 0; i < msg->height * msg->width; ++i, ++iter_xyz) {
        points.emplace_back(T * Eigen::Vector3d{iter_xyz[0], iter_xyz[1], iter_xyz[2]});
    }
    return points;
}

std::unique_ptr<PointCloud2> EigenToPointCloud2(const std::vector<Eigen::Vector3d> &points,
                                                const Header &header) {
    auto msg = std::make_unique<PointCloud2>();
    msg->header = header;

    auto &offset = msg->point_step;
    offset = addPointField(*msg, "x", 1, PointField::FLOAT32, offset);
    offset = addPointField(*msg, "y", 1, PointField::FLOAT32, offset);
    offset = addPointField(*msg, "z", 1, PointField::FLOAT32, offset);

    // Resize the
    sensor_msgs::PointCloud2Modifier modifier(*msg);
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_xyz(*msg, "x");
    for (size_t i = 0; i < points.size(); i++, ++iter_xyz) {
        const Eigen::Vector3d &point = points[i];
        iter_xyz[0] = point.x();
        iter_xyz[1] = point.y();
        iter_xyz[2] = point.z();
    }
    return msg;
}

visualization_msgs::msg::Marker VoxelsToMarker(const std::vector<Eigen::Vector3i> &voxels,
                                               const double voxel_size,
                                               const Header &header) {
    visualization_msgs::msg::Marker voxel_grid;
    voxel_grid.header = header;
    voxel_grid.ns = "voxel_grid";
    voxel_grid.id = 0;  // Always same voxel grid, this should modify the whole structure
    voxel_grid.type = visualization_msgs::msg::Marker::LINE_LIST;
    voxel_grid.action = visualization_msgs::msg::Marker::MODIFY;

    // TODO(God): Convert this marker into a custom vizualization type so we can modify these 5
    // parameters from rviz instead of the ROS node!
    voxel_grid.scale.x = 0.01;  // Line width
    voxel_grid.color.a = 0.1;   // alpha
    voxel_grid.color.r = 1.0;   // red
    voxel_grid.color.g = 1.0;   // green
    voxel_grid.color.b = 1.0;   // blue

    // Compute the 8 vertices of each voxel in grid coordinates
    const auto corners = [&voxels]() {
        std::vector<std::array<Eigen::Vector3i, 8>> voxel_corners;
        voxel_corners.reserve(voxels.size());
        for (const auto &voxel : voxels) {
            // clang-format off
            voxel_corners.emplace_back(std::array<Eigen::Vector3i, 8>{
                voxel + Eigen::Vector3i{0, 0, 0},
                voxel + Eigen::Vector3i{1, 0, 0},
                voxel + Eigen::Vector3i{1, 1, 0},
                voxel + Eigen::Vector3i{0, 1, 0},
                voxel + Eigen::Vector3i{0, 0, 1},
                voxel + Eigen::Vector3i{1, 0, 1},
                voxel + Eigen::Vector3i{1, 1, 1},
                voxel + Eigen::Vector3i{0, 1, 1}
            });
            // clang-format on
        }
        return voxel_corners;
    }();

    // Lambda function to add an edge between two voxel vertices
    auto AddEdge = [&](const Eigen::Vector3i &vertex1, const Eigen::Vector3i &vertex2) {
        geometry_msgs::msg::Point point1, point2;
        point1.x = vertex1.x() * voxel_size;
        point1.y = vertex1.y() * voxel_size;
        point1.z = vertex1.z() * voxel_size;
        voxel_grid.points.push_back(point1);

        point2.x = vertex2.x() * voxel_size;
        point2.y = vertex2.y() * voxel_size;
        point2.z = vertex2.z() * voxel_size;
        voxel_grid.points.push_back(point2);
    };

    // Add edges for each voxel to create the line list
    for (const auto &corner : corners) {
        AddEdge(corner[0], corner[1]);
        AddEdge(corner[1], corner[2]);
        AddEdge(corner[2], corner[3]);
        AddEdge(corner[3], corner[0]);
        AddEdge(corner[4], corner[5]);
        AddEdge(corner[5], corner[6]);
        AddEdge(corner[6], corner[7]);
        AddEdge(corner[7], corner[4]);
        AddEdge(corner[0], corner[4]);
        AddEdge(corner[1], corner[5]);
        AddEdge(corner[2], corner[6]);
        AddEdge(corner[3], corner[7]);
    }

    return voxel_grid;
}
}  // namespace kinematic_icp_ros::utils
