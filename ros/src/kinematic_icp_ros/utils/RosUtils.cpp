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
