#pragma once
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <vector>

namespace kinematic_icp {

struct StampedPoint {
    StampedPoint() = default;
    StampedPoint(const Eigen::Vector3d &point, const double stamp)
        : coordinates(point), timestamp(stamp){};

    Eigen::Vector3d coordinates = Eigen::Vector3d::Zero();
    double timestamp = 0.0;
};

inline StampedPoint operator*(const Sophus::SE3d &pose, const StampedPoint &point) {
    const Eigen::Vector3d transformed_coordinates = pose * point.coordinates;
    return StampedPoint(transformed_coordinates, point.timestamp);
}

using StampedPointCloud = std::vector<StampedPoint>;

inline StampedPointCloud TransformPointCloud(const Sophus::SE3d &pose,
                                             const StampedPointCloud &pcd) {
    StampedPointCloud transformed(pcd.size());
    std::transform(pcd.cbegin(), pcd.cend(), transformed.begin(),
                   [&](const auto &p) { return pose * p; });
    return transformed;
}
}  // namespace kinematic_icp
