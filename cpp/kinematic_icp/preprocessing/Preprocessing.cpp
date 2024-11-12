#include "Preprocessing.hpp"

#include <unordered_map>

#include "StampedPointCloud.hpp"

namespace {
Eigen::Vector3i PointToVoxel(const Eigen::Vector3d &point, const double voxel_size) {
    return Eigen::Vector3i(static_cast<int>(std::floor(point.x() / voxel_size)),
                           static_cast<int>(std::floor(point.y() / voxel_size)),
                           static_cast<int>(std::floor(point.z() / voxel_size)));
}
}  // namespace
template <>
struct std::hash<Eigen::Vector3i> {
    std::size_t operator()(const Eigen::Vector3i &voxel) const {
        const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
        return (vec[0] * 73856093 ^ vec[1] * 19349669 ^ vec[2] * 83492791);
    }
};  // namespace

namespace kinematic_icp {

StampedPointCloud VoxelDownsample(const StampedPointCloud &pcd, const double voxel_size) {
    std::unordered_map<Eigen::Vector3i, StampedPoint> grid;
    grid.reserve(pcd.size());
    std::for_each(pcd.cbegin(), pcd.cend(), [&](const auto &point) {
        const auto voxel = PointToVoxel(point.coordinates, voxel_size);
        if (!grid.contains(voxel)) grid.insert({voxel, point});
    });
    StampedPointCloud pcd_dowsampled;
    pcd_dowsampled.reserve(grid.size());
    std::for_each(grid.cbegin(), grid.cend(), [&](const auto &voxel_and_point) {
        pcd_dowsampled.emplace_back(voxel_and_point.second);
    });
    return pcd_dowsampled;
}

StampedPointCloud ClipMinMaxRange(const StampedPointCloud &pcd,
                                  const double max_range,
                                  const double min_range) {
    StampedPointCloud inliers;
    inliers.reserve(pcd.size());
    std::copy_if(pcd.cbegin(), pcd.cend(), std::back_inserter(inliers), [&](const auto &pt) {
        const double norm = pt.coordinates.norm();
        return norm < max_range && norm > min_range;
    });
    inliers.shrink_to_fit();
    return inliers;
}

StampedPointCloud DeSkew(const StampedPointCloud &pcd, const Sophus::SE3d &relative_motion) {
    const Sophus::SE3d::Tangent delta_pose = relative_motion.log();
    StampedPointCloud compensated_pcd(pcd.size());
    std::transform(pcd.cbegin(), pcd.cend(), compensated_pcd.begin(),
                   [&](const auto &stamped_point) {
                       const auto interpolated_pose =
                           Sophus::SE3d::exp((stamped_point.timestamp - 1.0) * delta_pose);
                       return interpolated_pose * stamped_point;
                   });
    return compensated_pcd;
}

}  // namespace kinematic_icp
