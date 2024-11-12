#pragma once
#include "StampedPointCloud.hpp"

namespace kinematic_icp {

StampedPointCloud VoxelDownsample(const StampedPointCloud &pcd, const double voxel_size);

StampedPointCloud ClipMinMaxRange(const StampedPointCloud &pcd,
                                  const double max_range,
                                  const double min_range);

StampedPointCloud DeSkew(const StampedPointCloud &pcd, const Sophus::SE3d &relative_motion);

}  // namespace kinematic_icp
