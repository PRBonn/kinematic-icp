#include "StampedPointCloud.hpp"

namespace kinematic_icp {
StampedPoint::StampedPoint(const Eigen::Vector3d &point, const double stamp)
    : coordinates(point), timestamp(stamp){};

}
