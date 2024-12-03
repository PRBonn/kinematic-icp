#include <algorithm>
#include <kinematic_icp/pipeline/KinematicICP.hpp>
#include <sophus/se3.hpp>
#include <vector>

#include "utils.hpp"

int main() {
    World world;
    kinematic_icp::pipeline::Config config;
    config.convergence_criterion = -1.0;
    config.max_num_threads = 1;
    config.use_adaptive_threshold = false;
    config.fixed_threshold = 100.0;
    config.use_adaptive_odometry_regularization = false;
    config.fixed_regularization = 0.0;
    kinematic_icp::pipeline::KinematicICP pipeline(config);
    const auto scan = world.Generate3DScan();
    std::vector<Eigen::Vector3d> transformed_scan(scan.size());
    const auto trajectory = world.generateCircularTrajectory();
    for (size_t i = 1; i < trajectory.size(); ++i) {
        std::transform(scan.cbegin(), scan.cend(), transformed_scan.begin(),
                       [&](const auto &p) { return p + trajectory.at(i); });
        pipeline.VoxelMap().AddPoints(scan);
    }
}
