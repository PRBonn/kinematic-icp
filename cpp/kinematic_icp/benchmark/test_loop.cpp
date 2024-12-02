#include <kinematic_icp/pipeline/KinematicICP.hpp>
#include <sophus/se3.hpp>

#include "utils.hpp"

int main() {
    World world;
    kinematic_icp::pipeline::Config config;
    config.convergence_criterion = -1.0;
    config.max_num_threads = 1;
    config.use_adaptive_odometry_regularization = false;
    config.fixed_regularization = 0.0;
    kinematic_icp::pipeline::KinematicICP pipeline(config);
    const auto trajectory = world.generateCircularTrajectory();
    const auto extrinsic = Sophus::SE3d();
    pipeline.RegisterFrame(world.Generate3DScan(), std::vector<double>(), extrinsic,
                           Sophus::SE3d());
    auto pose = pipeline.pose();
    for (size_t i = 1; i < trajectory.size(); ++i) {
        const auto scan = world.Generate3DScan(trajectory.at(i));
        const auto delta = trajectory.at(i) - trajectory.at(i - 1);
        Sophus::SE3d odom;
        odom.translation() = delta;
        pipeline.RegisterFrame(scan, std::vector<double>(), extrinsic, odom);
    }
}
