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
#include "Registration.hpp"

#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/global_control.h>
#include <tbb/info.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/task_arena.h>

#include <algorithm>
#include <cmath>
#include <kiss_icp/core/VoxelHashMap.hpp>
#include <limits>
#include <numeric>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <tuple>

using LinearSystem = std::pair<Eigen::Matrix2d, Eigen::Vector2d>;
using Correspondences = tbb::concurrent_vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>;

namespace {
constexpr double epsilon = std::numeric_limits<double>::min();

double ComputeOdometryRegularization(const Correspondences &associations,
                                     const Sophus::SE3d &odometry_initial_guess) {
    const double sum_of_squared_residuals =
        std::transform_reduce(associations.cbegin(), associations.cend(), 0.0, std::plus<double>(),
                              [&](const auto &association) {
                                  const auto &[source, target] = association;
                                  return (odometry_initial_guess * source - target).squaredNorm();
                              });
    const double N = static_cast<double>(associations.size());
    const double mean_squared_residual = sum_of_squared_residuals / N;
    const double beta = 1.0 / (mean_squared_residual + epsilon);
    return beta;
}

Correspondences DataAssociation(const std::vector<Eigen::Vector3d> &points,
                                const kiss_icp::VoxelHashMap &voxel_map,
                                const Sophus::SE3d &T,
                                const double max_correspondance_distance) {
    using points_iterator = std::vector<Eigen::Vector3d>::const_iterator;
    Correspondences correspondences;
    correspondences.reserve(points.size());
    tbb::parallel_for(
        // Range
        tbb::blocked_range<points_iterator>{points.cbegin(), points.cend()},
        [&](const tbb::blocked_range<points_iterator> &r) {
            std::for_each(r.begin(), r.end(), [&](const auto &point) {
                const auto &[closest_neighbor, distance] = voxel_map.GetClosestNeighbor(T * point);
                if (distance < max_correspondance_distance) {
                    correspondences.emplace_back(point, closest_neighbor);
                }
            });
        });
    return correspondences;
}

Eigen::Vector2d ComputePerturbation(const Correspondences &correspondences,
                                    const Sophus::SE3d &current_estimate,
                                    const double beta) {
    auto compute_jacobian_and_residual = [&](const auto &correspondence) {
        const auto &[source, target] = correspondence;
        const Eigen::Vector3d residual = current_estimate * source - target;
        Eigen::Matrix<double, 3, 2> J;
        J.col(0) = current_estimate.so3() * Eigen::Vector3d::UnitX();
        J.col(1) = current_estimate.so3() * Eigen::Vector3d(-source.y(), source.x(), 0.0);
        return std::make_tuple(J, residual);
    };

    auto sum_linear_systems = [](LinearSystem a, const LinearSystem &b) {
        a.first += b.first;
        a.second += b.second;
        return a;
    };

    using correspondence_iterator = Correspondences::const_iterator;
    auto [JTJ, JTr] = tbb::parallel_reduce(
        // Range
        tbb::blocked_range<correspondence_iterator>{correspondences.cbegin(),
                                                    correspondences.cend()},
        // Identity
        LinearSystem(Eigen::Matrix2d::Zero(), Eigen::Vector2d::Zero()),
        // 1st Lambda: Parallel computation
        [&](const tbb::blocked_range<correspondence_iterator> &r, LinearSystem J) -> LinearSystem {
            return std::transform_reduce(
                r.begin(), r.end(), J, sum_linear_systems, [&](const auto &correspondence) {
                    const auto &[J_r, residual] = compute_jacobian_and_residual(correspondence);
                    return LinearSystem(J_r.transpose() * J_r,        // JTJ
                                        J_r.transpose() * residual);  // JTr
                });
        },
        // 2nd Lambda: Parallel reduction of the private Jacboians
        sum_linear_systems);
    const double num_correspondences = static_cast<double>(correspondences.size());

    const Eigen::Matrix2d Omega = Eigen::Vector2d(beta, 0).asDiagonal();
    JTJ /= num_correspondences;
    JTr /= num_correspondences;
    JTJ += Omega;
    return -(JTJ.inverse() * JTr);
}

}  // namespace

namespace kinematic_icp {

KinematicRegistration::KinematicRegistration(const int max_num_iteration,
                                             const double convergence_criterion,
                                             const int max_num_threads,
                                             const bool use_adaptive_odometry_regularization,
                                             const double fixed_regularization)
    : max_num_iterations_(max_num_iteration),
      convergence_criterion_(convergence_criterion),
      // Only manipulate the number of threads if the user specifies something
      // greater than 0
      max_num_threads_(max_num_threads > 0 ? max_num_threads
                                           : tbb::this_task_arena::max_concurrency()),
      use_adaptive_odometry_regularization_(use_adaptive_odometry_regularization),
      fixed_regularization_(fixed_regularization) {
    // This global variable requires static duration storage to be able to
    // manipulate the max concurrency from TBB across the entire class
    static const auto tbb_control_settings = tbb::global_control(
        tbb::global_control::max_allowed_parallelism, static_cast<size_t>(max_num_threads_));
}

Sophus::SE3d KinematicRegistration::ComputeRobotMotion(const std::vector<Eigen::Vector3d> &frame,
                                                       const kiss_icp::VoxelHashMap &voxel_map,
                                                       const Sophus::SE3d &last_robot_pose,
                                                       const Sophus::SE3d &relative_wheel_odometry,
                                                       const double max_correspondence_distance) {
    Sophus::SE3d current_estimate = last_robot_pose * relative_wheel_odometry;
    if (voxel_map.Empty()) return current_estimate;

    auto motion_model = [](const Eigen::Vector2d &integrated_controls) {
        Sophus::SE3d::Tangent dx = Sophus::SE3d::Tangent::Zero();
        const double &displacement = integrated_controls(0);
        const double &theta = integrated_controls(1);
        dx(0) = displacement * std::sin(theta) / (theta + epsilon);
        dx(1) = displacement * (1.0 - std::cos(theta)) / (theta + epsilon);
        dx(5) = theta;
        return Sophus::SE3d::exp(dx);
    };
    auto correspondences =
        DataAssociation(frame, voxel_map, current_estimate, max_correspondence_distance);

    const double regularization_term = [&]() {
        if (use_adaptive_odometry_regularization_) {
            return ComputeOdometryRegularization(correspondences, current_estimate);
        } else {
            return fixed_regularization_;
        }
    }();
    // ICP-loop
    for (int j = 0; j < max_num_iterations_; ++j) {
        const auto dx = ComputePerturbation(correspondences, current_estimate, regularization_term);
        const auto delta_motion = motion_model(dx);
        current_estimate = current_estimate * delta_motion;
        // Break loop
        if (dx.norm() < convergence_criterion_) break;
        correspondences =
            DataAssociation(frame, voxel_map, current_estimate, max_correspondence_distance);
    }
    // Spit the final transformation
    return current_estimate;
}
}  // namespace kinematic_icp
