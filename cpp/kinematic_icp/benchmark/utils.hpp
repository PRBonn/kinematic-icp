#pragma once
#include <Eigen/Dense>
#include <iostream>
#include <kiss_icp/core/VoxelUtils.hpp>
#include <random>

auto VoxelDownsample(const std::vector<Eigen::Vector3d> &frame, const double voxel_size) {
    const std::vector<Eigen::Vector3d> &frame_downsample =
        kiss_icp::VoxelDownsample(frame, voxel_size * 0.5);
    const std::vector<Eigen::Vector3d> &source =
        kiss_icp::VoxelDownsample(frame_downsample, voxel_size * 1.5);
    return std::make_tuple(source, frame_downsample);
}

struct World {
    static constexpr size_t NumSamplesPerFace = 10000;
    static constexpr double CubeSizeInMeters = 100;
    static constexpr int ScanHorizontalResolution = 1024;
    static constexpr int ScanVerticalResolution = 16;
    static constexpr int NumPositionsTrajectory = 100;
    World() {
        world_points.reserve(6 * NumSamplesPerFace);
        samplePointsOnCubeFace({1, 0, 0}, CubeSizeInMeters);    // +X face
        samplePointsOnCubeFace({-1, 0, 0}, -CubeSizeInMeters);  // -X face
        samplePointsOnCubeFace({0, 1, 0}, CubeSizeInMeters);    // +Y face
        samplePointsOnCubeFace({0, -1, 0}, -CubeSizeInMeters);  // -Y face
        samplePointsOnCubeFace({0, 0, 1}, CubeSizeInMeters);    // +Z face
        samplePointsOnCubeFace({0, 0, -1}, -CubeSizeInMeters);  // -Z face
    }
    void samplePointsOnCubeFace(const Eigen::Vector3d &normal, const double offset) {
        std::default_random_engine generator;
        std::uniform_real_distribution<float> distribution(-CubeSizeInMeters, CubeSizeInMeters);

        for (size_t i = 0; i < NumSamplesPerFace; ++i) {
            Eigen::Vector3d point;
            if (normal.x() != 0) {
                point.x() = offset;
                point.y() = distribution(generator);
                point.z() = distribution(generator);
            } else if (normal.y() != 0) {
                point.x() = distribution(generator);
                point.y() = offset;
                point.z() = distribution(generator);
            } else {
                point.x() = distribution(generator);
                point.y() = distribution(generator);
                point.z() = offset;
            }
            world_points.emplace_back(point);
        }
    }

    std::vector<Eigen::Vector3d> Generate3DScan(
        const Eigen::Vector3d scanner_pos = Eigen::Vector3d(0, 0, 0)) {
        std::vector<Eigen::Vector3d> scan_results;
        scan_results.reserve(ScanHorizontalResolution * ScanVerticalResolution);

        double h_step = 2.0 * M_PI / (ScanHorizontalResolution - 1);
        double v_step = M_PI / 2.0 / (ScanVerticalResolution - 1);

        for (int h = 0; h < ScanHorizontalResolution; ++h) {
            double h_angle = -M_PI + h * h_step;
            for (int v = 0; v < ScanVerticalResolution; ++v) {
                double v_angle = -M_PI / 4.0 + v * v_step;

                Eigen::Vector3d direction(std::sin(h_angle) * std::cos(v_angle), std::sin(v_angle),
                                          std::cos(h_angle) * std::cos(v_angle));

                // Find the closest intersection with the cube points
                double min_distance = std::numeric_limits<double>::max();
                Eigen::Vector3d closest_point;

                for (const auto &point : world_points) {
                    Eigen::Vector3d to_point = point - scanner_pos;
                    double projection = to_point.dot(direction);

                    if (projection > 0) {
                        Eigen::Vector3d closest = scanner_pos + projection * direction;
                        double distance = (point - closest).norm();

                        if (distance < 0.1 && projection < min_distance) {
                            min_distance = projection;
                            closest_point = point;
                        }
                    }
                }

                if (min_distance < std::numeric_limits<double>::max()) {
                    scan_results.emplace_back(closest_point);
                }
            }
        }
        return scan_results;
    }
    std::vector<Eigen::Vector3d> generateCircularTrajectory() {
        std::vector<Eigen::Vector3d> trajectory;
        trajectory.reserve(NumPositionsTrajectory);
        const double radius = CubeSizeInMeters * 0.25;

        for (int i = 0; i < NumPositionsTrajectory; ++i) {
            double angle = 2 * M_PI * i / NumPositionsTrajectory;
            double x = radius * std::cos(angle);
            double y = radius * std::sin(angle);
            double z = 0.0;  // Assuming a planar circle

            trajectory.emplace_back(x, y, z);
        }

        return trajectory;
    }
    std::vector<Eigen::Vector3d> world_points;
};
