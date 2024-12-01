#pragma once
#include <Eigen/Dense>
#include <iostream>
#include <random>

struct World {
    static constexpr size_t NumSamplesPerFace = 10000;
    static constexpr double CubeSizeInMeters = 100;
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
            Eigen::Vector3f point;
            if (normal.x() != 0) {
                point = {offset, distribution(generator), distribution(generator)};
            } else if (normal.y() != 0) {
                point = {distribution(generator), offset, distribution(generator)};
            } else {
                point = {distribution(generator), distribution(generator), offset};
            }
            world_points.emplace_back(point);
        }
    }
    std::vector<Eigen::Vector3d> world_points;
};
