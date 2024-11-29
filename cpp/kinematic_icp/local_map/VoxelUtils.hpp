// MIT License
//
// Copyright (c) 2024 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
#pragma once

#include <Eigen/Core>
#include <cmath>
#include <vector>

namespace kinematic_icp {

struct Voxel {
    int32_t x;
    int32_t y;
    int32_t z;

    inline bool operator==(const Voxel &other) const {
        return x == other.x && y == other.y && z == other.z;
    }

    inline bool operator!=(const Voxel &other) const { return !(*this == other); }

    inline Voxel &operator+=(const Voxel &other) {
        this->x += other.x;
        this->y += other.y;
        this->z += other.z;
        return *this;
    }
    friend Voxel operator+(Voxel lhs, const Voxel &rhs) {
        lhs += rhs;
        return lhs;
    }
};

inline Voxel PointToVoxel(const Eigen::Vector3d &point, const double voxel_size) {
    return {static_cast<int>(std::floor(point.x() / voxel_size)),
            static_cast<int>(std::floor(point.y() / voxel_size)),
            static_cast<int>(std::floor(point.z() / voxel_size))};
}

/// Voxelize a point cloud keeping the original coordinates
std::vector<Eigen::Vector3d> VoxelDownsample(const std::vector<Eigen::Vector3d> &frame,
                                             const double voxel_size);

}  // namespace kinematic_icp

template <>
struct std::hash<kinematic_icp::Voxel> {
    std::size_t operator()(const kinematic_icp::Voxel &voxel) const {
        return (static_cast<uint32_t>(voxel.x) * 73856093 ^
                static_cast<uint32_t>(voxel.y) * 19349669 ^
                static_cast<uint32_t>(voxel.z) * 83492791);
    }
};
