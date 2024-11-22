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

#include "SparseVoxelGrid.hpp"

#include <Eigen/Core>
#include <bonxai/bonxai.hpp>
#include <sophus/se3.hpp>

#include "bonxai/grid_coord.hpp"

namespace {
static std::array<Bonxai::CoordT, 27> shifts{
    Bonxai::CoordT{-1, -1, -1}, Bonxai::CoordT{-1, -1, 0}, Bonxai::CoordT{-1, -1, 1},
    Bonxai::CoordT{-1, 0, -1},  Bonxai::CoordT{-1, 0, 0},  Bonxai::CoordT{-1, 0, 1},
    Bonxai::CoordT{-1, 1, -1},  Bonxai::CoordT{-1, 1, 0},  Bonxai::CoordT{-1, 1, 1},

    Bonxai::CoordT{0, -1, -1},  Bonxai::CoordT{0, -1, 0},  Bonxai::CoordT{0, -1, 1},
    Bonxai::CoordT{0, 0, -1},   Bonxai::CoordT{0, 0, 0},   Bonxai::CoordT{0, 0, 1},
    Bonxai::CoordT{0, 1, -1},   Bonxai::CoordT{0, 1, 0},   Bonxai::CoordT{0, 1, 1},

    Bonxai::CoordT{1, -1, -1},  Bonxai::CoordT{1, -1, 0},  Bonxai::CoordT{1, -1, 1},
    Bonxai::CoordT{1, 0, -1},   Bonxai::CoordT{1, 0, 0},   Bonxai::CoordT{1, 0, 1},
    Bonxai::CoordT{1, 1, -1},   Bonxai::CoordT{1, 1, 0},   Bonxai::CoordT{1, 1, 1}};
}

namespace kinematic_icp {
void VoxelBlock::addPoint(const Eigen::Vector3d &p) {
    points_[size_++] = p;
    if (size_ > VoxelBlock::MAX_SIZE) {
        throw std::runtime_error("VoxelBlock| size is too big, want to fix somehow");
    }
}

SparseVoxelGrid::SparseVoxelGrid(const double voxel_size,
                                 const double clipping_distance,
                                 const unsigned int max_points_per_voxel)
    : voxel_size_(voxel_size),
      clipping_distance_(clipping_distance),
      max_points_per_voxel_(max_points_per_voxel),
      map_(voxel_size),
      accessor_(map_.createAccessor()) {}

std::tuple<Eigen::Vector3d, double> SparseVoxelGrid::GetClosestNeighbor(
    const Eigen::Vector3d &query) const {
    Eigen::Vector3d closest_neighbor = Eigen::Vector3d::Zero();
    double closest_distance = std::numeric_limits<double>::max();
    const auto const_accessor = map_.createConstAccessor();
    const Bonxai::CoordT query_voxel = map_.posToCoord(query);
    std::for_each(shifts.cbegin(), shifts.cend(), [&](const Bonxai::CoordT &voxel_coordinates) {
        const VoxelBlock *voxel_points = const_accessor.value(query_voxel + voxel_coordinates);
        if (voxel_points != nullptr) {
            const Eigen::Vector3d &neighbor =
                *std::min_element(voxel_points->cbegin(), voxel_points->cend(),
                                  [&](const auto &lhs, const auto &rhs) {
                                      return (lhs - query).norm() < (rhs - query).norm();
                                  });
            double distance = (neighbor - query).norm();
            if (distance < closest_distance) {
                closest_neighbor = neighbor;
                closest_distance = distance;
            }
        }
    });
    return std::make_tuple(closest_neighbor, closest_distance);
}

void SparseVoxelGrid::AddPoints(const std::vector<Eigen::Vector3d> &points) {
    const double map_resolution = std::sqrt(voxel_size_ * voxel_size_ / max_points_per_voxel_);
    std::for_each(points.cbegin(), points.cend(), [&](const Eigen::Vector3d &p) {
        const auto voxel_coordinates = map_.posToCoord(p.x(), p.y(), p.z());
        VoxelBlock *voxel_points = accessor_.value(voxel_coordinates, true);
        if (voxel_points->size() == max_points_per_voxel_ ||
            std::any_of(voxel_points->cbegin(), voxel_points->cend(), [&](const auto &voxel_point) {
                return (voxel_point - p).norm() < map_resolution;
            })) {
            return;
        }
        voxel_points->addPoint(p);
    });
}

void SparseVoxelGrid::RemovePointsFarFromLocation(const Eigen::Vector3d &origin) {
    auto remove_voxel = [this, &origin](VoxelBlock &block, const Bonxai::CoordT &coordinate) {
        if ((block.front() - origin).norm() >= clipping_distance_) {
            accessor_.setCellOff(coordinate);
        }
    };
    map_.forEachCell(remove_voxel);
    // map_.releaseUnusedMemory();
}

void SparseVoxelGrid::Update(const std::vector<Eigen::Vector3d> &points, const Sophus::SE3d &pose) {
    std::vector<Eigen::Vector3d> points_transformed(points.size());
    std::transform(points.cbegin(), points.cend(), points_transformed.begin(),
                   [&](const auto &point) { return pose * point; });
    const Eigen::Vector3d &origin = pose.translation();
    AddPoints(points_transformed);
    RemovePointsFarFromLocation(origin);
}

std::vector<Eigen::Vector3d> SparseVoxelGrid::Pointcloud() const {
    std::vector<Eigen::Vector3d> point_cloud;
    point_cloud.reserve(map_.activeCellsCount() * max_points_per_voxel_);
    map_.forEachCell([&point_cloud, this](VoxelBlock &block, const auto &) {
        point_cloud.insert(point_cloud.end(), block.cbegin(), block.cend());
    });
    return point_cloud;
}

}  // namespace kinematic_icp
