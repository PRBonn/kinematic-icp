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
//
#include <Eigen/Core>
#include <array>
#include <bonxai/bonxai.hpp>
#include <sophus/se3.hpp>

namespace kinematic_icp {
// Following the Cult of Faconti
struct VoxelBlock {
    using IteratorType = std::array<Eigen::Vector3d, 20>::iterator;
    using ConstIteratorType = std::array<Eigen::Vector3d, 20>::const_iterator;
    VoxelBlock() = default;
    VoxelBlock(const VoxelBlock &other) = delete;
    VoxelBlock(VoxelBlock &&other) = default;
    void addPoint(const Eigen::Vector3d &p);

    const Eigen::Vector3d &front() const { return points_.front(); }
    inline IteratorType begin() { return points_.begin(); }
    inline ConstIteratorType cbegin() const { return points_.cbegin(); }

    inline IteratorType end() { return points_.end(); }
    inline ConstIteratorType cend() const { return points_.cend(); }

private:
    std::array<Eigen::Vector3d, 20> points_;
    std::size_t size_ = 0;
};
struct SparseVoxelGrid {
    explicit SparseVoxelGrid(const double voxel_size,
                             const double clipping_distance,
                             const unsigned int max_points_per_voxel);

    double voxel_size_;
    double max_distance_;
    unsigned int max_points_per_voxel_;
    Bonxai::VoxelGrid<VoxelBlock> map_;
};
}  // namespace kinematic_icp
