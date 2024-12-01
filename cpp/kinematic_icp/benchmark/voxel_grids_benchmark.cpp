#include <benchmark/benchmark.h>

#include <kinematic_icp/local_map/SparseVoxelGrid.hpp>
#include <kinematic_icp/pipeline/KinematicICP.hpp>
#include <kiss_icp/core/VoxelHashMap.hpp>
#include <sophus/se3.hpp>

#include "kinematic_icp/registration/Registration.hpp"
#include "utils.hpp"

static void GetClosestNeighbor_BonxaiGrid(benchmark::State &state) {
    World world;
    kinematic_icp::pipeline::Config default_config;
    kinematic_icp::SparseVoxelGrid grid(default_config.voxel_size, default_config.max_range,
                                        default_config.max_points_per_voxel);
    grid.AddPoints(world.world_points);
    const auto scan = world.Generate3DScan();
    for (auto _ : state) {
        for (const auto &p : scan) {
            const auto &[dist, nn] = grid.GetClosestNeighbor(p);
        }
    }
}

static void GetClosestNeighbor_VoxelHashMap(benchmark::State &state) {
    World world;
    kinematic_icp::pipeline::Config default_config;
    kiss_icp::VoxelHashMap grid(default_config.voxel_size, default_config.max_range,
                                default_config.max_points_per_voxel);
    grid.AddPoints(world.world_points);
    const auto scan = world.Generate3DScan();
    for (auto _ : state) {
        for (const auto &p : scan) {
            const auto &[dist, nn] = grid.GetClosestNeighbor(p);
        }
    }
}

// static void RegisterFrame_BonxaiGrid(benchmark::State &state) {
//     World world;
//     kinematic_icp::pipeline::Config default_config;
//     kinematic_icp::SparseVoxelGrid grid(default_config.voxel_size, default_config.max_range,
//                                         default_config.max_points_per_voxel);
//     grid.AddPoints(world.world_points);
//     kinematic_icp::KinematicRegistration registration(10, -1, 1, true, 1.0);
//     const auto trajectory = world.generateCircularTrajectory();
//     std::vector<std::vector<Eigen::Vector3d>> scans;
//     std::transform()
//     for (auto _ : state) {
//         for (const auto &p : scan) {
//             const auto &[dist, nn] = grid.GetClosestNeighbor(p);
//         }
//     }
// }

BENCHMARK(GetClosestNeighbor_BonxaiGrid);
BENCHMARK(GetClosestNeighbor_VoxelHashMap);
// Run the benchmark
BENCHMARK_MAIN();
