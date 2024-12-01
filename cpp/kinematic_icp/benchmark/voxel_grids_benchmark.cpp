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

static void AddPoints_BonxaiGrid(benchmark::State &state) {
    World world;
    kinematic_icp::pipeline::Config default_config;
    const auto trajectory = world.generateCircularTrajectory();
    const auto scan = world.Generate3DScan();
    kinematic_icp::SparseVoxelGrid grid(default_config.voxel_size, default_config.max_range,
                                        default_config.max_points_per_voxel);
    for (auto _ : state) {
        grid.AddPoints(scan);
    }
}

static void AddPoints_VoxelHashMap(benchmark::State &state) {
    World world;
    kinematic_icp::pipeline::Config default_config;
    const auto trajectory = world.generateCircularTrajectory();
    const auto scan = world.Generate3DScan();
    kiss_icp::VoxelHashMap grid(default_config.voxel_size, default_config.max_range,
                                default_config.max_points_per_voxel);
    for (auto _ : state) {
        grid.AddPoints(scan);
    }
}

static void Update_BonxaiGrid(benchmark::State &state) {
    World world;
    kinematic_icp::pipeline::Config default_config;
    const auto trajectory = world.generateCircularTrajectory();
    const auto scan = world.Generate3DScan();
    for (auto _ : state) {
        kinematic_icp::SparseVoxelGrid grid(default_config.voxel_size, default_config.max_range,
                                            default_config.max_points_per_voxel);
        for (size_t i = 0; i < trajectory.size(); ++i) {
            Sophus::SE3d pose;
            pose.translation() = trajectory.at(i);
            grid.Update(scan, pose);
        }
    }
}

static void Update_VoxelHashMap(benchmark::State &state) {
    World world;
    kinematic_icp::pipeline::Config default_config;
    const auto trajectory = world.generateCircularTrajectory();
    const auto scan = world.Generate3DScan();
    for (auto _ : state) {
        kiss_icp::VoxelHashMap grid(default_config.voxel_size, default_config.max_range,
                                    default_config.max_points_per_voxel);
        for (size_t i = 0; i < trajectory.size(); ++i) {
            Sophus::SE3d pose;
            pose.translation() = trajectory.at(i);
            grid.Update(scan, pose);
        }
    }
}

BENCHMARK(GetClosestNeighbor_BonxaiGrid);
BENCHMARK(GetClosestNeighbor_VoxelHashMap);
BENCHMARK(AddPoints_BonxaiGrid);
BENCHMARK(AddPoints_BonxaiGrid);
BENCHMARK(Update_VoxelHashMap);
BENCHMARK(Update_VoxelHashMap);
// Run the benchmark
BENCHMARK_MAIN();
