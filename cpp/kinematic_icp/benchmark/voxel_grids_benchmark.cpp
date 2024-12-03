#include <benchmark/benchmark.h>

#include <kinematic_icp/local_map/SparseVoxelGrid.hpp>
#include <kinematic_icp/pipeline/KinematicICP.hpp>
#include <kiss_icp/core/VoxelHashMap.hpp>
#include <sophus/se3.hpp>

#include "utils.hpp"

static void GetClosestNeighbor_BonxaiGrid(benchmark::State &state) {
    World world;
    kinematic_icp::pipeline::Config default_config;
    kinematic_icp::SparseVoxelGrid grid(default_config.voxel_size, default_config.max_range,
                                        default_config.max_points_per_voxel);
    grid.AddPoints(world.world_points);
    const auto scan = world.Generate3DScan();
    const auto &[source, _] = VoxelDownsample(scan, default_config.voxel_size);
    for (auto _ : state) {
        for (const auto &p : source) {
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
    const auto &[source, _] = VoxelDownsample(scan, default_config.voxel_size);
    for (auto _ : state) {
        for (const auto &p : source) {
            const auto &[dist, nn] = grid.GetClosestNeighbor(p);
        }
    }
}

static void AddPoints_BonxaiGrid(benchmark::State &state) {
    World world;
    kinematic_icp::pipeline::Config default_config;
    for (auto _ : state) {
        kinematic_icp::SparseVoxelGrid grid(default_config.voxel_size, default_config.max_range,
                                            default_config.max_points_per_voxel);
        grid.AddPoints(world.world_points);
    }
}

static void AddPoints_VoxelHashMap(benchmark::State &state) {
    World world;
    kinematic_icp::pipeline::Config default_config;
    for (auto _ : state) {
        kiss_icp::VoxelHashMap grid(default_config.voxel_size, default_config.max_range,
                                    default_config.max_points_per_voxel);
        grid.AddPoints(world.world_points);
    }
}

static void Update_BonxaiGrid(benchmark::State &state) {
    World world;
    kinematic_icp::pipeline::Config default_config;
    const auto trajectory = world.generateCircularTrajectory();
    const auto scan = world.Generate3DScan();
    const auto &[_, frame_downsample] = VoxelDownsample(scan, default_config.voxel_size);
    kinematic_icp::SparseVoxelGrid grid(default_config.voxel_size, default_config.max_range,
                                        default_config.max_points_per_voxel);
    grid.AddPoints(world.world_points);
    for (auto _ : state) {
        for (const auto &pose : trajectory) {
            grid.Update(frame_downsample, pose);
        }
    }
}

static void Update_VoxelHashMap(benchmark::State &state) {
    World world;
    kinematic_icp::pipeline::Config default_config;
    const auto trajectory = world.generateCircularTrajectory();
    const auto scan = world.Generate3DScan();
    const auto &[_, frame_downsample] = VoxelDownsample(scan, default_config.voxel_size);
    kiss_icp::VoxelHashMap grid(default_config.voxel_size, default_config.max_range,
                                default_config.max_points_per_voxel);
    grid.AddPoints(world.world_points);
    for (auto _ : state) {
        for (const auto &pose : trajectory) {
            grid.Update(frame_downsample, pose);
        }
    }
}

BENCHMARK(GetClosestNeighbor_BonxaiGrid);
BENCHMARK(GetClosestNeighbor_VoxelHashMap);
BENCHMARK(AddPoints_BonxaiGrid);
BENCHMARK(AddPoints_VoxelHashMap);
BENCHMARK(Update_BonxaiGrid);
BENCHMARK(Update_VoxelHashMap);
// Run the benchmark
BENCHMARK_MAIN();
