// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/grid_layout.hpp
/// @brief The block-hash layout every example's grids share.
///
/// One definition, so a changed default reaches the grid a fuse writes and
/// the grid a codec player decodes into alike. Header-only and `volume`-only:
/// a player includes it without the `tsdf` tier @ref create_fusion_grid
/// brings.

#include <cstdint>

#include "volumetric_kit/recon/volume/voxel_grid.hpp"

namespace vr_example {

namespace vr = volumetric_kit::recon;

/// Blocks per hash bucket, and heap slots per bucket.
inline constexpr std::int32_t kExampleBucketSize = 8;

/// @brief 8x8x8-voxel blocks hashed into buckets of eight, one heap slot per
///        bucket entry, chains of up to 128.
/// @param voxel_size   Voxel edge (metres).
/// @param trunc_dist   Truncation distance (metres).
/// @param num_buckets  Hash-bucket count; `kExampleBucketSize *
///                     num_buckets` must fit an `int32_t`.
inline vr::volume::VoxelGridParams example_grid_params(
    float voxel_size, float trunc_dist, std::int32_t num_buckets) {
  vr::volume::VoxelGridParams grid{};
  grid.voxel_size = voxel_size;
  grid.block_size = 8;
  grid.voxels_per_block = 512;
  grid.trunc_dist = trunc_dist;
  grid.bucket_size = kExampleBucketSize;
  grid.num_buckets = num_buckets;
  grid.num_blocks = grid.bucket_size * grid.num_buckets;
  grid.max_chain = 128;
  return grid;
}

}  // namespace vr_example
