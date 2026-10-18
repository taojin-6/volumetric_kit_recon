// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/grid_layout.hpp
/// @brief The block-hash layout of the grids the examples fuse and convert
///        into; a codec player's grid is the codec's (codec/decode_grid.hpp).
///
/// One definition, so a changed default reaches every such grid.
/// Header-only and `volume`-only, without the `tsdf` tier
/// @ref create_fusion_grid brings.

#include <cstdint>

#include "volumetric_kit/recon/volume/voxel_grid.hpp"

namespace vr_example {

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;

/// @brief 8x8x8-voxel blocks in the default hash layout
///        (@ref vr::volume::kDefaultBucketSize,
///        @ref vr::volume::kDefaultMaxChain), one heap slot per bucket entry.
/// @param voxel_size   Voxel edge (metres).
/// @param trunc_dist   Truncation distance (metres).
/// @param num_buckets  Hash-bucket count; `kDefaultBucketSize * num_buckets`
///                     must fit an `int32_t`.
inline vr::volume::VoxelGridParams example_grid_params(
    float voxel_size, float trunc_dist, std::int32_t num_buckets) {
  vr::volume::VoxelGridParams grid{};
  grid.voxel_size = voxel_size;
  grid.block_size = 8;
  grid.voxels_per_block = 512;
  grid.trunc_dist = trunc_dist;
  grid.bucket_size = vr::volume::kDefaultBucketSize;
  grid.num_buckets = num_buckets;
  grid.num_blocks = grid.bucket_size * grid.num_buckets;
  grid.max_chain = vr::volume::kDefaultMaxChain;
  return grid;
}

}  // namespace vr_example
