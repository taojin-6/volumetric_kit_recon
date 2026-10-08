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

#include <cmath>
#include <cstdint>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/recon/volume/voxel_grid.hpp"

namespace vr_example {

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;

/// Blocks per hash bucket, and heap slots per bucket.
inline constexpr std::int32_t kExampleBucketSize = 8;

/// @return The truncation band every example defaults to: four voxels, so
///         the band's width in voxels does not move with @p voxel_size.
inline float default_trunc(float voxel_size) { return 4.0f * voxel_size; }

/// @brief The rule every example's voxel edge meets: above 0, with a finite
///        @ref default_trunc band.
/// @return OK, or InvalidArgument naming `--voxel`, the flag that sets it.
inline vkc::Status check_voxel(float voxel_size) {
  if (!(voxel_size > 0.0f)) {
    return vkc::Status::invalid_argument("--voxel must be > 0");
  }
  if (!std::isfinite(default_trunc(voxel_size))) {
    return vkc::Status::invalid_argument("--voxel is too large");
  }
  return {};
}

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
