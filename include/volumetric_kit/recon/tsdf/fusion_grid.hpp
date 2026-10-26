// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file tsdf/fusion_grid.hpp
/// @brief The grid a fusing caller starts from: @ref volume::block_grid_params'
///        layout, carrying what @ref Fuser writes, its table sized to the
///        voxel and capped to the device.

#include <cstdint>
#include <optional>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/recon/core/fwd.hpp"
#include "volumetric_kit/recon/tsdf/export.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"

namespace volumetric_kit::recon::tsdf {

/// @brief The hash-bucket count @ref create_fusion_grid starts at.
///
/// @p num_buckets when given. Otherwise 16,384 at 1 cm and coarser, scaled by
/// (1 cm / @p voxel_size)^2 finer, as a surface's blocks are, so a first set
/// fits the table it starts in rather than filling it. The scale stops at 4x
/// (5 mm): a finer grid starts there and the fuser grows it, rather than
/// taking gigabytes up front.
///
/// Either is capped at the ceiling the grid can never grow past either
/// (@ref volume::VoxelBlockGrid::max_num_buckets): every attribute array
/// within one storage binding of @p max_storage_buffer_range, and a block
/// pointer within int32. That is 8,192 at the 128 MiB Vulkan guarantees,
/// which lavapipe reports, and above every default at the 4 GiB desktop and
/// Apple GPUs report. A caller's count it lowers is logged as a warning.
/// @param voxel_size                Voxel edge (metres).
/// @param max_storage_buffer_range  The device's limit
///                                  (`core::max_storage_buffer_range`).
/// @param num_buckets               The caller's start, if any.
/// @return The count.
VR_TSDF_API std::int32_t fusion_buckets(
    float voxel_size, VkDeviceSize max_storage_buffer_range,
    std::optional<std::int32_t> num_buckets = std::nullopt);

/// @brief A grid to fuse into: @ref volume::block_grid_params' layout,
///        carrying float `tsdf` and `weight` and the `uint32` packed `color`
///        @ref Fuser writes.
///
/// The fuser grows the table, so @p num_buckets sets where it starts, not
/// what it holds.
///
/// @code
/// VKC_ASSIGN(volume::VoxelBlockGrid grid,
///            tsdf::create_fusion_grid(device, allocator, 0.01f, 0.04f));
/// @endcode
/// @param device       The recon device (must outlive the grid).
/// @param allocator    Its allocator (must outlive the grid).
/// @param voxel_size   Voxel edge (metres).
/// @param trunc_dist   Truncation distance (metres).
/// @param num_buckets  Initial hash-bucket count, which @ref fusion_buckets
///                     defaults when unset and caps to fit @p device.
/// @return The grid, empty, or @ref volume::VoxelBlockGrid::create's error.
VR_TSDF_API core::Result<volume::VoxelBlockGrid> create_fusion_grid(
    core::Device& device, core::Allocator& allocator, float voxel_size,
    float trunc_dist, std::optional<std::int32_t> num_buckets = std::nullopt);

}  // namespace volumetric_kit::recon::tsdf
