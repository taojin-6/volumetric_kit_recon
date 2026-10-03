// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file volume/hierarchical_field_view.hpp
/// @brief Borrowed, cell-centered dyadic TSDF leaves for device consumers.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "volumetric_kit/recon/core/buffer.hpp"
#include "volumetric_kit/recon/volume/hash_types.hpp"
#include "volumetric_kit/recon/volume/voxel_grid.hpp"

namespace volumetric_kit::recon::volume {

/// @brief A node of a dyadic forest whose leaves hold 8 cubed samples.
///
/// Level zero is finest. Node bounds are the half-open box
/// `[coord * 8 * 2^level, (coord + 1) * 8 * 2^level)` in finest cells.
/// Sample `i` lies at `(coord * 8 + i + 0.5) * 2^level` finest cells.
/// The world origin is zero. Mirrors the shader's scalar-layout node.
struct HierarchicalNode {
  Vec3i coord{};               ///< Block coordinate at this node's level.
  std::int32_t ptr = -1;       ///< Sample-array base, or -1 if absent.
  std::uint32_t level = 0;     ///< Power-of-two spacing exponent.
  std::uint32_t children = 0;  ///< Zero for a leaf; otherwise first child + 1.
};
static_assert(sizeof(HierarchicalNode) == 24, "hierarchical node ABI");
static_assert(offsetof(HierarchicalNode, ptr) == 12, "hierarchical node ABI");
static_assert(offsetof(HierarchicalNode, level) == 16, "hierarchical node ABI");
static_assert(offsetof(HierarchicalNode, children) == 20,
              "hierarchical node ABI");

/// @brief Borrowed device storage of a cell-centered adaptive field.
///
/// All buffers use STORAGE_BUFFER and belong to the consumer's VkDevice.
/// Calls are externally serialized: meshing reads the field, while integration
/// writes its sample arrays; no topology mutation may overlap either call.
/// Buffer objects remain alive throughout the call. The root hash uses
/// `root_grid`; its block pointer divided by 512 is a root node index.
/// Children occupy eight consecutive nodes in x + 2*y + 4*z order.
/// The leaf list contains each active leaf exactly once. Sample arrays have
/// node_capacity * 512 elements, including unused/internal slots. TSDF is in
/// metres; color, when present, is packed sRGB RGBA8 as in VoxelBlockGrid.
/// No consumer may retain this view across a producer move or destruction.
struct HierarchicalFieldView {
  const Buffer* root_hash = nullptr;     ///< Root HashEntry array.
  const Buffer* nodes = nullptr;         ///< HierarchicalNode array.
  const Buffer* leaf_indices = nullptr;  ///< uint32 active node indices.
  const Buffer* tsdf = nullptr;          ///< Float distances in metres.
  const Buffer* weight = nullptr;        ///< Float observation weights.
  const Buffer* color = nullptr;         ///< Optional packed encoded color.
  VoxelGridParams root_grid{};           ///< Hash geometry at max_level.
  std::uint32_t node_capacity = 0;       ///< Nodes addressable by this view.
  std::uint32_t leaf_count = 0;          ///< Valid entries in leaf_indices.
  std::uint32_t max_level = 0;           ///< The root spacing exponent.
  float finest_voxel_size = 0.0f;        ///< Metres per finest cell edge.
  std::uint64_t generation = 0;          ///< Producer's topology generation.
  const std::uint64_t* live_generation = nullptr;  ///< Borrowed live counter.

  /// @return Whether topology and borrowed buffer addresses are still current.
  /// @warning Producer-thread only; not synchronized with topology mutations.
  bool is_current() const noexcept {
    return live_generation != nullptr && generation != 0 &&
           *live_generation == generation;
  }

  /// @brief Validate the host-checkable shape and buffer extents.
  /// @return OK, or InvalidArgument for stale topology, invalid geometry,
  ///         missing buffers/usage, or undersized storage. Node contents and
  ///         duplicate-free leaf membership remain the producer's contract.
  Status validate() const {
    if (!is_current()) {
      return Status::invalid_argument("HierarchicalFieldView: stale view");
    }
    VR_TRY(root_grid.validate());
    if (root_grid.block_size != 8 || max_level > 3 || node_capacity == 0 ||
        node_capacity >
            std::uint32_t(std::numeric_limits<std::int32_t>::max()) / 512u ||
        leaf_count > node_capacity ||
        std::uint32_t(root_grid.num_blocks) > node_capacity ||
        !std::isfinite(root_grid.voxel_size) ||
        !std::isfinite(root_grid.trunc_dist) ||
        !std::isfinite(finest_voxel_size) || !(finest_voxel_size > 0.0f) ||
        root_grid.voxel_size != std::ldexp(finest_voxel_size, int(max_level))) {
      return Status::invalid_argument(
          "HierarchicalFieldView: invalid geometry");
    }
    const auto check = [](const Buffer* b, VkDeviceSize bytes) {
      return b != nullptr && b->valid() && b->size() >= bytes &&
             (b->usage() & VK_BUFFER_USAGE_STORAGE_BUFFER_BIT) != 0;
    };
    const VkDeviceSize samples = VkDeviceSize(node_capacity) * 512u;
    if (!check(root_hash,
               VkDeviceSize(root_grid.num_blocks) * sizeof(HashEntry)) ||
        !check(nodes, VkDeviceSize(node_capacity) * sizeof(HierarchicalNode)) ||
        !check(leaf_indices,
               VkDeviceSize(leaf_count) * sizeof(std::uint32_t)) ||
        !check(tsdf, samples * sizeof(float)) ||
        !check(weight, samples * sizeof(float)) ||
        (color != nullptr && !check(color, samples * sizeof(std::uint32_t)))) {
      return Status::invalid_argument("HierarchicalFieldView: invalid storage");
    }
    return {};
  }
};

}  // namespace volumetric_kit::recon::volume
