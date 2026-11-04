// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/tsdf/fusion_grid.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <string>

#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/grid/volume/voxel_grid.hpp"
#include "volumetric_kit/recon/core/log.hpp"
#include "volumetric_kit/recon/tsdf/source.hpp"

namespace volumetric_kit::recon::tsdf {
namespace {

// What every fusion grid carries: the attributes Fuser writes, the source
// byte (tsdf/source.hpp) last.
constexpr volumetric_kit::grid::volume::AttributeSpec kFusionAttributes[] = {
    {volumetric_kit::grid::volume::kTsdfAttribute, sizeof(float)},
    {volumetric_kit::grid::volume::kWeightAttribute, sizeof(float)},
    {volumetric_kit::grid::volume::kColorAttribute, sizeof(std::uint32_t)},
    {kSourceAttribute, sizeof(std::uint8_t)}};

// A filter on the source byte at `min_weight` that lets every sensor of each
// listed class answer: the bytes `base | (index + 1)`, for each class's base
// byte (0 for the fixed cameras, kMobileBit for anchored fill).
volumetric_kit::grid::volume::CastFilter source_filter(
    float min_weight, std::initializer_list<std::uint8_t> classes) {
  volumetric_kit::grid::volume::CastFilter filter;
  filter.min_weight = min_weight;
  filter.attribute = kSourceAttribute;
  filter.allow.fill(0u);
  for (const std::uint8_t base : classes) {
    for (std::uint32_t index = 0; index < kMaxSourcesPerClass; ++index) {
      const std::uint32_t v = base | (index + 1u);
      filter.allow[v >> 5] |= 1u << (v & 31u);
    }
  }
  return filter;
}

// The warning for a caller's start capped to the device; a host allocation
// failure loses the warning alone.
void warn_capped(std::int64_t asked, std::int64_t ceiling) noexcept {
  try {
    log_message(core::LogLevel::Warning,
                "fusion grid: starting at " + std::to_string(ceiling) +
                    " buckets, not " + std::to_string(asked) +
                    ": the device's maxStorageBufferRange binds no more");
  } catch (...) {
  }
}

}  // namespace

std::int32_t fusion_buckets(float voxel_size,
                            VkDeviceSize max_storage_buffer_range,
                            std::optional<std::int32_t> num_buckets) {
  const double ratio = 0.01 / double(voxel_size);
  // A NaN size takes the default (fmax drops the NaN), so the conversion stays
  // defined and the grid's validation refuses the size.
  const double scale = std::fmin(std::fmax(ratio * ratio, 1.0), 4.0);
  const std::int64_t start =
      num_buckets ? *num_buckets : std::int64_t(16384.0 * scale);
  // max_num_buckets' arithmetic, on the layout of a one-bucket grid.
  const volumetric_kit::grid::volume::VoxelGridParams grid =
      volumetric_kit::grid::volume::block_grid_params(voxel_size, voxel_size,
                                                      1);
  const auto voxels = std::uint64_t(grid.voxels_per_block);
  std::uint64_t blocks =
      std::uint64_t(std::numeric_limits<std::int32_t>::max()) / voxels;
  for (const volumetric_kit::grid::volume::AttributeSpec& attr :
       kFusionAttributes) {
    blocks = std::min<std::uint64_t>(
        blocks, max_storage_buffer_range / (voxels * attr.element_size));
  }
  const auto ceiling =
      static_cast<std::int64_t>(blocks / std::uint64_t(grid.bucket_size));
  if (num_buckets && start > ceiling) warn_capped(start, ceiling);
  return static_cast<std::int32_t>(std::min(start, ceiling));
}

core::Result<volumetric_kit::grid::volume::VoxelBlockGrid> create_fusion_grid(
    core::Device& device, core::Allocator& allocator, float voxel_size,
    float trunc_dist, std::optional<std::int32_t> num_buckets) {
  return volumetric_kit::grid::volume::VoxelBlockGrid::create(
      device, allocator,
      volumetric_kit::grid::volume::block_grid_params(
          voxel_size, trunc_dist,
          fusion_buckets(voxel_size, core::max_storage_buffer_range(device),
                         num_buckets)),
      kFusionAttributes, std::size(kFusionAttributes));
}

volumetric_kit::grid::volume::CastFilter rig_cast(float min_weight) {
  return source_filter(min_weight, {0u});
}

volumetric_kit::grid::volume::CastFilter tracking_cast(float min_weight) {
  return source_filter(min_weight, {0u, kMobileBit});
}

}  // namespace volumetric_kit::recon::tsdf
