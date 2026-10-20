// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/set_codec.hpp
/// @brief fuse_orbbec's coded sets, apart from its camera driver, so their
///        test runs on every build.

#include <algorithm>
#include <cstddef>
#include <string>

#include "codec_stream.hpp"
#include "config.hpp"
#include "fuse_frame.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/recon/sensor/array/sensor_array.hpp"
#include "volumetric_kit/recon/tsdf/fuser.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"

namespace vr_example {

/// @brief Fuse @p set, as the array prepared it on the device, into @p grid
///        as @p fusion says (@ref fuse_set, `Dynamic` with `fusion.dynamic`),
///        free its stale blocks every `fusion.free_after` sets
///        (@ref free_stale_blocks_every), and code the grid as one frame of
///        @p stream at the set's time.
///
/// The free pass runs before the coding, so the coder scans the smaller
/// set; the frame is the same either way, since the pass frees only blocks
/// without an observed voxel, which the coder leaves out. A set stamped
/// before the last frame recorded, as a clock stepping back stamps it, is
/// coded at that frame's time rather than refused.
/// @param index   The set's index in the run: the frame's number, above every
///                frame coded before, and the sets fused before it.
/// @param stream  The stream, or null to fuse alone.
/// @param stages  Optional stage rows for the fusion and the free pass; the
///                stream keeps the codec's own.
/// @return OK; `Fuser::fuse`'s or `CodecStream::code`'s error.
inline vkc::Status fuse_and_code(vr::tsdf::Fuser& fuser,
                                 vr::volume::VoxelBlockGrid& grid,
                                 const vr::sensor::DeviceFrameSet& set,
                                 std::size_t index,
                                 const config::Fusion& fusion,
                                 CodecStream* stream,
                                 vkc::StageMetrics* stages) {
  VKC_TRY(fuse_set(fuser, grid, set.frames, fusion.max_weight,
                   integration_mode(fusion.dynamic), stages));
  free_stale_blocks_every(grid, index + 1, fusion.free_after, "fuse_orbbec",
                          stages);
  if (stream == nullptr) return {};
  return stream->code(grid, index,
                      std::max(set.timestamp_ns, stream->last_pts_ns()));
}

/// @brief Refuse to write @p path over a file @p outputs name
///        (@ref check_not_same_file), once they exist.
/// @return OK, or the first refusal.
inline vkc::Status check_apart(const std::string& path,
                               const CodecOutputs& outputs) {
  for (const std::string* output : {&outputs.stream, &outputs.csv}) {
    if (!output->empty()) VKC_TRY(check_not_same_file(path, *output));
  }
  return {};
}

}  // namespace vr_example
