// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// A codec stream as a player sees it: encode a grid, decode the frame into a
// player grid built from the first frame's header, and keep the totals the
// examples report. The player-side policy lives here -- growing the grid when
// a frame outgrows it, retrying lock contention -- because it is a player's,
// not the library's (the Decoder refuses rather than grows).

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>
#include <utility>
#include <vector>

#include "mesh_distance.hpp"
#include "volumetric_kit/recon/codec/decoder.hpp"
#include "volumetric_kit/recon/codec/encoder.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/core/stage_metrics.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"

namespace vr_example {

namespace vr = volumetric_kit::recon;

using Bytes = std::vector<std::uint8_t>;

/// Raw tsdf + weight per block: what a frame compresses.
inline constexpr double kRawBytesPerBlock = 512.0 * 2.0 * sizeof(float);

/// @brief A grid for a stream, from a frame's header: its geometry, `tsdf` +
///        `weight` only (all a frame carries, and all the Decoder accepts).
inline vr::Result<vr::volume::VoxelBlockGrid> player_grid(
    vr::Device& device, vr::Allocator& allocator, const Bytes& frame,
    std::int32_t num_buckets) {
  VR_ASSIGN(const vr::codec::FrameInfo info,
            vr::codec::read_frame_info(frame.data(), frame.size()));
  vr::volume::VoxelGridParams gp{};
  gp.voxel_size = info.voxel_size;
  gp.block_size = vr::codec::kBlockSize;
  gp.voxels_per_block = std::int32_t(vr::codec::kVoxelsPerBlock);
  gp.trunc_dist = info.trunc_dist;
  gp.bucket_size = 8;
  gp.num_buckets = num_buckets;
  gp.num_blocks = gp.bucket_size * gp.num_buckets;
  gp.max_chain = 128;
  const vr::volume::AttributeSpec attrs[] = {{"tsdf", sizeof(float)},
                                             {"weight", sizeof(float)}};
  return vr::volume::VoxelBlockGrid::create(device, allocator, gp, attrs, 2);
}

/// @brief Decode, doubling the grid on OutOfMemory -- the recovery the
///        Decoder names -- and retrying IoError (lock contention).
inline vr::Status decode_growing(vr::codec::Decoder& dec, const Bytes& frame,
                                 vr::volume::VoxelBlockGrid& grid,
                                 vr::StageMetrics* rows, int* grows) {
  for (int attempt = 0; attempt < 16; ++attempt) {
    const vr::Status s = dec.decode(frame.data(), frame.size(), grid, rows);
    if (s.ok()) {
      return s;
    }
    if (s.domain() == vr::Status::Code::OutOfMemory) {
      VR_TRY(grid.resize(grid.grid().num_buckets * 2));
      ++*grows;
    } else if (s.domain() != vr::Status::Code::IoError) {
      return s;
    }
  }
  return vr::Status::out_of_memory(
      "the player grid kept failing to hold "
      "the frame");
}

/// @return A row's host milliseconds, or 0 if @p m has no row of that name.
inline double row_ms(const vr::StageMetrics& m, const char* name) {
  for (const vr::StageRow& r : m.rows()) {
    if (std::strcmp(r.name, name) == 0) return r.cpu_ms;
  }
  return 0.0;
}

/// @brief Print @p rows, each divided by @p per (host ms / device ms).
inline void print_stage_rows(const char* title, const vr::StageMetrics& rows,
                             std::size_t per) {
  std::printf("  %s, per coded frame (host ms / device ms):\n", title);
  for (const vr::StageRow& r : rows.rows()) {
    std::printf("    %-18s %8.3f", r.name, r.cpu_ms / double(per));
    if (r.has_gpu) {
      std::printf("  %8.3f\n", r.gpu_ms / double(per));
    } else {
      std::printf("         -\n");
    }
  }
}

/// @brief Print an accuracy / coverage comparison, distances in mm.
inline void print_comparison(const MeshComparison& c, float voxel) {
  auto line = [voxel](const char* name, const DistanceStats& s) {
    std::printf(
        "    %-9s mean %6.3f  rms %6.3f  p95 %6.3f  max %6.3f mm "
        "(%.3f voxels rms), %zu of %zu beyond reach\n",
        name, s.mean * 1e3, s.rms * 1e3, s.p95 * 1e3, s.max * 1e3,
        s.rms / double(voxel), s.beyond_reach, s.count);
  };
  line("accuracy", c.accuracy);
  line("coverage", c.coverage);
}

/// @brief An encoder, a decoder and the player grid it decodes into, with the
///        totals of every frame coded through them.
class CodecStream {
 public:
  /// @param num_buckets  The player grid's starting size; it grows as needed.
  static vr::Result<CodecStream> create(vr::Device& device,
                                        vr::Allocator& allocator,
                                        const vr::codec::EncoderConfig& config,
                                        std::int32_t num_buckets) {
    VR_ASSIGN(vr::codec::Encoder enc,
              vr::codec::Encoder::create(device, allocator, config));
    VR_ASSIGN(vr::codec::Decoder dec,
              vr::codec::Decoder::create(device, allocator));
    return CodecStream(device, allocator, std::move(enc), std::move(dec),
                       num_buckets);
  }

  /// @brief Encode @p source as one frame and decode it into the player.
  vr::Status code(vr::volume::VoxelBlockGrid& source) {
    VR_ASSIGN(const Bytes frame, encoder_.encode(source, &encode_rows_));
    if (!player_) {
      VR_ASSIGN(vr::volume::VoxelBlockGrid g,
                player_grid(*device_, *allocator_, frame, num_buckets_));
      player_.emplace(std::move(g));
    }
    VR_TRY(decode_growing(decoder_, frame, *player_, &decode_rows_, &grows_));
    VR_ASSIGN(const vr::codec::FrameInfo info,
              vr::codec::read_frame_info(frame.data(), frame.size()));
    ++frames_;
    bytes_ += double(frame.size());
    blocks_ += double(info.block_count);
    max_bytes_ = std::max(max_bytes_, frame.size());
    return {};
  }

  /// @return The frames coded so far.
  std::size_t frames() const noexcept { return frames_; }

  /// @return The grid the last frame was decoded into. @pre frames() > 0.
  vr::volume::VoxelBlockGrid& player() { return *player_; }

  /// @brief Print what the stream cost: size, bitrate at @p fps, and where
  ///        the encode and decode time went. @pre frames() > 0.
  void report(double fps) const {
    const double per_frame = bytes_ / double(frames_);
    const double per_block = blocks_ > 0 ? bytes_ / blocks_ : 0.0;
    std::printf("stream: %zu frames coded%s\n", frames_,
                grows_ > 0 ? " (the player grid grew)" : "");
    std::printf(
        "  %.0f bytes per frame (max %zu), %.0f blocks; %.1f bytes "
        "per block, %.0fx under raw; %.2f Mbit/s at %.0f fps\n",
        per_frame, max_bytes_, blocks_ / double(frames_), per_block,
        per_block > 0 ? kRawBytesPerBlock / per_block : 0.0,
        per_frame * 8.0 * fps / 1e6, fps);
    print_stage_rows("encode", encode_rows_, frames_);
    print_stage_rows("decode", decode_rows_, frames_);
    const double budget = 1e3 / fps;
    const double enc = row_ms(encode_rows_, "  ..rans encode") / frames_;
    const double dec = row_ms(decode_rows_, "  ..rans decode") / frames_;
    std::printf(
        "  host rANS: %.2f ms encode, %.2f ms decode per frame "
        "(%.0f%% / %.0f%% of a %.1f ms frame)\n",
        enc, dec, 100.0 * enc / budget, 100.0 * dec / budget, budget);
  }

 private:
  CodecStream(vr::Device& device, vr::Allocator& allocator,
              vr::codec::Encoder enc, vr::codec::Decoder dec,
              std::int32_t num_buckets)
      : device_(&device),
        allocator_(&allocator),
        encoder_(std::move(enc)),
        decoder_(std::move(dec)),
        num_buckets_(num_buckets) {}

  vr::Device* device_;
  vr::Allocator* allocator_;
  vr::codec::Encoder encoder_;
  vr::codec::Decoder decoder_;
  std::int32_t num_buckets_;
  std::optional<vr::volume::VoxelBlockGrid> player_;
  vr::StageMetrics encode_rows_;
  vr::StageMetrics decode_rows_;
  std::size_t frames_ = 0;
  double bytes_ = 0.0;
  double blocks_ = 0.0;
  std::size_t max_bytes_ = 0;
  int grows_ = 0;
};

}  // namespace vr_example
