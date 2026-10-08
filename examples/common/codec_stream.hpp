// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// A codec stream as a player sees it: encode a grid, decode the frame into a
// player grid built from the first frame's header, and keep the totals the
// examples report. The player-side policy lives here -- when to grow the grid
// for a frame that outgrows it, retrying lock contention -- because it is a
// player's, not the decoder's (the Decoder refuses rather than grows); the
// grow itself is the volume tier's (volume::GridGrowth).

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "grid_layout.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/recon/codec/decoder.hpp"
#include "volumetric_kit/recon/codec/encoder.hpp"
#include "volumetric_kit/recon/eval/mesh_distance.hpp"
#include "volumetric_kit/recon/volume/grid_growth.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"

namespace vr_example {

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;

using Bytes = std::vector<std::uint8_t>;

/// Raw tsdf + weight per block: what a frame compresses.
inline constexpr double kRawBytesPerBlock = 512.0 * 2.0 * sizeof(float);

/// @return Buckets enough for @p blocks at half the heap, and at least 1024.
inline std::int32_t player_buckets(std::uint32_t blocks) {
  const std::int64_t need =
      (2 * std::int64_t(blocks) + kExampleBucketSize - 1) / kExampleBucketSize;
  return std::int32_t(std::max<std::int64_t>(need, 1024));
}

/// @brief A grid for a stream, from a frame's header: its geometry, `tsdf` +
///        `weight` only (all a frame carries, and all the Decoder accepts),
///        in the examples' layout, sized for the frame's own blocks.
inline vkc::Result<vr::volume::VoxelBlockGrid> player_grid(
    vkc::Device& device, vkc::Allocator& allocator, const Bytes& frame) {
  VKC_ASSIGN(const vr::codec::FrameInfo info,
             vr::codec::read_frame_info(frame.data(), frame.size()));
  const vr::volume::AttributeSpec attrs[] = {{"tsdf", sizeof(float)},
                                             {"weight", sizeof(float)}};
  return vr::volume::VoxelBlockGrid::create(
      device, allocator,
      example_grid_params(info.voxel_size, info.trunc_dist,
                          player_buckets(info.block_count)),
      attrs, 2);
}

/// @brief Decode, growing the grid on OutOfMemory -- the recovery the
///        Decoder names -- to at least the frame's @ref player_buckets, and
///        decoding again, twice at most, on IoError.
///
/// IoError is how the Decoder reports bucket-lock contention that outlasted
/// its own rounds, which another decode clears, and nothing else; every
/// other error is returned as itself. Only the attempt that succeeds is timed
/// into @p rows, so a frame that grew the grid on the way is reported as one
/// decode.
inline vkc::Status decode_growing(vr::codec::Decoder& dec, const Bytes& frame,
                                  vr::volume::VoxelBlockGrid& grid,
                                  vkc::StageMetrics* rows, int* grows) {
  constexpr int kContendedRetries = 2;
  VKC_ASSIGN(const vr::codec::FrameInfo info,
             vr::codec::read_frame_info(frame.data(), frame.size()));
  vr::volume::GridGrowth growth;
  int contended = 0;
  for (;;) {
    vkc::StageMetrics attempt;
    const vkc::Status s = dec.decode(frame.data(), frame.size(), grid,
                                     rows != nullptr ? &attempt : nullptr);
    if (s.ok()) {
      if (rows != nullptr) rows->merge(attempt);
      return s;
    }
    if (s.domain() == vkc::Status::Code::OutOfMemory) {
      VKC_ASSIGN(const vr::volume::GrowthEvent grown,
                 growth.grow(grid, player_buckets(info.block_count)));
      if (!grown.grew()) {
        return grown.outcome == vr::volume::GrowthOutcome::ResizeFailed
                   ? grown.error
                   : s;
      }
      ++*grows;
    } else if (s.domain() != vkc::Status::Code::IoError ||
               ++contended > kContendedRetries) {
      return s;
    }
  }
}

/// @return A row's host milliseconds, or 0 if @p m has no row of that name.
inline double row_ms(const vkc::StageMetrics& m, const char* name) {
  for (const vkc::StageRow& r : m.rows()) {
    if (std::strcmp(r.name, name) == 0) return r.cpu_ms;
  }
  return 0.0;
}

/// @brief Print @p rows, each divided by @p per (host ms / device ms).
inline void print_stage_rows(const char* title, const vkc::StageMetrics& rows,
                             std::size_t per) {
  std::printf("  %s, per coded frame (host ms / device ms):\n", title);
  for (const vkc::StageRow& r : rows.rows()) {
    std::printf("    %-18s %8.3f", r.name, r.cpu_ms / double(per));
    if (r.has_gpu) {
      std::printf("  %8.3f\n", r.gpu_ms / double(per));
    } else {
      std::printf("         -\n");
    }
  }
}

/// @brief Print an accuracy / coverage / F-score comparison, in mm.
inline void print_comparison(const vr::eval::MeshComparison& c, float voxel) {
  auto line = [voxel](const char* name, const vr::eval::DistanceStats& s) {
    std::printf(
        "    %-9s mean %6.3f  rms %6.3f  p95 %6.3f  max %6.3f mm "
        "(%.3f voxels rms), %zu of %zu beyond reach\n",
        name, s.mean * 1e3, s.rms * 1e3, s.p95 * 1e3, s.max * 1e3,
        s.rms / double(voxel), s.beyond_reach, s.count);
  };
  line("accuracy", c.accuracy);
  line("coverage", c.coverage);
  if (c.fscore.threshold > 0.0f) {
    std::printf("    F-score   %.4f at %.1f mm (precision %.4f, recall %.4f)\n",
                c.fscore.f, double(c.fscore.threshold) * 1e3,
                c.fscore.precision, c.fscore.recall);
  }
}

/// @brief An encoder, a decoder and the player grid it decodes into, with the
///        totals of every frame coded through them.
class CodecStream {
 public:
  /// The player grid is built from the first frame's header, sized for its
  /// blocks, and grows as later frames need. The decoder runs its rANS
  /// coding where @p config says the encoder does.
  static vkc::Result<CodecStream> create(
      vkc::Device& device, vkc::Allocator& allocator,
      const vr::codec::EncoderConfig& config) {
    VKC_ASSIGN(vr::codec::Encoder enc,
               vr::codec::Encoder::create(device, allocator, config));
    vr::codec::DecoderConfig dc;
    dc.entropy = config.entropy;
    VKC_ASSIGN(vr::codec::Decoder dec,
               vr::codec::Decoder::create(device, allocator, dc));
    return CodecStream(device, allocator, std::move(enc), std::move(dec));
  }

  /// @brief Encode @p source as one frame and decode it into the player.
  /// @param frame_path Optional output file for the successfully decoded
  ///                   compressed frame; file I/O is outside codec timings.
  vkc::Status code(vr::volume::VoxelBlockGrid& source,
                   const std::string& frame_path = {}) {
    VKC_ASSIGN(const Bytes frame, encoder_.encode(source, &encode_rows_));
    if (!player_) {
      VKC_ASSIGN(vr::volume::VoxelBlockGrid g,
                 player_grid(*device_, *allocator_, frame));
      player_.emplace(std::move(g));
    }
    VKC_TRY(decode_growing(decoder_, frame, *player_, &decode_rows_, &grows_));
    if (!frame_path.empty()) {
      if (frame.size() >
          std::size_t(std::numeric_limits<std::streamsize>::max())) {
        return vkc::Status::invalid_argument("frame is too large to write");
      }
      std::ofstream output(frame_path, std::ios::binary);
      output.write(reinterpret_cast<const char*>(frame.data()),
                   static_cast<std::streamsize>(frame.size()));
      output.close();
      if (!output) {
        return vkc::Status::io_error("cannot write codec frame: " + frame_path);
      }
    }
    VKC_ASSIGN(const vr::codec::FrameInfo info,
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

  /// @brief Print what the stream cost: size, bitrate, and where the encode
  ///        and decode time went. @pre frames() > 0.
  /// @param fps    The source's frame rate.
  /// @param every  Source frames per coded frame; 0 when only one frame was
  ///               coded, which has no bitrate.
  void report(double fps, int every) const {
    const double per_frame = bytes_ / double(frames_);
    const double per_block = blocks_ > 0 ? bytes_ / blocks_ : 0.0;
    std::printf("stream: %zu frames coded%s\n", frames_,
                grows_ > 0 ? " (the player grid grew)" : "");
    std::printf(
        "  %.0f bytes per frame (max %zu), %.0f blocks; %.1f bytes "
        "per block, %.0fx under raw",
        per_frame, max_bytes_, blocks_ / double(frames_), per_block,
        per_block > 0 ? kRawBytesPerBlock / per_block : 0.0);
    if (every > 0) {
      const double coded_fps = fps / double(every);
      std::printf("; %.2f Mbit/s at %.1f coded frames/s (1 in %d at %.0f fps)",
                  per_frame * 8.0 * coded_fps / 1e6, coded_fps, every, fps);
    }
    std::printf("\n");
    print_stage_rows("encode", encode_rows_, frames_);
    print_stage_rows("decode", decode_rows_, frames_);
    const double enc = row_ms(encode_rows_, "  ..rans encode") / frames_;
    const double dec = row_ms(decode_rows_, "  ..rans decode") / frames_;
    std::printf(
        "  rANS: %.2f ms encode, %.2f ms decode wall clock per coded frame",
        enc, dec);
    if (every > 0) {
      // Against the source's interval: what coding every frame live must fit.
      // A standalone mesh or final-grid snapshot has no measured frame rate.
      const double budget = 1e3 / fps;
      std::printf(" (%.0f%% / %.0f%% of a %.1f ms source frame interval)",
                  100.0 * enc / budget, 100.0 * dec / budget, budget);
    }
    std::printf("\n");
  }

 private:
  CodecStream(vkc::Device& device, vkc::Allocator& allocator,
              vr::codec::Encoder enc, vr::codec::Decoder dec)
      : device_(&device),
        allocator_(&allocator),
        encoder_(std::move(enc)),
        decoder_(std::move(dec)) {}

  vkc::Device* device_;
  vkc::Allocator* allocator_;
  vr::codec::Encoder encoder_;
  vr::codec::Decoder decoder_;
  std::optional<vr::volume::VoxelBlockGrid> player_;
  vkc::StageMetrics encode_rows_;
  vkc::StageMetrics decode_rows_;
  std::size_t frames_ = 0;
  double bytes_ = 0.0;
  double blocks_ = 0.0;
  std::size_t max_bytes_ = 0;
  int grows_ = 0;
};

}  // namespace vr_example
