// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// A codec stream as a player sees it: encode a grid, decode the frame into a
// player grid built from the first frame's header, and keep a row per coded
// frame for the examples' report. The player-side policy lives here -- when to
// grow the grid for a frame that outgrows it, retrying lock contention --
// because it is a player's, not the decoder's (the Decoder refuses rather than
// grows); the grow itself is the volume tier's (volume::grow_grid).

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <ios>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "codec_rows.hpp"
#include "grid_layout.hpp"
#include "stage_table.hpp"
#include "volumetric_kit/core/base/file_io.hpp"
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
      VKC_TRY(vr::volume::grow_grid(grid, player_buckets(info.block_count)));
      ++*grows;
    } else if (s.domain() != vkc::Status::Code::IoError ||
               ++contended > kContendedRetries) {
      return s;
    }
  }
}

/// @return A row's host milliseconds, or 0 if @p m has no row of that name.
inline double row_ms(const vkc::StageMetrics& m, const char* name) {
  const vkc::StageRow* row = find_row(m, name);
  return row != nullptr ? row->cpu_ms : 0.0;
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

/// @brief An encoder, a decoder and the player grid it decodes into, with a
///        row for every frame coded through them.
class CodecStream {
 public:
  /// The player grid is built from the first frame's header, sized for its
  /// blocks, and grows as later frames need. The decoder runs its rANS
  /// coding where @p config says the encoder does. @p outputs are made here,
  /// after the codec objects and before any coding: the frame directory, then
  /// the CSV, its header flushed.
  /// @return The stream; `create_new_directory`'s error for the frame
  ///         directory (InvalidArgument when something is at its path,
  ///         NotFound when its parent is missing); IoError naming a CSV that
  ///         cannot be written, with the frame directory removed again; or
  ///         the `Encoder::create` / `Decoder::create` error.
  static vkc::Result<CodecStream> create(vkc::Device& device,
                                         vkc::Allocator& allocator,
                                         const vr::codec::EncoderConfig& config,
                                         CodecOutputs outputs = {}) {
    VKC_ASSIGN(vr::codec::Encoder enc,
               vr::codec::Encoder::create(device, allocator, config));
    vr::codec::DecoderConfig dc;
    dc.entropy = config.entropy;
    VKC_ASSIGN(vr::codec::Decoder dec,
               vr::codec::Decoder::create(device, allocator, dc));
    if (!outputs.frame_directory.empty()) {
      VKC_TRY(vkc::create_new_directory(outputs.frame_directory));
    }
    // The core writes whole files only, so the CSV, extended a row at a time,
    // is the stream's own.
    std::ofstream csv;
    if (!outputs.csv.empty()) {
      csv.open(outputs.csv, std::ios::binary | std::ios::trunc);
      csv << kCodecRowsCsvHeader << std::flush;
      if (!csv) {
        // The directory is this call's and still empty: take it back, so a
        // re-run is refused for the CSV again, not for the directory.
        if (!outputs.frame_directory.empty()) {
          std::error_code ignored;
          std::filesystem::remove(outputs.frame_directory, ignored);
        }
        return csv_error(outputs.csv);
      }
    }
    return CodecStream(device, allocator, std::move(enc), std::move(dec),
                       std::move(outputs), std::move(csv));
  }

  /// @brief Encode @p source as one frame, decode it into the player, and
  ///        keep its row. Files are written outside the codec timings: the
  ///        frame to the frame directory's @ref frame_file before it is
  ///        decoded, so a frame the decoder refuses is kept; once it decodes,
  ///        to @p frame_path, and its row to the CSV, flushed.
  /// @param frame       The source's index of the frame: its row's and its
  ///                    frame file's.
  /// @param frame_path  Optional further file for a frame that decodes,
  ///                    replaced if it exists.
  /// @return OK; the encode or decode error; `write_file`'s error for a file
  ///         (InvalidArgument for a frame file already there, @p frame coded
  ///         before); or IoError naming the CSV.
  vkc::Status code(vr::volume::VoxelBlockGrid& source, std::size_t frame,
                   const std::string& frame_path = {}) {
    vkc::StageMetrics encoded;
    VKC_ASSIGN(const Bytes bytes, encoder_.encode(source, &encoded));
    const std::string_view chars(reinterpret_cast<const char*>(bytes.data()),
                                 bytes.size());
    if (!outputs_.frame_directory.empty()) {
      VKC_TRY(vkc::write_file(frame_file(outputs_.frame_directory, frame),
                              chars, vkc::WriteMode::CreateNew));
    }
    VKC_ASSIGN(const vr::codec::FrameInfo info,
               vr::codec::read_frame_info(bytes.data(), bytes.size()));
    if (!player_) {
      VKC_ASSIGN(vr::volume::VoxelBlockGrid g,
                 player_grid(*device_, *allocator_, bytes));
      player_.emplace(std::move(g));
    }
    vkc::StageMetrics decoded;
    VKC_TRY(decode_growing(decoder_, bytes, *player_, &decoded, &grows_));
    if (!frame_path.empty()) {
      VKC_TRY(vkc::write_file(frame_path, chars, vkc::WriteMode::Replace));
    }
    encode_rows_.merge(encoded);
    decode_rows_.merge(decoded);
    rows_.push_back(make_codec_row(frame, bytes.size(), info.block_count,
                                   encoded, decoded));
    if (csv_.is_open()) {
      csv_ << codec_row_csv(rows_.back()) << std::flush;
      if (!csv_) return csv_error(outputs_.csv);
    }
    return {};
  }

  /// @return A row per frame coded so far, in coding order.
  const std::vector<CodecRow>& rows() const noexcept { return rows_; }

  /// @return The grid the last frame was decoded into. @pre rows() is not
  ///         empty.
  vr::volume::VoxelBlockGrid& player() { return *player_; }

  /// @brief Print what the stream cost: size, bitrate, each column's spread
  ///        over the frames, and where the encode and decode time went.
  ///        @pre rows() is not empty.
  /// @param fps    The source's frame rate.
  /// @param every  Source frames per coded frame; 0 when only one frame was
  ///               coded, which has no bitrate.
  void report(double fps, int every) const {
    const auto frames = double(rows_.size());
    double bytes = 0.0;
    double blocks = 0.0;
    for (const CodecRow& row : rows_) {
      bytes += double(row.bytes);
      blocks += double(row.blocks);
    }
    const double per_frame = bytes / frames;
    const double per_block = blocks > 0 ? bytes / blocks : 0.0;
    std::printf("stream: %zu frames coded%s\n", rows_.size(),
                grows_ > 0 ? " (the player grid grew)" : "");
    std::printf(
        "  %.0f bytes per frame, %.0f blocks; %.1f bytes per block, %.0fx "
        "under raw",
        per_frame, blocks / frames, per_block,
        per_block > 0 ? kRawBytesPerBlock / per_block : 0.0);
    if (every > 0) {
      const double coded_fps = fps / double(every);
      std::printf("; %.2f Mbit/s at %.1f coded frames/s (1 in %d at %.0f fps)",
                  per_frame * 8.0 * coded_fps / 1e6, coded_fps, every, fps);
    }
    std::printf("\n");
    // One frame is its own spread, already printed.
    if (rows_.size() > 1) {
      std::fputs(format_codec_rows(rows_, 2).c_str(), stdout);
    }
    print_stage_rows("encode, per coded frame", encode_rows_, rows_.size(), 2);
    print_stage_rows("decode, per coded frame", decode_rows_, rows_.size(), 2);
    const double enc = row_ms(encode_rows_, "  ..rans encode") / frames;
    const double dec = row_ms(decode_rows_, "  ..rans decode") / frames;
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
              vr::codec::Encoder enc, vr::codec::Decoder dec,
              CodecOutputs outputs, std::ofstream csv)
      : device_(&device),
        allocator_(&allocator),
        encoder_(std::move(enc)),
        decoder_(std::move(dec)),
        outputs_(std::move(outputs)),
        csv_(std::move(csv)) {}

  static vkc::Status csv_error(const std::string& path) {
    return vkc::Status::io_error("CodecStream: cannot write the CSV " + path);
  }

  vkc::Device* device_;
  vkc::Allocator* allocator_;
  vr::codec::Encoder encoder_;
  vr::codec::Decoder decoder_;
  CodecOutputs outputs_;
  std::ofstream csv_;  // open when outputs_.csv names a file
  std::optional<vr::volume::VoxelBlockGrid> player_;
  vkc::StageMetrics encode_rows_;  // every frame's, summed by stage
  vkc::StageMetrics decode_rows_;
  std::vector<CodecRow> rows_;
  int grows_ = 0;
};

}  // namespace vr_example
