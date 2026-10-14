// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// A codec stream as a player sees it: encode a grid, decode the frame into a
// player grid built from the first frame's header, and keep a row per coded
// frame, which the report sums and the CSV lists; the frames themselves go to
// a .vrst file. The player-side policy lives here -- when to grow the grid for
// a frame that outgrows it, retrying lock contention -- because it is a
// player's, not the decoder's (the Decoder refuses rather than grows); the
// grow itself is the volume tier's (volume::grow_grid).

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <ios>
#include <memory>
#include <optional>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "grid_layout.hpp"
#include "stage_table.hpp"
#include "volumetric_kit/core/base/check.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/recon/codec/decoder.hpp"
#include "volumetric_kit/recon/codec/encoder.hpp"
#include "volumetric_kit/recon/eval/mesh_distance.hpp"
#include "volumetric_kit/recon/stream/stream_writer.hpp"
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

/// @brief Decode, growing the grid once on OutOfMemory -- the recovery the
///        Decoder names -- to at least the frame's @ref player_buckets, and
///        decoding again, twice at most, on IoError.
///
/// The Decoder's OutOfMemory is the grid's (too few slots for the frame, or
/// a hash table that cannot place it), or the device's own, out of memory
/// for its buffers, which no grow helps. One grow leaves a player grid's heap
/// at most half full of the frame, so a second OutOfMemory is the device's,
/// and is returned as the Decoder's. IoError is how the Decoder
/// reports bucket-lock contention that outlasted its own rounds, which
/// another decode clears, and nothing else; every other error is returned as
/// itself. Only the attempt that succeeds is timed into @p rows, so a frame
/// that grew the grid on the way is reported as one decode.
inline vkc::Status decode_growing(vr::codec::Decoder& dec, const Bytes& frame,
                                  vr::volume::VoxelBlockGrid& grid,
                                  vkc::StageMetrics* rows, int* grows) {
  constexpr int kContendedRetries = 2;
  VKC_ASSIGN(const vr::codec::FrameInfo info,
             vr::codec::read_frame_info(frame.data(), frame.size()));
  int contended = 0;
  bool grown = false;
  for (;;) {
    vkc::StageMetrics attempt;
    const vkc::Status s = dec.decode(frame.data(), frame.size(), grid,
                                     rows != nullptr ? &attempt : nullptr);
    if (s.ok()) {
      if (rows != nullptr) rows->merge(attempt);
      return s;
    }
    if (s.domain() == vkc::Status::Code::OutOfMemory) {
      if (grown) return s;
      VKC_TRY(vr::volume::grow_grid(grid, player_buckets(info.block_count)));
      grown = true;
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

/// @brief One coded frame: its size, and what encoding and decoding it cost.
///        A device column is unset where its call timed no device span.
struct CodecRow {
  std::size_t frame = 0;     ///< The source's index of the frame coded.
  std::size_t bytes = 0;     ///< The compressed frame's size.
  std::uint32_t blocks = 0;  ///< The blocks it carries.
  double encode_ms = 0.0;    ///< Encode, host wall clock.
  std::optional<double> encode_device_ms;  ///< Encode, device.
  double decode_ms = 0.0;                  ///< Decode, host wall clock.
  std::optional<double> decode_device_ms;  ///< Decode, device.
};

/// @return A codec call's device time: every span @p m timed summed
///         (`StageMetrics::total_gpu_ms`, breakdowns included, as each is a
///         dispatch of its own), or unset when it timed none.
inline std::optional<double> device_ms(const vkc::StageMetrics& m) {
  for (const vkc::StageRow& row : m.rows()) {
    if (row.has_gpu) return m.total_gpu_ms();
  }
  return std::nullopt;
}

/// @brief A frame's row from the stage rows its encode and decode filled:
///        each call's host span (`StageMetrics::total_cpu_ms`, its stage
///        without the breakdowns inside it) and its @ref device_ms.
inline CodecRow make_codec_row(std::size_t frame, std::size_t bytes,
                               std::uint32_t blocks,
                               const vkc::StageMetrics& encode,
                               const vkc::StageMetrics& decode) {
  CodecRow row;
  row.frame = frame;
  row.bytes = bytes;
  row.blocks = blocks;
  row.encode_ms = encode.total_cpu_ms();
  row.encode_device_ms = device_ms(encode);
  row.decode_ms = decode.total_cpu_ms();
  row.decode_device_ms = device_ms(decode);
  return row;
}

/// The first line of a stream's CSV, naming @ref codec_row_csv's columns.
inline constexpr const char kCodecRowsCsvHeader[] =
    "frame,bytes,blocks,encode_host_ms,encode_device_ms,decode_host_ms,"
    "decode_device_ms\n";

/// @return @p row as a line under @ref kCodecRowsCsvHeader: milliseconds to
///         the microsecond, and an unset device column empty.
inline std::string codec_row_csv(const CodecRow& row) {
  const auto device = [](const std::optional<double>& ms) {
    char text[32] = "";
    if (ms) std::snprintf(text, sizeof text, "%.3f", *ms);
    return std::string(text);
  };
  char text[96];
  std::snprintf(text, sizeof text, "%zu,%zu,%u,%.3f,", row.frame, row.bytes,
                row.blocks, row.encode_ms);
  std::string out = text + device(row.encode_device_ms);
  std::snprintf(text, sizeof text, ",%.3f,", row.decode_ms);
  return out + text + device(row.decode_device_ms) + "\n";
}

/// @return Source frame @p frame's time at @p fps, frame 0 at 0, in ns
///         rounded to the nearest. @pre @p fps is positive.
inline std::uint64_t frame_pts_ns(std::size_t frame, double fps) {
  VKC_CHECK(fps > 0.0, "frame_pts_ns needs a positive frame rate");
  return std::uint64_t(std::llround(double(frame) * 1e9 / fps));
}

/// @brief Where a codec stream writes besides its report, each frame as it is
///        coded; an empty path writes nothing.
struct CodecOutputs {
  /// A file of @ref kCodecRowsCsvHeader, then the @ref codec_row_csv of each
  /// frame that decodes; replaced if it exists. Must not alias the stream.
  std::string csv;
  /// A new `.vrst` file (stream/session.hpp) of every frame the encoder
  /// returns, a record each on its one Geometry track. Nothing may be at the
  /// path, and its directory must exist.
  std::string stream;
};

/// @brief An encoder, a decoder and the player grid it decodes into, with a
///        row for every frame decoded through them.
class CodecStream {
 public:
  /// The player grid is built from the first frame's header, sized for its
  /// blocks, and grows as later frames need. @p outputs are made here,
  /// after the codec objects and before any coding: the stream first, as a
  /// rerun onto its file is the common refusal, then the CSV, its header
  /// flushed.
  /// @param program  The program writing the stream, recorded in its session
  ///                 header.
  /// @return The stream; `StreamWriter::create`'s error for the stream
  ///         (InvalidArgument when something is at its path, NotFound when
  ///         its directory is missing); InvalidArgument if the CSV aliases
  ///         the stream; IoError naming a CSV that cannot be written; or the
  ///         `Encoder::create` / `Decoder::create` error.
  ///         Either CSV error removes the new stream's file again.
  static vkc::Result<CodecStream> create(vkc::Device& device,
                                         vkc::Allocator& allocator,
                                         const vr::codec::EncoderConfig& config,
                                         CodecOutputs outputs = {},
                                         const std::string& program = {}) {
    VKC_ASSIGN(vr::codec::Encoder enc,
               vr::codec::Encoder::create(device, allocator, config));
    VKC_ASSIGN(vr::codec::Decoder dec,
               vr::codec::Decoder::create(device, allocator));
    // No session id or start time, so a rerun writes the same bytes.
    std::unique_ptr<vr::stream::StreamWriter> writer;
    if (!outputs.stream.empty()) {
      vr::stream::Track geometry;
      geometry.kind = vr::stream::TrackKind::Geometry;
      vr::stream::Session session;
      session.writer = program;
      session.tracks.push_back(geometry);
      VKC_ASSIGN(writer,
                 vr::stream::StreamWriter::create(outputs.stream, session));
    }
    const auto remove_stream = [&] {
      // The stream's file is this call's and holds no record: take it
      // back, so a rerun is refused for the CSV again, not for the stream.
      if (writer) {
        writer.reset();
        std::error_code ignored;
        std::filesystem::remove(outputs.stream, ignored);
      }
    };
    // The core writes whole files only, so the CSV, extended a row at a time,
    // is the stream's own.
    std::ofstream csv;
    if (!outputs.csv.empty()) {
      if (writer) {
        // Creating the stream first resolves dangling symlinks and case
        // aliases. Check identity before opening the CSV can truncate it.
        std::error_code error;
        if (std::filesystem::exists(outputs.csv, error) &&
            std::filesystem::equivalent(outputs.csv, outputs.stream, error)) {
          remove_stream();
          return vkc::Status::invalid_argument(
              "CodecStream: CSV " + outputs.csv + " aliases stream " +
              outputs.stream);
        }
        if (error) {
          remove_stream();
          return csv_error(outputs.csv);
        }
      }
      csv.open(outputs.csv, std::ios::binary | std::ios::trunc);
      csv << kCodecRowsCsvHeader << std::flush;
      if (!csv) {
        remove_stream();
        return csv_error(outputs.csv);
      }
    }
    return CodecStream(device, allocator, std::move(enc), std::move(dec),
                       std::move(outputs), std::move(writer), std::move(csv));
  }

  /// @brief Encode @p source as one frame, decode it into the player, and
  ///        keep its row. Files are written outside the codec timings: the
  ///        frame to the stream as soon as it is encoded, so a frame the
  ///        decoder refuses is kept, and once it decodes, its row to the CSV,
  ///        flushed.
  /// @param frame   The source's index of the frame: its row's, and its
  ///                record's set. Above every frame coded before.
  /// @param pts_ns  Its time, its record's pts: no earlier than the frames'
  ///                coded before (@ref frame_pts_ns).
  /// @return OK; InvalidArgument, before anything is encoded, for a
  ///         @p frame or @p pts_ns out of order or a finished stream; the
  ///         encode or decode error; `StreamWriter::append`'s error; or
  ///         IoError naming the CSV.
  vkc::Status code(vr::volume::VoxelBlockGrid& source, std::size_t frame,
                   std::uint64_t pts_ns) {
    // Refused here, not by the writer once the encoder holds the frame.
    if (finished_) {
      return vkc::Status::invalid_argument("CodecStream: finished");
    }
    if (coded_ > 0 && (frame <= last_frame_ || pts_ns < last_pts_ns_)) {
      return vkc::Status::invalid_argument(
          "CodecStream: frame " + std::to_string(frame) + " at " +
          std::to_string(pts_ns) + " ns does not follow frame " +
          std::to_string(last_frame_) + " at " + std::to_string(last_pts_ns_) +
          " ns");
    }
    vkc::StageMetrics encoded;
    VKC_ASSIGN(const Bytes bytes, encoder_.encode(source, &encoded));
    if (writer_) {
      // TODO(examples): key from FrameInfo::type once P-frames are coded
      // (C5b); every frame is intra until then.
      vr::stream::RecordInfo record;
      record.key = true;
      record.pts_ns = pts_ns;
      record.seq = coded_;
      record.set = frame;
      VKC_TRY(writer_->append(record, bytes.data(), bytes.size()));
    }
    ++coded_;
    last_frame_ = frame;
    last_pts_ns_ = pts_ns;
    VKC_ASSIGN(const vr::codec::FrameInfo info,
               vr::codec::read_frame_info(bytes.data(), bytes.size()));
    if (!player_) {
      VKC_ASSIGN(vr::volume::VoxelBlockGrid g,
                 player_grid(*device_, *allocator_, bytes));
      player_.emplace(std::move(g));
    }
    vkc::StageMetrics decoded;
    VKC_TRY(decode_growing(decoder_, bytes, *player_, &decoded, &grows_));
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

  /// @brief Finish the stream's file with its footer index
  ///        (`StreamWriter::finish`); without a stream, only mark it
  ///        finished. One attempt: however it ends, every later call, this
  ///        one included, is refused, so a file whose footer failed stays
  ///        unfinished, read as a killed writer's.
  /// @return OK; InvalidArgument once finished; or the writer's error.
  vkc::Status finish() {
    if (finished_) {
      return vkc::Status::invalid_argument("CodecStream: finished");
    }
    finished_ = true;
    return writer_ ? writer_->finish() : vkc::Status{};
  }

  /// @return A row per frame decoded so far, in coding order.
  const std::vector<CodecRow>& rows() const noexcept { return rows_; }

  /// @return The grid the last frame was decoded into. @pre rows() is not
  ///         empty.
  vr::volume::VoxelBlockGrid& player() { return *player_; }

  /// @brief Print what the stream cost: size, bitrate, and where the encode
  ///        and decode time went. @pre rows() is not empty.
  /// @param fps    The source's frame rate.
  /// @param every  Source frames per coded frame; 0 when only one frame was
  ///               coded, which has no bitrate.
  void report(double fps, int every) const {
    const auto frames = double(rows_.size());
    double bytes = 0.0;
    double blocks = 0.0;
    std::size_t max_bytes = 0;
    for (const CodecRow& row : rows_) {
      bytes += double(row.bytes);
      blocks += double(row.blocks);
      max_bytes = std::max(max_bytes, row.bytes);
    }
    const double per_frame = bytes / frames;
    const double per_block = blocks > 0 ? bytes / blocks : 0.0;
    std::printf("stream: %zu frames coded%s\n", rows_.size(),
                grows_ > 0 ? " (the player grid grew)" : "");
    std::printf(
        "  %.0f bytes per frame (max %zu), %.0f blocks; %.1f bytes "
        "per block, %.0fx under raw",
        per_frame, max_bytes, blocks / frames, per_block,
        per_block > 0 ? kRawBytesPerBlock / per_block : 0.0);
    if (every > 0) {
      const double coded_fps = fps / double(every);
      std::printf("; %.2f Mbit/s at %.1f coded frames/s (1 in %d at %.0f fps)",
                  per_frame * 8.0 * coded_fps / 1e6, coded_fps, every, fps);
    }
    std::printf("\n");
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
              CodecOutputs outputs,
              std::unique_ptr<vr::stream::StreamWriter> writer,
              std::ofstream csv)
      : device_(&device),
        allocator_(&allocator),
        encoder_(std::move(enc)),
        decoder_(std::move(dec)),
        outputs_(std::move(outputs)),
        writer_(std::move(writer)),
        csv_(std::move(csv)) {}

  static vkc::Status csv_error(const std::string& path) {
    return vkc::Status::io_error("CodecStream: cannot write the CSV " + path);
  }

  vkc::Device* device_;
  vkc::Allocator* allocator_;
  vr::codec::Encoder encoder_;
  vr::codec::Decoder decoder_;
  CodecOutputs outputs_;
  std::unique_ptr<vr::stream::StreamWriter> writer_;  // null without a stream
  std::ofstream csv_;           // open when outputs_.csv names a file
  std::uint64_t coded_ = 0;     // frames encoded and recorded: the next seq
  std::size_t last_frame_ = 0;  // the last of them, once coded_ > 0
  std::uint64_t last_pts_ns_ = 0;
  bool finished_ = false;
  std::optional<vr::volume::VoxelBlockGrid> player_;
  vkc::StageMetrics encode_rows_;  // every frame's, summed by stage
  vkc::StageMetrics decode_rows_;
  std::vector<CodecRow> rows_;
  int grows_ = 0;
};

}  // namespace vr_example
