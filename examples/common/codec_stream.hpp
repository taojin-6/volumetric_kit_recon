// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// A codec stream as a player sees it: encode a grid, decode the frame into a
// player grid built from the first frame's header and grown as frames need
// (codec/decode_grid.hpp), and keep a row per coded frame, which the report
// sums by frame type and the CSV lists; the frames themselves go to a .vrst
// file, each I-frame a key, and to the viewers of a live server. An I-frame
// after a frame the player missed is the stream's policy, kept here.

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

#include "stage_table.hpp"
#include "volumetric_kit/core/base/check.hpp"
#include "volumetric_kit/core/base/log.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/recon/codec/decode_grid.hpp"
#include "volumetric_kit/recon/codec/decoder.hpp"
#include "volumetric_kit/recon/codec/encoder.hpp"
#include "volumetric_kit/recon/core/log.hpp"
#include "volumetric_kit/recon/eval/mesh_distance.hpp"
#include "volumetric_kit/recon/networking/stream_server.hpp"
#include "volumetric_kit/recon/stream/stream_writer.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"

namespace vr_example {

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;

using Bytes = std::vector<std::uint8_t>;

/// Raw tsdf + weight per block: what a frame compresses.
inline constexpr double kRawBytesPerBlock = 512.0 * 2.0 * sizeof(float);

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

/// @return How the codec examples compare a decoded surface with its source,
///         inside the reconstruction's own resolution so the figures move
///         with the codec: a band's width of reach (@p trunc, at least 2 cm),
///         about 1 vertex in 4 both ways, and an F-score at half a voxel
///         (@p voxel).
inline vr::eval::CompareOptions codec_compare_options(float voxel,
                                                      float trunc) {
  vr::eval::CompareOptions compare;
  compare.reach = std::max(trunc, 0.02f);
  compare.stride = 4;
  compare.fscore_threshold = 0.5f * voxel;
  return compare;
}

/// @return @p type's name in a stream's rows: "I" or "P".
inline const char* frame_type_name(vr::codec::FrameType type) {
  return type == vr::codec::FrameType::kIntra ? "I" : "P";
}

/// @brief One coded frame: its type and size, and what encoding and decoding
///        it cost. A device column is unset where its call timed no device
///        span.
struct CodecRow {
  std::size_t frame = 0;  ///< The source's index of the frame coded.
  /// An I-frame, or a P-frame on the frame coded before.
  vr::codec::FrameType type = vr::codec::FrameType::kIntra;
  std::size_t bytes = 0;                   ///< The compressed frame's size.
  std::uint32_t blocks = 0;                ///< The blocks it carries.
  double encode_ms = 0.0;                  ///< Encode, host wall clock.
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
inline CodecRow make_codec_row(std::size_t frame, vr::codec::FrameType type,
                               std::size_t bytes, std::uint32_t blocks,
                               const vkc::StageMetrics& encode,
                               const vkc::StageMetrics& decode) {
  CodecRow row;
  row.frame = frame;
  row.type = type;
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
    "frame,type,bytes,blocks,encode_host_ms,encode_device_ms,decode_host_ms,"
    "decode_device_ms\n";

/// @return @p row as a line under @ref kCodecRowsCsvHeader: its type as
///         @ref frame_type_name names it, milliseconds to the microsecond,
///         and an unset device column empty.
inline std::string codec_row_csv(const CodecRow& row) {
  const auto device = [](const std::optional<double>& ms) {
    char text[32] = "";
    if (ms) std::snprintf(text, sizeof text, "%.3f", *ms);
    return std::string(text);
  };
  char text[96];
  std::snprintf(text, sizeof text, "%zu,%s,%zu,%u,%.3f,", row.frame,
                frame_type_name(row.type), row.bytes, row.blocks,
                row.encode_ms);
  std::string out = text + device(row.encode_device_ms);
  std::snprintf(text, sizeof text, ",%.3f,", row.decode_ms);
  return out + text + device(row.decode_device_ms) + "\n";
}

/// @brief The sizes of a stream's frames of one type, in bytes.
struct FrameSizes {
  std::size_t count = 0;  ///< Frames of the type.
  double mean = 0.0;
  std::size_t p50 = 0;  ///< The nearest-rank median.
  std::size_t p95 = 0;  ///< The nearest-rank 95th percentile.
  std::size_t max = 0;
};

/// @return The sizes of @p rows' frames of @p type; every field 0 when there
///         is none.
inline FrameSizes frame_sizes(const std::vector<CodecRow>& rows,
                              vr::codec::FrameType type) {
  std::vector<std::size_t> sizes;
  double total = 0.0;
  for (const CodecRow& row : rows) {
    if (row.type != type) continue;
    sizes.push_back(row.bytes);
    total += double(row.bytes);
  }
  FrameSizes s;
  s.count = sizes.size();
  if (sizes.empty()) return s;
  std::sort(sizes.begin(), sizes.end());
  // Nearest rank: the ceil(p n)-th smallest.
  const auto rank = [&](std::size_t percent) {
    return sizes[(s.count * percent + 99) / 100 - 1];
  };
  s.mean = total / double(s.count);
  s.p50 = rank(50);
  s.p95 = rank(95);
  s.max = sizes.back();
  return s;
}

/// @return A GOP's bytes a frame at intra period @p period: an I-frame and
///         @p period - 1 P-frames, each at its type's mean (@p intra's and
///         @p inter's), the rate a long stream settles at whatever I-frames
///         its own run forced. @pre @p period is at least 1.
inline double gop_bytes(const FrameSizes& intra, const FrameSizes& inter,
                        std::uint32_t period) {
  VKC_CHECK(period >= 1, "gop_bytes needs a period of at least 1");
  return (intra.mean + double(period - 1) * inter.mean) / double(period);
}

/// @return @p bytes_per_frame at @p frames_per_s, in Mbit/s.
inline double mbit_per_s(double bytes_per_frame, double frames_per_s) {
  return bytes_per_frame * 8.0 * frames_per_s / 1e6;
}

/// The rate a source with no time of its own is coded at: Replica's frames
/// and a mesh sequence's numbers.
inline constexpr double kNominalFps = 30.0;

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
  /// returns, a record each on its one Geometry track, an I-frame's a key.
  /// Nothing may be at the path, and its directory must exist.
  std::string stream;
  /// Where to serve those records live, as they are recorded
  /// (`networking::StreamServer`, the session the stream's); none serves
  /// nothing. Defaulted, so two-field braces stay quiet under
  /// -Wmissing-field-initializers.
  std::optional<vr::networking::ServerOptions> serve = std::nullopt;
};

/// @brief Refuse to write @p path when it is @p file under another name,
///        through a link or a case alias: the core compares paths as text.
/// @return OK when nothing is at @p path or at @p file, or they are two
///         files; InvalidArgument naming both when they are one; IoError
///         when that cannot be told, as for a name too long to look up.
inline vkc::Status check_not_same_file(const std::string& path,
                                       const std::string& file) {
  // Both checked for existence first: libc++'s `equivalent` reports an
  // error when one is missing, libstdc++'s returns false.
  std::error_code error;
  const bool same = std::filesystem::exists(path, error) &&
                    std::filesystem::exists(file, error) &&
                    std::filesystem::equivalent(path, file, error);
  if (error) {
    return vkc::Status::io_error("cannot tell whether " + path + " is " + file);
  }
  if (same) {
    return vkc::Status::invalid_argument(path + " is the same file as " + file);
  }
  return {};
}

/// @brief An encoder, a decoder and the player grid it decodes into, with a
///        row for every frame decoded through them.
class CodecStream {
 public:
  /// The player grid is built from the first frame's header, sized for its
  /// blocks, and grows as later frames need. @p outputs are made here,
  /// after the codec objects and before any coding: the server first, so a
  /// port in use touches no file, then the stream, as a rerun onto its file
  /// is the common refusal, then the CSV, its header flushed.
  /// @param program  The program writing the stream, recorded in its session
  ///                 header.
  /// @param geometry_params  Its Geometry track's params, recorded as given:
  ///                 empty, or a mesh source's `io::encode_transform` bytes
  ///                 (stream/session.hpp).
  /// @return The stream; `StreamServer::start`'s error for the server
  ///         (IoError for a port in use); `StreamWriter::create`'s error for
  ///         the stream (InvalidArgument when something is at its path,
  ///         NotFound when its directory is missing);
  ///         @ref check_not_same_file's error when the CSV is the stream's
  ///         file; IoError naming a CSV that cannot be written; or the
  ///         `Encoder::create` / `Decoder::create` error. Each CSV error
  ///         removes the new stream's file again.
  static vkc::Result<CodecStream> create(vkc::Device& device,
                                         vkc::Allocator& allocator,
                                         const vr::codec::EncoderConfig& config,
                                         CodecOutputs outputs = {},
                                         const std::string& program = {},
                                         std::string geometry_params = {}) {
    VKC_ASSIGN(vr::codec::Encoder enc,
               vr::codec::Encoder::create(device, allocator, config));
    VKC_ASSIGN(vr::codec::Decoder dec,
               vr::codec::Decoder::create(device, allocator));
    // One session for the file and the viewers. No session id or start
    // time, so a rerun writes the same bytes.
    vr::stream::Track geometry;
    geometry.kind = vr::stream::TrackKind::Geometry;
    geometry.params = std::move(geometry_params);
    vr::stream::Session session;
    session.writer = program;
    session.tracks.push_back(std::move(geometry));
    std::unique_ptr<vr::networking::StreamServer> server;
    if (outputs.serve) {
      VKC_ASSIGN(server,
                 vr::networking::StreamServer::start(session, *outputs.serve));
    }
    std::unique_ptr<vr::stream::StreamWriter> writer;
    if (!outputs.stream.empty()) {
      VKC_ASSIGN(writer,
                 vr::stream::StreamWriter::create(outputs.stream, session));
    }
    // The core writes whole files only, so the CSV, extended a row at a time,
    // is the stream's own.
    std::ofstream csv;
    if (!outputs.csv.empty()) {
      if (writer) {
        // Creating the stream first resolves dangling symlinks and case
        // aliases. Check identity before opening the CSV can truncate it.
        const vkc::Status apart =
            check_not_same_file(outputs.csv, outputs.stream);
        if (!apart.ok()) {
          take_back(writer, outputs.stream);
          return apart.with_context("CodecStream: CSV");
        }
      }
      csv.open(outputs.csv, std::ios::binary | std::ios::trunc);
      csv << kCodecRowsCsvHeader << std::flush;
      if (!csv) {
        take_back(writer, outputs.stream);
        return csv_error(outputs.csv);
      }
    }
    return CodecStream(device, allocator, std::move(enc), std::move(dec),
                       config.intra_period, std::move(outputs),
                       std::move(server), std::move(writer), std::move(csv));
  }

  CodecStream(CodecStream&&) = default;
  CodecStream& operator=(CodecStream&&) = delete;

  /// Closes the viewers and the stream's file. One never finished that holds
  /// no record, as after a run that failed before its first frame, is
  /// removed, so a rerun onto it is not refused; one that holds records
  /// stays, read as a killed writer's.
  ~CodecStream() {
    if (!finished_ && coded_ == 0) take_back(writer_, outputs_.stream);
  }

  /// @brief Encode @p source as one frame, decode it into the player, and
  ///        keep its row. Files are written outside the codec timings: the
  ///        frame to the stream as soon as it is encoded, so a frame the
  ///        decoder refuses is kept, then to the viewers, and once it
  ///        decodes, its row to the CSV, flushed. The encoder codes its next
  ///        P-frame against every frame it returns, so any failure after the
  ///        encode makes the next frame an I-frame (`Encoder::force_intra`):
  ///        a frame the stream or the player lacks is never a reference. A
  ///        frame the server refuses is logged at Warning and never fails the
  ///        call: the viewers lose the rest of its GOP, the recording
  ///        nothing.
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
    const vkc::Status played = play(bytes, frame, pts_ns, encoded);
    if (!played.ok()) encoder_.force_intra();
    return played;
  }

  /// @brief Close the viewers, which see the stream end, and finish the
  ///        stream's file with its footer index (`StreamWriter::finish`);
  ///        without either, only mark it finished. One attempt: however it
  ///        ends, every later call, this one included, is refused, so a file
  ///        whose footer failed stays unfinished, read as a killed writer's.
  /// @return OK; InvalidArgument once finished; or the writer's error.
  vkc::Status finish() {
    if (finished_) {
      return vkc::Status::invalid_argument("CodecStream: finished");
    }
    finished_ = true;
    server_.reset();
    return writer_ ? writer_->finish() : vkc::Status{};
  }

  /// @return The server the frames are served on; null when not serving, or
  ///         once finished.
  const vr::networking::StreamServer* server() const noexcept {
    return server_.get();
  }

  /// @return A row per frame decoded so far, in coding order.
  const std::vector<CodecRow>& rows() const noexcept { return rows_; }

  /// @return The frames encoded and recorded so far, a frame the decoder
  ///         refused among them (it has no row); the destructor removes an
  ///         unfinished stream's file only while this is 0.
  std::uint64_t coded() const noexcept { return coded_; }

  /// @return The pts of the last frame recorded, below which @ref code
  ///         refuses one; 0 before the first.
  std::uint64_t last_pts_ns() const noexcept { return last_pts_ns_; }

  /// @return The grid the last frame was decoded into. @pre rows() is not
  ///         empty.
  vr::volume::VoxelBlockGrid& player() { return *player_; }

  /// @return The encode stage rows of every decoded frame of @p type,
  ///         summed: what report() prints, a frame of that type each.
  const vkc::StageMetrics& encode_stages(vr::codec::FrameType type) const {
    return encode_rows_[static_cast<std::size_t>(type)];
  }

  /// @return As @ref encode_stages, the decode's.
  const vkc::StageMetrics& decode_stages(vr::codec::FrameType type) const {
    return decode_rows_[static_cast<std::size_t>(type)];
  }

  /// @brief Print what the stream cost: its size and bitrate, a GOP's
  ///        bitrate at the intra period, and for each frame type its sizes
  ///        (@ref frame_sizes) and where its encode and decode time went.
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
    const double coded_fps = every > 0 ? fps / double(every) : 0.0;
    std::printf("stream: %zu frames coded%s\n", rows_.size(),
                grew_ ? " (the player grid grew)" : "");
    std::printf(
        "  %.0f bytes per frame, %.0f blocks; %.1f bytes per block, %.0fx "
        "under raw",
        per_frame, blocks / frames, per_block,
        per_block > 0 ? kRawBytesPerBlock / per_block : 0.0);
    if (every > 0) {
      std::printf("; %.2f Mbit/s at %.1f coded frames/s (1 in %d at %.0f fps)",
                  mbit_per_s(per_frame, coded_fps), coded_fps, every, fps);
    }
    std::printf("\n");
    const vr::codec::FrameType types[] = {vr::codec::FrameType::kIntra,
                                          vr::codec::FrameType::kInter};
    const FrameSizes sizes[] = {frame_sizes(rows_, types[0]),
                                frame_sizes(rows_, types[1])};
    if (every > 0 && intra_period_ > 1 && sizes[0].count > 0 &&
        sizes[1].count > 0) {
      std::printf(
          "  GOP of %u: %.2f Mbit/s, an I-frame and %u P-frames at their "
          "means\n",
          intra_period_,
          mbit_per_s(gop_bytes(sizes[0], sizes[1], intra_period_), coded_fps),
          intra_period_ - 1);
    }
    for (std::size_t t = 0; t < 2; ++t) {
      const FrameSizes& s = sizes[t];
      if (s.count == 0) continue;
      const std::string name =
          std::string(frame_type_name(types[t])) + "-frame";
      std::printf("  %zu %ss: bytes mean %.0f, p50 %zu, p95 %zu, max %zu\n",
                  s.count, name.c_str(), s.mean, s.p50, s.p95, s.max);
      const vkc::StageMetrics& encode = encode_stages(types[t]);
      const vkc::StageMetrics& decode = decode_stages(types[t]);
      print_stage_rows("encode, per " + name, encode, s.count, 4);
      print_stage_rows("decode, per " + name, decode, s.count, 4);
      const double enc = row_ms(encode, "  ..rans encode") / double(s.count);
      const double dec = row_ms(decode, "  ..rans decode") / double(s.count);
      std::printf("    rANS: %.2f ms encode, %.2f ms decode wall clock per %s",
                  enc, dec, name.c_str());
      if (every > 0) {
        // Against the source's interval: what coding every frame live must
        // fit. A standalone mesh or final-grid snapshot has no measured
        // frame rate.
        const double budget = 1e3 / fps;
        std::printf(" (%.0f%% / %.0f%% of a %.1f ms source frame interval)",
                    100.0 * enc / budget, 100.0 * dec / budget, budget);
      }
      std::printf("\n");
    }
  }

 private:
  CodecStream(vkc::Device& device, vkc::Allocator& allocator,
              vr::codec::Encoder enc, vr::codec::Decoder dec,
              std::uint32_t intra_period, CodecOutputs outputs,
              std::unique_ptr<vr::networking::StreamServer> server,
              std::unique_ptr<vr::stream::StreamWriter> writer,
              std::ofstream csv)
      : device_(&device),
        allocator_(&allocator),
        encoder_(std::move(enc)),
        decoder_(std::move(dec)),
        intra_period_(intra_period),
        outputs_(std::move(outputs)),
        server_(std::move(server)),
        writer_(std::move(writer)),
        csv_(std::move(csv)) {}

  static vkc::Status csv_error(const std::string& path) {
    return vkc::Status::io_error("CodecStream: cannot write the CSV " + path);
  }

  // Close a stream that holds no record and remove its file at `path`, so a
  // rerun is refused for its real cause, not for this empty stream.
  static void take_back(std::unique_ptr<vr::stream::StreamWriter>& writer,
                        const std::string& path) {
    if (!writer) return;
    writer.reset();
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
  }

  // What code() does once the encoder returns frame @p frame's @p bytes:
  // record and serve them, decode them into the player, and keep and write
  // their row, @p encoded its encode's stage rows.
  vkc::Status play(const Bytes& bytes, std::size_t frame, std::uint64_t pts_ns,
                   const vkc::StageMetrics& encoded) {
    VKC_ASSIGN(const vr::codec::FrameInfo info,
               vr::codec::read_frame_info(bytes.data(), bytes.size()));
    vr::stream::RecordInfo record;
    // A reader starts, or seeks, at a key: a frame that decodes alone.
    record.key = info.type == vr::codec::FrameType::kIntra;
    record.pts_ns = pts_ns;
    record.seq = coded_;
    record.set = frame;
    if (writer_) VKC_TRY(writer_->append(record, bytes.data(), bytes.size()));
    // After the file, so no viewer gets a frame the file lacks. Returning a
    // refusal would leave coded_ behind the file, whose writer would then
    // refuse the next seq.
    if (server_) {
      const vkc::Status served =
          server_->publish(record, bytes.data(), bytes.size());
      if (!served.ok()) {
        vr::log_message(vkc::LogLevel::Warning,
                        "CodecStream: frame " + std::to_string(frame) +
                            " not served: " + served.message());
      }
    }
    ++coded_;
    last_frame_ = frame;
    last_pts_ns_ = pts_ns;
    if (!player_) {
      VKC_ASSIGN(vr::volume::VoxelBlockGrid g,
                 vr::codec::make_decode_grid(*device_, *allocator_,
                                             bytes.data(), bytes.size()));
      player_.emplace(std::move(g));
    }
    const std::int32_t buckets = player_->grid().num_buckets;
    vkc::StageMetrics decoded;
    const vkc::Status status = vr::codec::decode_growing(
        decoder_, bytes.data(), bytes.size(), *player_, &decoded);
    grew_ = grew_ || player_->grid().num_buckets != buckets;
    VKC_TRY(status);
    const auto t = static_cast<std::size_t>(info.type);
    encode_rows_[t].merge(encoded);
    decode_rows_[t].merge(decoded);
    rows_.push_back(make_codec_row(frame, info.type, bytes.size(),
                                   info.block_count, encoded, decoded));
    if (csv_.is_open()) {
      csv_ << codec_row_csv(rows_.back()) << std::flush;
      if (!csv_) return csv_error(outputs_.csv);
    }
    return {};
  }

  vkc::Device* device_;
  vkc::Allocator* allocator_;
  vr::codec::Encoder encoder_;
  vr::codec::Decoder decoder_;
  std::uint32_t intra_period_;  // the encoder's, for a GOP's bitrate
  CodecOutputs outputs_;
  // Null when not serving, or once finished.
  std::unique_ptr<vr::networking::StreamServer> server_;
  std::unique_ptr<vr::stream::StreamWriter> writer_;  // null without a stream
  std::ofstream csv_;           // open when outputs_.csv names a file
  std::uint64_t coded_ = 0;     // frames encoded and recorded: the next seq
  std::size_t last_frame_ = 0;  // the last of them, once coded_ > 0
  std::uint64_t last_pts_ns_ = 0;
  bool finished_ = false;
  std::optional<vr::volume::VoxelBlockGrid> player_;
  // Every frame's, summed by stage, by frame type: I-frames', then P-frames'.
  vkc::StageMetrics encode_rows_[2];
  vkc::StageMetrics decode_rows_[2];
  std::vector<CodecRow> rows_;
  bool grew_ = false;  // a decode grew the player grid
};

}  // namespace vr_example
