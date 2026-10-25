// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// A codec stream as a player sees it: encode a grid, decode the frame, unless
// told not to, into a player grid built from the first frame's header and
// grown as frames need (codec/decode_grid.hpp), and keep a row per coded
// frame, which the report sums by frame type and the CSV lists; the frames
// themselves go to a .vrst file, each I-frame a key, and to the viewers of a
// live server. An I-frame after a frame the player missed is the stream's
// policy, kept here.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <ios>
#include <memory>
#include <optional>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "set_log.hpp"
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

/// @return A row's device milliseconds, or 0 if @p m has no row of that
///         name or it was not timed on the device.
inline double row_device_ms(const vkc::StageMetrics& m, const char* name) {
  const vkc::StageRow* row = find_row(m, name);
  return row != nullptr && row->device == vkc::DeviceTime::Measured
             ? row->gpu_ms
             : 0.0;
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
///        span, and both decode columns where the frame was not decoded.
struct CodecRow {
  std::size_t frame = 0;  ///< The source's index of the frame coded.
  /// An I-frame, or a P-frame on the frame coded before.
  vr::codec::FrameType type = vr::codec::FrameType::kIntra;
  std::size_t bytes = 0;     ///< The compressed frame's size.
  std::uint32_t blocks = 0;  ///< The blocks it carries.
  /// Encode, host busy: its waits on the device left out, wherever they ran.
  double encode_ms = 0.0;
  std::optional<double> encode_device_ms;  ///< Encode, device.
  std::optional<double> decode_ms;         ///< Decode, host busy.
  std::optional<double> decode_device_ms;  ///< Decode, device.
};

/// @return A codec call's device time, its stages' summed
///         (`StageMetrics::total_gpu_ms`: a breakdown's is inside its
///         stage's); unset when no stage was timed, or when one with a device
///         half was not, so a partial sum never passes for the call's.
inline std::optional<double> device_ms(const vkc::StageMetrics& m) {
  bool measured = false;
  for (const vkc::StageRow& row : m.rows()) {
    if (row.device == vkc::DeviceTime::None ||
        std::strncmp(row.name, vkc::StageMetrics::kBreakdownPrefix,
                     std::strlen(vkc::StageMetrics::kBreakdownPrefix)) == 0) {
      continue;
    }
    if (row.device != vkc::DeviceTime::Measured) return std::nullopt;
    measured = true;
  }
  if (!measured) return std::nullopt;
  return m.total_gpu_ms();
}

/// @brief A frame's row from the stage rows its encode and decode filled:
///        each call's host busy span (`StageMetrics::total_cpu_ms` less
///        `total_wait_ms`, its stages without the breakdowns inside them or
///        their waits on the device: one meaning whether a call waits inside
///        its rows, as `Encoder::encode` does, or a sender waits outside
///        them) and its @ref device_ms.
/// @param decode  The decode's rows, or null for a frame not decoded.
inline CodecRow make_codec_row(std::size_t frame, vr::codec::FrameType type,
                               std::size_t bytes, std::uint32_t blocks,
                               const vkc::StageMetrics& encode,
                               const vkc::StageMetrics* decode) {
  CodecRow row;
  row.frame = frame;
  row.type = type;
  row.bytes = bytes;
  row.blocks = blocks;
  row.encode_ms = encode.total_cpu_ms() - encode.total_wait_ms();
  row.encode_device_ms = device_ms(encode);
  if (decode != nullptr) {
    row.decode_ms = decode->total_cpu_ms() - decode->total_wait_ms();
    row.decode_device_ms = device_ms(*decode);
  }
  return row;
}

/// The first line of a stream's CSV, naming @ref codec_row_csv's columns.
inline constexpr const char kCodecRowsCsvHeader[] =
    "frame,type,bytes,blocks,encode_host_ms,encode_device_ms,decode_host_ms,"
    "decode_device_ms\n";

/// @return @p row as a line under @ref kCodecRowsCsvHeader: its type as
///         @ref frame_type_name names it, milliseconds to the microsecond,
///         and an unset column empty.
inline std::string codec_row_csv(const CodecRow& row) {
  const auto ms = [](const std::optional<double>& value) {
    char text[32] = "";
    if (value) std::snprintf(text, sizeof text, "%.3f", *value);
    return std::string(text);
  };
  char text[64];
  std::snprintf(text, sizeof text, "%zu,%s,%zu,%u,", row.frame,
                frame_type_name(row.type), row.bytes, row.blocks);
  return text + ms(row.encode_ms) + "," + ms(row.encode_device_ms) + "," +
         ms(row.decode_ms) + "," + ms(row.decode_device_ms) + "\n";
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
  s.mean = total / double(s.count);
  s.p50 = nearest_rank(sizes, 50);
  s.p95 = nearest_rank(sizes, 95);
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

/// The rate a source with no time of its own is coded at, frames a second:
/// Replica's frames and a mesh sequence's numbers.
inline constexpr std::uint32_t kNominalFps = 30;

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
  /// frame with a row; replaced if it exists. Must not alias the stream.
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
  /// Decode each frame into the player grid, for its row's decode columns,
  /// the report's decode rows and @ref CodecStream::player. Off, a row
  /// carries the encode alone, and no decoder or player grid is made.
  bool decode = true;
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

/// @brief An encoder, with @ref CodecOutputs::decode a decoder and the player
///        grid it decodes into, and a row for every frame coded through
///        them.
class CodecStream {
 public:
  /// The player grid is built from the first frame's header, sized for its
  /// blocks, and grows as later frames need. @p outputs are made here,
  /// before the codec objects, so a refused output costs no kernel: the
  /// server first, so a port in use touches no file, then the stream, as a
  /// rerun onto its file is the common refusal, then the CSV, its header
  /// flushed.
  /// @param program  The program writing the stream, recorded in its session
  ///                 header.
  /// @param rate     Its Geometry track's nominal rate, the source's: unknown
  ///                 by default.
  /// @param geometry_params  Its Geometry track's params, recorded as given:
  ///                 empty, or a mesh source's `io::encode_transform` bytes
  ///                 (stream/session.hpp).
  /// @return The stream; `StreamServer::start`'s error for the server
  ///         (IoError for a port in use); `StreamWriter::create`'s error for
  ///         the stream (InvalidArgument when something is at its path,
  ///         NotFound when its directory is missing);
  ///         @ref check_not_same_file's error when the CSV is the stream's
  ///         file; IoError naming a CSV that cannot be written; or the
  ///         `Encoder::create` / `Decoder::create` error. Each error after
  ///         the stream is made removes its file again, and a codec error
  ///         the CSV too.
  static vkc::Result<CodecStream> create(vkc::Device& device,
                                         vkc::Allocator& allocator,
                                         const vr::codec::EncoderConfig& config,
                                         CodecOutputs outputs = {},
                                         const std::string& program = {},
                                         vr::stream::Rate rate = {},
                                         std::string geometry_params = {}) {
    // One session for the file and the viewers. No session id or start
    // time, so a rerun writes the same bytes.
    vr::stream::Track geometry;
    geometry.kind = vr::stream::TrackKind::Geometry;
    geometry.rate = rate;
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
    // A codec error takes back the files made above.
    const auto refuse = [&](const vkc::Status& status) {
      take_back(writer, outputs.stream);
      if (csv.is_open()) {
        csv.close();
        std::error_code ignored;
        std::filesystem::remove(outputs.csv, ignored);
      }
      return status;
    };
    vkc::Result<vr::codec::Encoder> enc =
        vr::codec::Encoder::create(device, allocator, config);
    if (!enc.ok()) return refuse(enc.status());
    std::optional<vr::codec::Decoder> dec;
    if (outputs.decode) {
      vkc::Result<vr::codec::Decoder> made =
          vr::codec::Decoder::create(device, allocator);
      if (!made.ok()) return refuse(made.status());
      dec.emplace(std::move(made).value());
    }
    return CodecStream(device, allocator, std::move(enc).value(),
                       std::move(dec), config.intra_period, std::move(outputs),
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
  ///        nothing. Without @ref CodecOutputs::decode, no frame is decoded.
  /// @param frame   The source's index of the frame: its row's, and its
  ///                record's set. Above every frame coded before.
  /// @param pts_ns  Its time, its record's pts: no earlier than the frames'
  ///                coded before (@ref frame_pts_ns).
  /// @param set     Optional: the sender's set this frame codes, whose rows
  ///                take the encode's and the `"write"` and `"publish"`
  ///                rows, and which is published once the frame is
  ///                (`SetLog::Set::published`), before the decode.
  /// @return OK; InvalidArgument, before anything is encoded, for a
  ///         @p frame or @p pts_ns out of order or a finished stream; the
  ///         encode or decode error; `StreamWriter::append`'s error; or
  ///         IoError naming the CSV.
  vkc::Status code(vr::volume::VoxelBlockGrid& source, std::size_t frame,
                   std::uint64_t pts_ns, SetLog::Set* set = nullptr) {
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
    if (set != nullptr) set->rows.merge(encoded);
    const vkc::Status played =
        play(vr::stream::RecordPayload::of(bytes.data(), bytes.size()), frame,
             pts_ns, encoded,
             static_cast<std::uint32_t>(source.grid().num_blocks), set);
    if (!played.ok()) encoder_.force_intra();
    return played;
  }

  /// @brief Record @p source's frame's front half into @p front, after what
  ///        it holds -- a sender's fusion -- for @ref finish
  ///        (`Encoder::record_front`), its rows the frame's own.
  /// @return The pending frame; InvalidArgument for a finished stream; or
  ///         the encoder's refusal, with nothing recorded.
  vkc::Result<vr::codec::PendingEncode> record(
      vkc::CommandBatch& front, vr::volume::VoxelBlockGrid& source) {
    if (finished_) {
      return vkc::Status::invalid_argument("CodecStream: finished");
    }
    front_rows_ = vkc::StageMetrics{};
    max_blocks_ = static_cast<std::uint32_t>(source.grid().num_blocks);
    return encoder_.record_front(front, source, &front_rows_);
  }

  /// @brief Once the front batch @ref record recorded into has resolved:
  ///        its device rows into the frame's rows where the front half placed
  ///        that row (`Encoder::record_front` places its stage's and each
  ///        breakdown's as it opens), the rest into @p rest, when given.
  void front_ran(const std::vector<vkc::StageRow>& device_rows,
                 vkc::StageMetrics* rest) {
    std::vector<vkc::StageRow> encode;
    std::vector<vkc::StageRow> others;
    for (const vkc::StageRow& row : device_rows) {
      (find_row(front_rows_, row.name) != nullptr ? encode : others)
          .push_back(row);
    }
    front_rows_.merge_device(encode);
    if (rest != nullptr) rest->merge_device(others);
  }

  /// @brief The front half again, in a batch of its own and waited on: what
  ///        a sender does after a fusion done again (`FuseReport::redone`,
  ///        rare), whose front batch encoded the grid without the set. The
  ///        frame's rows keep the first front half's, which the device ran
  ///        too, and add this one's.
  /// @param rows  Optional: the set's rows, which take the wait
  ///              (`"wait: redo"`).
  /// @return The new pending frame, @p pending dropped; or the encoder's or
  ///         the batch's failure.
  vkc::Result<vr::codec::PendingEncode> redo(
      const vr::codec::PendingEncode& pending,
      vr::volume::VoxelBlockGrid& source, vkc::StageMetrics* rows) {
    encoder_.abandon(pending);
    const vkc::StageMetrics first = std::move(front_rows_);
    vkc::CommandBatch front(*device_, *allocator_);
    VKC_ASSIGN(const vr::codec::PendingEncode again, record(front, source));
    const vkc::Status ran = [&]() -> vkc::Status {
      VKC_ASSIGN(vkc::PendingBatch done, front.submit_async({}, {}));
      {
        const vkc::WaitScope wait(rows, "wait: redo");
        VKC_TRY(done.wait());
      }
      front_ran(done.device_rows(), nullptr);
      return {};
    }();
    if (!ran.ok()) {
      encoder_.abandon(again);
      return ran;
    }
    vkc::StageMetrics both = first;
    both.merge(front_rows_);
    front_rows_ = std::move(both);
    return again;
  }

  /// @brief Once @p pending's front batch has resolved (@ref front_ran):
  ///        code its rANS half in a batch of its own and wait on it
  ///        (`"wait: frame"`), finish the frame, then play it as @ref code
  ///        does -- written (`"write"`) and published (`"publish"`) as its
  ///        head and payload lie, under one CRC (`stream::RecordPayload`,
  ///        `"crc"`),
  ///        the set published, and decoded, when the stream decodes, from
  ///        the decoder's input slot they are copied into.
  /// @param frame   As @ref code.
  /// @param pts_ns  As @ref code.
  /// @param set     The sender's set: its rows take the frame's and the
  ///                waits', its type and whether its payload took a tail.
  /// @return As @ref code; an order refusal drops @p pending first.
  vkc::Status finish(const vr::codec::PendingEncode& pending, std::size_t frame,
                     std::uint64_t pts_ns, SetLog::Set& set) {
    if (finished_ ||
        (coded_ > 0 && (frame <= last_frame_ || pts_ns < last_pts_ns_))) {
      encoder_.abandon(pending);
      return vkc::Status::invalid_argument(
          "CodecStream: frame " + std::to_string(frame) + " at " +
          std::to_string(pts_ns) + " ns does not follow frame " +
          std::to_string(last_frame_) + " at " + std::to_string(last_pts_ns_) +
          " ns, or the stream is finished");
    }
    vkc::StageMetrics back_rows;
    vkc::CommandBatch back(*device_, *allocator_);
    const vkc::Status coded = [&]() -> vkc::Status {
      VKC_TRY(encoder_.record_back(back, pending, &back_rows));
      VKC_ASSIGN(vkc::PendingBatch done, back.submit_async({}, {}));
      {
        // The output: waited on at once.
        const vkc::WaitScope wait(&set.rows, "wait: frame");
        VKC_TRY(done.wait());
      }
      back_rows.merge_device(done.device_rows());
      return {};
    }();
    if (!coded.ok()) {
      encoder_.abandon(pending);
      return coded;
    }
    VKC_ASSIGN(const vr::codec::EncodedFrame encoded,
               encoder_.finish(pending, &back_rows));
    vkc::StageMetrics rows = front_rows_;
    rows.merge(back_rows);
    set.rows.merge(rows);
    set.tail = find_row(back_rows, "  ..tail") != nullptr;
    const vr::stream::RecordPayload payload = [&] {
      // The record's one CRC, for the file and the viewers alike.
      const vkc::StageScope crc(&set.rows, "crc");
      return vr::stream::RecordPayload::of(encoded.head.data(),
                                           encoded.head.size(), encoded.payload,
                                           encoded.payload_size);
    }();
    const vkc::Status played =
        play(payload, frame, pts_ns, rows, max_blocks_, &set);
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

  /// @return A row per frame recorded so far, and decoded when the stream
  ///         decodes, in coding order.
  const std::vector<CodecRow>& rows() const noexcept { return rows_; }

  /// @return The frames encoded and recorded so far, a frame the decoder
  ///         refused among them (it has no row); the destructor removes an
  ///         unfinished stream's file only while this is 0.
  std::uint64_t coded() const noexcept { return coded_; }

  /// @return The pts of the last frame recorded, below which @ref code
  ///         refuses one; 0 before the first.
  std::uint64_t last_pts_ns() const noexcept { return last_pts_ns_; }

  /// @return The grid the frames decode into, holding the last decoded; null
  ///         before the first frame is coded, and always when the stream
  ///         does not decode.
  vr::volume::VoxelBlockGrid* player() noexcept {
    return player_ ? &*player_ : nullptr;
  }

  /// @return The encode stage rows of every frame of @p type with a row,
  ///         summed: what report() prints, a frame of that type each.
  const vkc::StageMetrics& encode_stages(vr::codec::FrameType type) const {
    return encode_rows_[static_cast<std::size_t>(type)];
  }

  /// @return As @ref encode_stages, the decode's: none when the stream does
  ///         not decode.
  const vkc::StageMetrics& decode_stages(vr::codec::FrameType type) const {
    return decode_rows_[static_cast<std::size_t>(type)];
  }

  /// @brief Print what the stream cost: its size and bitrate, a GOP's
  ///        bitrate at the intra period, and for each frame type its sizes
  ///        (@ref frame_sizes) and where its encode time went, and its
  ///        decode's when the stream decodes.
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
      if (decoder_) print_stage_rows("decode, per " + name, decode, s.count, 4);
      // The coders' device time: their host halves are only their
      // recording, as each runs in its caller's batch.
      const double enc =
          row_device_ms(encode, "  ..rans encode") / double(s.count);
      const double dec =
          row_device_ms(decode, "  ..rans decode") / double(s.count);
      std::printf("    rANS: %.2f ms encode", enc);
      if (decoder_) std::printf(", %.2f ms decode", dec);
      std::printf(" on the device per %s", name.c_str());
      if (every > 0) {
        // Against the source's interval: what coding every frame live must
        // fit. A standalone mesh or final-grid snapshot has no measured
        // frame rate.
        const double budget = 1e3 / fps;
        std::printf(" (%.0f%%", 100.0 * enc / budget);
        if (decoder_) std::printf(" / %.0f%%", 100.0 * dec / budget);
        std::printf(" of a %.1f ms source frame interval)", budget);
      }
      std::printf("\n");
    }
  }

 private:
  CodecStream(vkc::Device& device, vkc::Allocator& allocator,
              vr::codec::Encoder enc, std::optional<vr::codec::Decoder> dec,
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

  // What code() and finish() do once the encoder returns frame @p frame's
  // @p bytes -- its head, then its payload where the encoder left it, under
  // one CRC: record and serve them, decode them into the player if the
  // stream decodes, from the decoder's input slot they are copied into, and
  // keep and write their row, @p encoded its encode's stage rows. The
  // source's slots, @p max_blocks, bound its frames' blocks: the player's
  // ceiling. @p set, when given, takes the write's and the publish's rows and
  // is published before the decode.
  vkc::Status play(const vr::stream::RecordPayload& bytes, std::size_t frame,
                   std::uint64_t pts_ns, const vkc::StageMetrics& encoded,
                   std::uint32_t max_blocks, SetLog::Set* set) {
    VKC_ASSIGN(const vr::codec::FrameInfo info,
               vr::codec::read_frame_info(bytes.first(), bytes.first_size()));
    vr::stream::RecordInfo record;
    // A reader starts, or seeks, at a key: a frame that decodes alone.
    record.key = info.type == vr::codec::FrameType::kIntra;
    record.pts_ns = pts_ns;
    record.seq = coded_;
    record.set = frame;
    vkc::StageMetrics* rows = set != nullptr ? &set->rows : nullptr;
    if (writer_) {
      const vkc::StageScope write(rows, "write");
      VKC_TRY(writer_->append(record, bytes));
    }
    // After the file, so no viewer gets a frame the file lacks. Returning a
    // refusal would leave coded_ behind the file, whose writer would then
    // refuse the next seq.
    if (server_) {
      const vkc::StageScope publish(rows, "publish");
      const vkc::Status served = server_->publish(record, bytes);
      if (!served.ok()) {
        vr::log_message(vkc::LogLevel::Warning,
                        "CodecStream: frame " + std::to_string(frame) +
                            " not served: " + served.message());
      }
    }
    ++coded_;
    last_frame_ = frame;
    last_pts_ns_ = pts_ns;
    if (set != nullptr) {
      set->type = info.type;
      set->published();
    }
    const auto t = static_cast<std::size_t>(info.type);
    vkc::StageMetrics decoded;
    if (decoder_) {
      // The frame's one copy on the way to the decoder: into its slot.
      VKC_ASSIGN(const vkc::HostSlot slot, decoder_->claim_input(bytes.size()));
      if (bytes.first_size() != 0) {
        std::memcpy(slot.data(), bytes.first(), bytes.first_size());
      }
      if (bytes.second_size() != 0) {
        std::memcpy(slot.data() + bytes.first_size(), bytes.second(),
                    bytes.second_size());
      }
      if (!player_) {
        VKC_ASSIGN(
            vr::volume::VoxelBlockGrid g,
            vr::codec::make_decode_grid(*device_, *allocator_, slot.data(),
                                        bytes.size(), max_blocks));
        player_.emplace(std::move(g));
      }
      const std::int32_t buckets = player_->grid().num_buckets;
      const vkc::Status status = vr::codec::decode_growing(
          *decoder_, slot, bytes.size(), *player_, max_blocks, &decoded);
      grew_ = grew_ || player_->grid().num_buckets != buckets;
      VKC_TRY(status);
      decode_rows_[t].merge(decoded);
    }
    encode_rows_[t].merge(encoded);
    rows_.push_back(make_codec_row(frame, info.type, bytes.size(),
                                   info.block_count, encoded,
                                   decoder_ ? &decoded : nullptr));
    if (csv_.is_open()) {
      csv_ << codec_row_csv(rows_.back()) << std::flush;
      if (!csv_) return csv_error(outputs_.csv);
    }
    return {};
  }

  vkc::Device* device_;
  vkc::Allocator* allocator_;
  vr::codec::Encoder encoder_;
  std::optional<vr::codec::Decoder> decoder_;  // with outputs_.decode
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
  // The frame in flight between record() and finish(): its front half's
  // rows, and its source's slots.
  vkc::StageMetrics front_rows_;
  std::uint32_t max_blocks_ = 0;
};

}  // namespace vr_example
