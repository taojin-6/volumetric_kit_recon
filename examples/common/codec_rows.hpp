// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/codec_rows.hpp
/// @brief What a codec stream keeps of each coded frame: one row of its size
///        and costs, the rows' p50 / p95 / max, the rows as CSV lines, and
///        where its frame directory puts each frame. Header-only, on the
///        core's `StageMetrics` alone, so its tests need no device.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "volumetric_kit/core/base/stage_metrics.hpp"

namespace vr_example {

namespace vkc = volumetric_kit::core;

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

/// @brief A column's spread over the frames that have it.
struct Percentiles {
  double p50 = 0.0;       ///< The median, nearest rank.
  double p95 = 0.0;       ///< The 95th percentile, nearest rank.
  double max = 0.0;       ///< The largest value.
  std::size_t count = 0;  ///< The values it is over; all 0 when none.
};

/// @return @p values' nearest-rank p50 and p95 -- the ceil(p n)-th smallest,
///         as `eval` reports a p95 -- and their max.
inline Percentiles percentiles(std::vector<double> values) {
  Percentiles out;
  out.count = values.size();
  if (values.empty()) return out;
  std::sort(values.begin(), values.end());
  const auto rank = [&values](std::size_t percent) {
    return values[(values.size() * percent + 99) / 100 - 1];
  };
  out.p50 = rank(50);
  out.p95 = rank(95);
  out.max = values.back();
  return out;
}

/// @brief @p rows' p50 / p95 / max per column, a device column over the
///        frames that timed it and left out when none did:
///
///     per coded frame (p50 / p95 / max):
///       bytes                    2048       2100       2304
///       encode host ms          1.200      1.500      2.000
///
/// @param indent  Spaces before the title; the rows sit two further in.
/// @return The table, or an empty string for no rows.
inline std::string format_codec_rows(const std::vector<CodecRow>& rows,
                                     int indent = 0) {
  if (rows.empty()) return {};
  std::vector<double> bytes, blocks, encode, encode_device, decode,
      decode_device;
  for (const CodecRow& row : rows) {
    bytes.push_back(double(row.bytes));
    blocks.push_back(double(row.blocks));
    encode.push_back(row.encode_ms);
    decode.push_back(row.decode_ms);
    if (row.encode_device_ms) encode_device.push_back(*row.encode_device_ms);
    if (row.decode_device_ms) decode_device.push_back(*row.decode_device_ms);
  }
  const std::string pad(static_cast<std::size_t>(indent), ' ');
  std::string out = pad + "per coded frame (p50 / p95 / max):\n";
  const auto line = [&](const char* name, const std::vector<double>& values,
                        int decimals) {
    const Percentiles p = percentiles(values);
    if (p.count == 0) return;
    char text[160];
    std::snprintf(text, sizeof text, "%s  %-18s %10.*f %10.*f %10.*f",
                  pad.c_str(), name, decimals, p.p50, decimals, p.p95, decimals,
                  p.max);
    out += text;
    if (p.count < rows.size()) {
      std::snprintf(text, sizeof text, "  (%zu of %zu frames)", p.count,
                    rows.size());
      out += text;
    }
    out += "\n";
  };
  line("bytes", bytes, 0);
  line("blocks", blocks, 0);
  line("encode host ms", encode, 3);
  line("encode device ms", encode_device, 3);
  line("decode host ms", decode, 3);
  line("decode device ms", decode_device, 3);
  return out;
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

/// @return Where a frame directory @p dir holds frame @p frame:
///         `<dir>/frame_NNNNN.vrtc`, the source's index, five digits or more.
inline std::string frame_file(const std::string& dir, std::size_t frame) {
  char name[48];
  std::snprintf(name, sizeof name, "frame_%05zu.vrtc", frame);
  return (std::filesystem::path(dir) / name).string();
}

/// @brief Where a codec stream writes besides its report, each frame as it is
///        coded; an empty path writes nothing.
struct CodecOutputs {
  /// A file of @ref kCodecRowsCsvHeader, then each frame's @ref codec_row_csv;
  /// replaced if it exists.
  std::string csv;
  /// A directory, which must not exist yet, of every coded frame's
  /// @ref frame_file. Its parent must exist.
  std::string frame_directory;
};

}  // namespace vr_example
