// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// A codec stream's per-frame rows (examples/common/codec_rows.hpp): a row's
// host and device milliseconds from its calls' stage rows, the nearest-rank
// percentiles, the p50 / p95 / max table, the CSV lines, and the frame file
// names. Host only, strings only: the stream test writes the files.

#include <cstdio>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "codec_rows.hpp"
#include "test_check.hpp"

namespace vkc = volumetric_kit::core;

namespace {

int rows() {
  // Host: the stage, its breakdowns inside it. Device: every span timed,
  // breakdowns included, as they are separate dispatches.
  vkc::StageMetrics encode;
  encode.add_cpu("codec encode", 3.0);
  encode.add_gpu("codec encode", 1.0);
  encode.add_cpu("  ..active set", 0.5);
  encode.add_gpu("  ..active set", 0.25);
  vkc::StageMetrics decode;  // no timer ran
  decode.add_cpu("codec decode", 2.0);
  decode.add_cpu("  ..rans decode", 1.5);
  const vr_example::CodecRow row =
      vr_example::make_codec_row(7, 4096, 12, encode, decode);
  CHECK(row.frame == 7 && row.bytes == 4096 && row.blocks == 12);
  CHECK(row.encode_ms == 3.0 && row.encode_device_ms == 1.25);
  CHECK(row.decode_ms == 2.0 && !row.decode_device_ms);
  // The sweep's device columns count the same way.
  CHECK(vr_example::device_ms(encode) == 1.25 &&
        !vr_example::device_ms(decode));
  return 0;
}

int percentiles() {
  // The ceil(p n)-th smallest, whatever order the values came in: of 1..20,
  // the 10th and the 19th.
  std::vector<double> values;
  for (int v = 20; v >= 1; --v) values.push_back(v);
  vr_example::Percentiles p = vr_example::percentiles(values);
  CHECK(p.count == 20 && p.p50 == 10.0 && p.p95 == 19.0 && p.max == 20.0);
  // Of two, the first is the median and the second the p95.
  p = vr_example::percentiles({4.0, 2.0});
  CHECK(p.p50 == 2.0 && p.p95 == 4.0 && p.max == 4.0);
  p = vr_example::percentiles({0.5});
  CHECK(p.count == 1 && p.p50 == 0.5 && p.p95 == 0.5 && p.max == 0.5);
  p = vr_example::percentiles({});
  CHECK(p.count == 0 && p.p50 == 0.0 && p.p95 == 0.0 && p.max == 0.0);
  return 0;
}

std::vector<vr_example::CodecRow> sample_rows() {
  return {{3, 1000, 10, 1.5, 0.5, 2.0, std::nullopt},
          {7, 3000, 30, 2.5, std::nullopt, 1.0, std::nullopt},
          {11, 2000, 20, 0.25, 0.75, 3.0, std::nullopt}};
}

int table() {
  // A device column over the frames that timed it, said so; one that none
  // did is left out.
  const std::string text = vr_example::format_codec_rows(sample_rows(), 2);
  std::fputs(text.c_str(), stdout);
  CHECK(text ==
        "  per coded frame (p50 / p95 / max):\n"
        "    bytes                    2000       3000       3000\n"
        "    blocks                     20         30         30\n"
        "    encode host ms          1.500      2.500      2.500\n"
        "    encode device ms        0.500      0.750      0.750  (2 of 3 "
        "frames)\n"
        "    decode host ms          2.000      3.000      3.000\n");
  CHECK(vr_example::format_codec_rows({}).empty());
  return 0;
}

int csv() {
  // The header, then a line per row, an unset device column empty.
  std::string text = vr_example::kCodecRowsCsvHeader;
  for (const vr_example::CodecRow& row : sample_rows()) {
    text += vr_example::codec_row_csv(row);
  }
  CHECK(text ==
        "frame,bytes,blocks,encode_host_ms,encode_device_ms,decode_host_ms,"
        "decode_device_ms\n"
        "3,1000,10,1.500,0.500,2.000,\n"
        "7,3000,30,2.500,,1.000,\n"
        "11,2000,20,0.250,0.750,3.000,\n");
  return 0;
}

int frame_files() {
  // Named by the source's index, five digits or more.
  const std::filesystem::path root = std::filesystem::path("frames") / "room0";
  CHECK(vr_example::frame_file(root.string(), 42) ==
        (root / "frame_00042.vrtc").string());
  CHECK(vr_example::frame_file(root.string(), 123456) ==
        (root / "frame_123456.vrtc").string());
  return 0;
}

}  // namespace

int main() {
  CHECK(rows() == 0);
  CHECK(percentiles() == 0);
  CHECK(table() == 0);
  CHECK(csv() == 0);
  CHECK(frame_files() == 0);
  std::puts("example_codec_rows: OK");
  return 0;
}
