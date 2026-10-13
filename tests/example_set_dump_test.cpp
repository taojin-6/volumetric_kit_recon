// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// fuse_orbbec's dump of its first sets (examples/common/set_dump.hpp): each
// camera's JPEG written byte for byte and, given the readback's views, its
// luma beside it as a PNG whose R, G and B are the luma and A opaque, row
// strides respected; a camera missing from the set, or a view missing,
// writes nothing; a frame with no JPEG (before anything is written), a file
// already there and ids or views not one per frame are refused. Over a run:
// the directory checked up front, made with the first set and not before,
// taken back when that set fails, no more sets than the limit, and the
// closing line, a warning when the run ended first. Host only.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "scratch_dir.hpp"
#include "set_dump.hpp"
#include "test_check.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/recon/io/image_io.hpp"
#include "volumetric_kit/recon/sensor/array/sensor_array.hpp"
#include "volumetric_kit/recon/sensor/rgbd_frame.hpp"
#include "volumetric_kit/recon/sensor/utils/picture_readback.hpp"

namespace vkc = volumetric_kit::core;
namespace io = volumetric_kit::recon::io;
namespace sensor = volumetric_kit::recon::sensor;

namespace {

using Code = vkc::Status::Code;
using Views = std::vector<std::optional<sensor::PictureView>>;

// A frame whose colour_bitstream is `bytes`, held by the frame.
sensor::RgbdFrame with_jpeg(std::vector<std::uint8_t> bytes) {
  auto held = std::make_shared<std::vector<std::uint8_t>>(std::move(bytes));
  sensor::RgbdFrame frame;
  frame.color_bitstream.codec = sensor::ColorBitstream::Codec::Jpeg;
  frame.color_bitstream.data = held->data();
  frame.color_bitstream.size = held->size();
  frame.pixels = std::move(held);
  return frame;
}

bool exists(const std::string& path) {
  std::error_code ignored;
  return std::filesystem::exists(path, ignored);
}

int test_names() {
  CHECK(vr_example::dump_file("d", 0, "CL2A141000N", ".jpg") ==
        "d/set_000_CL2A141000N.jpg");
  CHECK(vr_example::dump_file("d", 1234, "X", "_luma.png") ==
        "d/set_1234_X_luma.png");
  return 0;
}

// Each pixel's luma in R, G and B, A opaque, row by row past the stride.
int test_luma_rgba() {
  const std::vector<std::uint8_t> luma = {10, 20, 30, 0xee,  // stride 4
                                          40, 50, 60, 0xee};
  sensor::PictureView view;
  view.width = 3;
  view.height = 2;
  view.stride = 4;
  view.luma = luma.data();
  const std::vector<std::uint8_t> rgba = vr_example::luma_rgba(view);
  CHECK(rgba.size() == 3 * 2 * 4);
  const std::uint8_t expected[] = {10, 20, 30, 40, 50, 60};
  for (std::size_t i = 0; i < 6; ++i) {
    CHECK(rgba[4 * i] == expected[i] && rgba[4 * i + 1] == expected[i] &&
          rgba[4 * i + 2] == expected[i]);
    CHECK(rgba[4 * i + 3] == 255);
  }
  return 0;
}

int test_dump() {
  vr_test::Scratch scratch("vr_set_dump_");
  const std::string dir = scratch.path.string();
  const std::vector<std::string> ids = {"A", "B", "C"};
  const std::vector<std::uint8_t> jpeg_a = {0xff, 0xd8, 0x00, 0x01, 0xff, 0xd9};
  const std::vector<std::uint8_t> jpeg_c = {0xff, 0xd8, 0x7f, 0xff, 0xd9};

  // B missing from the set.
  sensor::FrameSet set;
  set.frames.resize(3);
  set.frames[0] = with_jpeg(jpeg_a);
  set.frames[2] = with_jpeg(jpeg_c);

  // A's luma, 5x3 in rows 8 bytes apart; none for C.
  constexpr std::uint32_t kW = 5;
  constexpr std::uint32_t kH = 3;
  constexpr std::size_t kStride = 8;
  std::vector<std::uint8_t> luma_a(kStride * kH, 0xee);  // padding: 0xee
  for (std::uint32_t y = 0; y < kH; ++y) {
    for (std::uint32_t x = 0; x < kW; ++x) {
      luma_a[y * kStride + x] = static_cast<std::uint8_t>(16 * y + 3 * x);
    }
  }
  sensor::PictureView view;
  view.width = kW;
  view.height = kH;
  view.stride = kStride;
  view.luma = luma_a.data();
  Views luma(3);
  luma[0] = view;

  CHECK(vr_example::dump_set(dir, 7, set, ids, &luma).ok());
  CHECK(vr_test::read_bytes(dir + "/set_007_A.jpg") == jpeg_a);
  CHECK(vr_test::read_bytes(dir + "/set_007_C.jpg") == jpeg_c);
  CHECK(!exists(dir + "/set_007_B.jpg"));
  CHECK(!exists(dir + "/set_007_C_luma.png"));
  const vkc::Result<std::vector<std::uint32_t>> png =
      io::load_color_packed(dir + "/set_007_A_luma.png", kW, kH);
  CHECK(png.ok());
  for (std::uint32_t y = 0; y < kH; ++y) {
    for (std::uint32_t x = 0; x < kW; ++x) {
      const std::uint32_t v = luma_a[y * kStride + x];
      CHECK(png.value()[y * kW + x] == (v | v << 8 | v << 16));  // R = G = B
    }
  }

  // No views: the JPEGs alone.
  CHECK(vr_example::dump_set(dir, 8, set, ids, nullptr).ok());
  CHECK(vr_test::read_bytes(dir + "/set_008_A.jpg") == jpeg_a);
  CHECK(!exists(dir + "/set_008_A_luma.png"));

  // A set dumped again: its first JPEG is already there, and is kept.
  const vkc::Status again = vr_example::dump_set(dir, 7, set, ids, &luma);
  CHECK(again.domain() == Code::InvalidArgument);
  CHECK(vr_test::read_bytes(dir + "/set_007_A.jpg") == jpeg_a);

  // A frame with no JPEG, as H.265 colour has none: refused, naming it,
  // before any of the set is written.
  set.frames[1] = sensor::RgbdFrame{};
  const vkc::Status no_jpeg = vr_example::dump_set(dir, 9, set, ids, nullptr);
  CHECK(no_jpeg.domain() == Code::InvalidArgument);
  CHECK(no_jpeg.message().find("camera B") != std::string::npos);
  CHECK(!exists(dir + "/set_009_A.jpg"));

  // Ids or views that are not one per frame.
  set.frames[1].reset();
  CHECK(vr_example::dump_set(dir, 10, set, {"A", "B"}, nullptr).domain() ==
        Code::InvalidArgument);
  const Views two(2);
  CHECK(vr_example::dump_set(dir, 10, set, ids, &two).domain() ==
        Code::InvalidArgument);
  CHECK(!exists(dir + "/set_010_A.jpg"));
  return 0;
}

// A two-camera set, each frame carrying a JPEG.
sensor::FrameSet two_jpegs() {
  sensor::FrameSet set;
  set.frames.push_back(with_jpeg({0xff, 0xd8, 0x01, 0xff, 0xd9}));
  set.frames.push_back(with_jpeg({0xff, 0xd8, 0x02, 0xff, 0xd9}));
  return set;
}

// The dump over a run, from its check before the cameras open to its report.
int test_run() {
  vr_test::Scratch scratch("vr_set_dump_run_");
  const std::string dir = (scratch.path / "dump").string();
  const std::vector<std::string> ids = {"A", "B"};
  const sensor::FrameSet set = two_jpegs();

  // Off: nothing checked, made or written, and no line.
  vr_example::SetDump off(dir, 0);
  CHECK(off.check().ok());
  CHECK(off.write(set, ids, nullptr).ok() && off.written() == 0);
  CHECK(off.report().empty() && !exists(dir));

  // Checked up front: nothing is made, a trailing separator included.
  vr_example::SetDump dump(dir, 2);
  CHECK(dump.check().ok() && !exists(dir));
  CHECK(vr_example::SetDump(dir + "/", 2).check().ok());
  CHECK(vr_example::SetDump((scratch.path / "no" / "dump").string(), 2)
            .check()
            .domain() == Code::NotFound);
  CHECK(vr_example::SetDump(scratch.path.string(), 2).check().domain() ==
        Code::InvalidArgument);
  CHECK(vr_example::SetDump(scratch.path.string() + "/", 2).check().domain() ==
        Code::InvalidArgument);

  // A first set the dump refuses makes no directory.
  sensor::FrameSet h265 = two_jpegs();
  h265.frames[1] = sensor::RgbdFrame{};
  CHECK(dump.write(h265, ids, nullptr).domain() == Code::InvalidArgument);
  CHECK(!exists(dir) && dump.written() == 0);

  // A first set whose files fail takes its empty directory back: an id with a
  // separator names a directory that is not there.
  CHECK(dump.write(set, {"A/x", "B"}, nullptr).domain() == Code::NotFound);
  CHECK(!exists(dir) && dump.written() == 0);

  // The first set makes the directory; the second set goes beside it, and a
  // third, past the limit, writes nothing.
  CHECK(dump.write(set, ids, nullptr).ok() && dump.written() == 1);
  CHECK(exists(dir + "/set_000_A.jpg") && exists(dir + "/set_000_B.jpg"));
  CHECK(dump.check().domain() == Code::InvalidArgument);  // there now
  CHECK(dump.write(set, ids, nullptr).ok() && dump.written() == 2);
  CHECK(exists(dir + "/set_001_B.jpg"));
  CHECK(dump.write(set, ids, nullptr).ok() && dump.written() == 2);
  CHECK(!exists(dir + "/set_002_A.jpg"));
  CHECK(dump.report() == "dumped 2 sets to " + dir + "\n");

  // A run that ends first: a warning, naming the count it was given.
  const std::string short_dir = (scratch.path / "short").string();
  vr_example::SetDump cut(short_dir, 20);
  CHECK(cut.write(set, ids, nullptr).ok());
  CHECK(cut.report() == "warning: dumped 1 of 20 sets to " + short_dir +
                            ": the run ended first (run.frames counts "
                            "frames, one per camera)\n");
  return 0;
}

}  // namespace

int main() {
  CHECK(test_names() == 0);
  CHECK(test_luma_rgba() == 0);
  CHECK(test_dump() == 0);
  CHECK(test_run() == 0);
  std::puts("example_set_dump: OK");
  return 0;
}
