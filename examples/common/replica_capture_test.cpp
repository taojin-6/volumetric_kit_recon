// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Behavioural test for ReplicaCapture, the one ICameraCapture this tree
// builds and runs. Host-only and dataset-free: it writes a tiny synthetic
// scene in Replica's layout (a 4x3 camera, a handful of frames, a baseline
// JPEG for colour + a 16-bit PNG for depth) into a temporary directory and
// drives the capture through the contract, so it runs on every CI leg --
// including the sanitizer one, which is where a view read after the poll that
// freed it would surface.
//
// What it pins is what the examples depend on and nothing else in the tree
// checks: the on-disk probe (bounded by the limit, stepping by the stride, so
// a thinned sequence plays in full and a trajectory longer than its images
// does not over-promise), frame_count() agreeing with what preload() caches
// and what poll() hands out, exhausted() turning true exactly after the last
// frame, a decode error leaving the position where it was, the pose and
// intrinsics stamped on each frame, the depth-range refusal being named as
// this capture's, and a moved-from capture being empty rather than a shell
// that still claims frames.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "replica_capture.hpp"
#include "volumetric_kit/recon/core/color_space.hpp"
#include "volumetric_kit/recon/sensor/camera_capture.hpp"

namespace vr = volumetric_kit::recon;
namespace sensor = volumetric_kit::recon::sensor;
namespace fs = std::filesystem;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

namespace {

// The synthetic camera. Small, and shaped so the intrinsics are recognisable
// on the frame; the depth scale makes raw 1000 exactly 1 m.
constexpr std::uint32_t kWidth = 4;
constexpr std::uint32_t kHeight = 3;
constexpr float kFx = 2.0f;
constexpr float kFy = 2.5f;
constexpr float kCx = 1.5f;
constexpr float kCy = 1.0f;
constexpr float kDepthScale = 1000.0f;
constexpr std::uint8_t kGrey = 128;  // the flat colour every frame is painted

// The colour image: a 4x3 baseline JPEG of flat grey 128, generated once with
// stb_image_write at quality 100 (its encoder trips UBSan on a signed shift,
// so it is not run here) and decoding to exactly 128 through the pinned
// stb_image. A real JPEG, as Replica's are, not a PNG under a .jpg name.
constexpr std::uint8_t kFrameJpeg[] = {
    0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, 0x4A, 0x46, 0x49, 0x46, 0x00, 0x01,
    0x01, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0xFF, 0xDB, 0x00, 0x84,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0xFF, 0xC0,
    0x00, 0x11, 0x08, 0x00, 0x03, 0x00, 0x04, 0x03, 0x01, 0x11, 0x00, 0x02,
    0x11, 0x01, 0x03, 0x11, 0x01, 0xFF, 0xC4, 0x01, 0xA2, 0x00, 0x00, 0x01,
    0x05, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
    0x0A, 0x0B, 0x10, 0x00, 0x02, 0x01, 0x03, 0x03, 0x02, 0x04, 0x03, 0x05,
    0x05, 0x04, 0x04, 0x00, 0x00, 0x01, 0x7D, 0x01, 0x02, 0x03, 0x00, 0x04,
    0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07, 0x22,
    0x71, 0x14, 0x32, 0x81, 0x91, 0xA1, 0x08, 0x23, 0x42, 0xB1, 0xC1, 0x15,
    0x52, 0xD1, 0xF0, 0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0A, 0x16, 0x17,
    0x18, 0x19, 0x1A, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x34, 0x35, 0x36,
    0x37, 0x38, 0x39, 0x3A, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A,
    0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x63, 0x64, 0x65, 0x66,
    0x67, 0x68, 0x69, 0x6A, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A,
    0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x92, 0x93, 0x94, 0x95,
    0x96, 0x97, 0x98, 0x99, 0x9A, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8,
    0xA9, 0xAA, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xC2,
    0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xD2, 0xD3, 0xD4, 0xD5,
    0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7,
    0xE8, 0xE9, 0xEA, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9,
    0xFA, 0x01, 0x00, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
    0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x11, 0x00, 0x02, 0x01, 0x02, 0x04,
    0x04, 0x03, 0x04, 0x07, 0x05, 0x04, 0x04, 0x00, 0x01, 0x02, 0x77, 0x00,
    0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51,
    0x07, 0x61, 0x71, 0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xA1,
    0xB1, 0xC1, 0x09, 0x23, 0x33, 0x52, 0xF0, 0x15, 0x62, 0x72, 0xD1, 0x0A,
    0x16, 0x24, 0x34, 0xE1, 0x25, 0xF1, 0x17, 0x18, 0x19, 0x1A, 0x26, 0x27,
    0x28, 0x29, 0x2A, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x43, 0x44, 0x45,
    0x46, 0x47, 0x48, 0x49, 0x4A, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
    0x5A, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x73, 0x74, 0x75,
    0x76, 0x77, 0x78, 0x79, 0x7A, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88,
    0x89, 0x8A, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0xA2,
    0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xB2, 0xB3, 0xB4, 0xB5,
    0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8,
    0xC9, 0xCA, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xE2,
    0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xF2, 0xF3, 0xF4, 0xF5,
    0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0xFF, 0xDA, 0x00, 0x0C, 0x03, 0x01, 0x00,
    0x02, 0x11, 0x03, 0x11, 0x00, 0x3F, 0x00, 0x28, 0x03, 0xFF, 0xD9,
};

// Raw depth of frame `index`: 1.0 m, 1.1 m, ... -- inside the default range,
// distinct per frame, exact in a 16-bit PNG.
std::uint16_t raw_depth(std::size_t index) {
  return static_cast<std::uint16_t>(1000 + 100 * index);
}
float metres(std::size_t index) {
  return static_cast<float>(raw_depth(index)) / kDepthScale;
}

// --- A minimal 16-bit grayscale PNG encoder -------------------------------
// The depth loader insists on a genuine 16-bit file (an 8-bit one would decode
// ~257x too large), which nothing in the tree writes. The image is tiny, so
// the zlib stream is one stored (uncompressed) deflate block.

std::uint32_t crc32(const std::vector<std::uint8_t>& bytes, std::size_t from) {
  std::uint32_t crc = 0xFFFFFFFFu;
  for (std::size_t i = from; i < bytes.size(); ++i) {
    crc ^= bytes[i];
    for (int k = 0; k < 8; ++k) {
      crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
  }
  return ~crc;
}

void put_be32(std::vector<std::uint8_t>& out, std::uint32_t v) {
  out.push_back(static_cast<std::uint8_t>(v >> 24));
  out.push_back(static_cast<std::uint8_t>(v >> 16));
  out.push_back(static_cast<std::uint8_t>(v >> 8));
  out.push_back(static_cast<std::uint8_t>(v));
}

// Append one chunk: length, type, data, CRC over type + data.
void put_chunk(std::vector<std::uint8_t>& out, const char* type,
               const std::vector<std::uint8_t>& data) {
  put_be32(out, static_cast<std::uint32_t>(data.size()));
  const std::size_t crc_from = out.size();
  for (int i = 0; i < 4; ++i) out.push_back(static_cast<std::uint8_t>(type[i]));
  out.insert(out.end(), data.begin(), data.end());
  put_be32(out, crc32(out, crc_from));
}

bool write_depth_png(const fs::path& path, std::uint16_t raw) {
  std::vector<std::uint8_t> png = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
  std::vector<std::uint8_t> ihdr;
  put_be32(ihdr, kWidth);
  put_be32(ihdr, kHeight);
  ihdr.push_back(16);  // bit depth
  ihdr.push_back(0);   // colour type: greyscale
  ihdr.push_back(0);   // compression
  ihdr.push_back(0);   // filter
  ihdr.push_back(0);   // interlace
  put_chunk(png, "IHDR", ihdr);

  // Scanlines: filter byte 0, then big-endian 16-bit samples.
  std::vector<std::uint8_t> scan;
  for (std::uint32_t y = 0; y < kHeight; ++y) {
    scan.push_back(0);
    for (std::uint32_t x = 0; x < kWidth; ++x) {
      scan.push_back(static_cast<std::uint8_t>(raw >> 8));
      scan.push_back(static_cast<std::uint8_t>(raw));
    }
  }
  // zlib: header, one final stored block (LEN, NLEN little-endian), adler32.
  std::vector<std::uint8_t> idat = {0x78, 0x01, 0x01};
  const auto len = static_cast<std::uint16_t>(scan.size());
  idat.push_back(static_cast<std::uint8_t>(len));
  idat.push_back(static_cast<std::uint8_t>(len >> 8));
  idat.push_back(static_cast<std::uint8_t>(~len));
  idat.push_back(
      static_cast<std::uint8_t>(static_cast<std::uint16_t>(~len) >> 8));
  idat.insert(idat.end(), scan.begin(), scan.end());
  std::uint32_t a = 1, b = 0;
  for (const std::uint8_t byte : scan) {
    a = (a + byte) % 65521u;
    b = (b + a) % 65521u;
  }
  put_be32(idat, (b << 16) | a);
  put_chunk(png, "IDAT", idat);
  put_chunk(png, "IEND", {});

  std::ofstream out(path, std::ios::binary);
  out.write(reinterpret_cast<const char*>(png.data()),
            static_cast<std::streamsize>(png.size()));
  return static_cast<bool>(out);
}

bool write_color_jpg(const fs::path& path) {
  std::ofstream out(path, std::ios::binary);
  out.write(reinterpret_cast<const char*>(kFrameJpeg), sizeof(kFrameJpeg));
  return static_cast<bool>(out);
}

std::string frame_name(const char* prefix, std::size_t index,
                       const char* suffix) {
  char name[64];
  std::snprintf(name, sizeof(name), "%s%06zu%s", prefix, index, suffix);
  return name;
}

// A scene: `poses` trajectory lines (identity rotation, translation
// (index, 0, 0) so a frame's pose names its index), and images for exactly
// the indices in `on_disk`.
bool write_scene(const fs::path& scene, std::size_t poses,
                 const std::vector<std::size_t>& on_disk) {
  std::error_code ec;
  fs::create_directories(scene / "results", ec);
  if (ec) return false;
  std::ofstream traj(scene / "traj.txt");
  for (std::size_t i = 0; i < poses; ++i) {
    // Row-major 4x4, translation in the last column of the first row.
    traj << "1 0 0 " << i << " 0 1 0 0 0 0 1 0 0 0 0 1\n";
  }
  if (!traj) return false;
  for (const std::size_t index : on_disk) {
    if (!write_color_jpg(scene / "results" /
                         frame_name("frame", index, ".jpg")))
      return false;
    if (!write_depth_png(scene / "results" / frame_name("depth", index, ".png"),
                         raw_depth(index)))
      return false;
  }
  return true;
}

bool write_cam_params(const fs::path& path) {
  std::ofstream out(path);
  out << "{\"camera\": {\"w\": " << kWidth << ", \"h\": " << kHeight
      << ", \"fx\": " << kFx << ", \"fy\": " << kFy << ", \"cx\": " << kCx
      << ", \"cy\": " << kCy << ", \"scale\": " << kDepthScale << "}}\n";
  return static_cast<bool>(out);
}

bool close(float a, float b) { return std::abs(a - b) <= 1e-6f; }

// The frame `index` looks like when the capture hands it out: pose, pixels,
// cameras, the declarations.
bool frame_is(const sensor::CapturedFrame& frame, std::size_t index) {
  if (frame.depth == nullptr || !frame.has_color()) return false;
  if (frame.depth_camera.width != kWidth ||
      frame.depth_camera.height != kHeight)
    return false;
  if (frame.color_camera.width != kWidth ||
      frame.color_camera.height != kHeight)
    return false;
  // Both poses from the one trajectory entry, translated by the index.
  if (!close(frame.depth_camera.cam_to_world[3].x, static_cast<float>(index)))
    return false;
  if (!close(frame.color_camera.cam_to_world[3].x, static_cast<float>(index)))
    return false;
  for (std::size_t p = 0; p < static_cast<std::size_t>(kWidth) * kHeight; ++p) {
    if (!close(frame.depth[p], metres(index))) return false;
    // The high byte is the loader's 0, and the three channels are the one
    // grey (exact for this blob through the pinned decoder; a code of slack
    // so a decoder update's rounding cannot fail the capture's test).
    const std::uint32_t c = frame.color[p];
    const int r = static_cast<int>(c & 0xFFu);
    const int g = static_cast<int>((c >> 8) & 0xFFu);
    const int b = static_cast<int>((c >> 16) & 0xFFu);
    if ((c >> 24) != 0 || std::abs(r - kGrey) > 1 || std::abs(g - kGrey) > 1 ||
        std::abs(b - kGrey) > 1)
      return false;
  }
  return vr::is_canonical(frame.color_encoding) && frame.timestamp_ns == 0;
}

// Drive `capture` to exhaustion, checking that the frames come out in the
// order `expected` lists and that exhausted() turns true exactly after the
// last one. Returns the number handed out, or -1 on a mismatch.
int play(sensor::ICameraCapture& capture,
         const std::vector<std::size_t>& expected) {
  int handed = 0;
  for (const std::size_t index : expected) {
    if (capture.exhausted()) return -1;
    vr::Result<std::optional<sensor::CapturedFrame>> polled = capture.poll();
    if (!polled.ok() || !polled.value().has_value()) return -1;
    if (!frame_is(*polled.value(), index)) return -1;
    ++handed;
  }
  if (!capture.exhausted()) return -1;
  vr::Result<std::optional<sensor::CapturedFrame>> after = capture.poll();
  if (!after.ok() || after.value().has_value()) return -1;
  return handed;
}

vr_example::ReplicaCapture::Options options(std::size_t limit,
                                            std::size_t stride) {
  vr_example::ReplicaCapture::Options o;
  o.frame_limit = limit;
  o.frame_stride = stride;
  return o;
}

}  // namespace

int main() {
  // A fresh directory per run, so two runs cannot read each other's scene.
  const fs::path root =
      fs::temp_directory_path() /
      ("vr_replica_capture_test_" +
       std::to_string(
           std::chrono::steady_clock::now().time_since_epoch().count()));
  struct Cleanup {
    fs::path path;
    ~Cleanup() {
      std::error_code ec;
      fs::remove_all(path, ec);
    }
  } cleanup{root};

  {
    std::error_code ec;
    fs::create_directories(root, ec);
    CHECK(!ec);
  }
  const fs::path cam_params = root / "cam_params.json";
  CHECK(write_cam_params(cam_params));
  // Scene A: eight poses, images for the first five -- a trajectory longer
  // than its images, which is what room0 is (2000 against 400).
  const fs::path scene_a = root / "a";
  CHECK(write_scene(scene_a, 8, {0, 1, 2, 3, 4}));
  // Scene B: the same, thinned on disk to every other frame.
  const fs::path scene_b = root / "b";
  CHECK(write_scene(scene_b, 8, {0, 2, 4, 6}));

  const std::size_t no_limit =
      vr_example::ReplicaCapture::Options{}.frame_limit;

  // --- open: the probe counts the frames that will play ---------------------
  {
    auto r = vr_example::ReplicaCapture::open(
        scene_a.string(), cam_params.string(), options(no_limit, 1));
    CHECK(r.ok());
    vr_example::ReplicaCapture capture = std::move(r).value();
    CHECK(capture.frame_count() == 5);  // the images, not the poses
    CHECK(capture.preload_bytes_projected() == 5 * kWidth * kHeight * 8);
    CHECK(close(capture.color_camera().fx, kFx));
    CHECK(close(capture.color_camera().fy, kFy));
    CHECK(close(capture.color_camera().cx, kCx));
    CHECK(close(capture.color_camera().cy, kCy));
    CHECK(capture.color_camera().width == kWidth);
    CHECK(capture.color_camera().height == kHeight);
    CHECK(close(capture.depth_scale(), kDepthScale));

    // Not started: no frame, no error, and not exhausted -- it has not begun.
    sensor::ICameraCapture& device = capture;
    CHECK(!device.exhausted());
    CHECK(device.poll().ok() && !device.poll().value().has_value());

    CHECK(device.start().ok());
    CHECK(play(device, {0, 1, 2, 3, 4}) == 5);

    // stop() rewinds; start() plays from the first frame again.
    device.stop();
    CHECK(!device.exhausted());
    CHECK(device.start().ok());
    CHECK(play(device, {0, 1, 2, 3, 4}) == 5);
  }

  // --- The limit bounds the probe and the playback ---------------------------
  {
    auto r = vr_example::ReplicaCapture::open(
        scene_a.string(), cam_params.string(), options(2, 1));
    CHECK(r.ok());
    CHECK(r.value().frame_count() == 2);
    CHECK(r.value().start().ok());
    CHECK(play(r.value(), {0, 1}) == 2);
  }
  {
    // A limit of zero plays nothing and is exhausted from the start.
    auto r = vr_example::ReplicaCapture::open(
        scene_a.string(), cam_params.string(), options(0, 1));
    CHECK(r.ok());
    CHECK(r.value().frame_count() == 0);
    CHECK(r.value().start().ok());
    CHECK(r.value().exhausted());
    CHECK(play(r.value(), {}) == 0);
  }

  // --- The stride: frame_count, preload and poll agree on the same set ------
  {
    auto r = vr_example::ReplicaCapture::open(
        scene_a.string(), cam_params.string(), options(no_limit, 2));
    CHECK(r.ok());
    vr_example::ReplicaCapture capture = std::move(r).value();
    CHECK(capture.frame_count() == 3);  // 0, 2, 4
    vr::Result<std::size_t> cached = capture.preload();
    CHECK(cached.ok() && cached.value() == 3);
    CHECK(capture.preloaded_bytes() == 3 * kWidth * kHeight * 8);
    CHECK(capture.start().ok());
    CHECK(play(capture, {0, 2, 4}) == 3);  // served from the cache
    // A stride past the end plays the first frame only.
    auto one = vr_example::ReplicaCapture::open(
        scene_a.string(), cam_params.string(), options(no_limit, 100));
    CHECK(one.ok() && one.value().frame_count() == 1);
    CHECK(one.value().start().ok());
    CHECK(play(one.value(), {0}) == 1);
  }

  // --- A sequence thinned on disk plays in full under the matching stride ---
  // The probe visits only the strided indices, so the absent odd frames never
  // end the sequence; at stride 1 the gap at index 1 does.
  {
    auto thinned = vr_example::ReplicaCapture::open(
        scene_b.string(), cam_params.string(), options(no_limit, 2));
    CHECK(thinned.ok());
    CHECK(thinned.value().frame_count() == 4);  // 0, 2, 4, 6
    CHECK(thinned.value().start().ok());
    CHECK(play(thinned.value(), {0, 2, 4, 6}) == 4);

    auto dense = vr_example::ReplicaCapture::open(
        scene_b.string(), cam_params.string(), options(no_limit, 1));
    CHECK(dense.ok());
    CHECK(dense.value().frame_count() == 1);
    CHECK(dense.value().start().ok());
    CHECK(play(dense.value(), {0}) == 1);
  }

  // --- Refusals at open, named as this capture's -----------------------------
  {
    vr_example::ReplicaCapture::Options bad_range = options(no_limit, 1);
    bad_range.min_depth = 5.0f;
    bad_range.max_depth = 1.0f;
    auto r = vr_example::ReplicaCapture::open(scene_a.string(),
                                              cam_params.string(), bad_range);
    CHECK(!r.ok());
    CHECK(r.status().message().rfind("ReplicaCapture::open", 0) == 0);
    CHECK(r.status().message().find("max_depth") != std::string::npos);

    CHECK(!vr_example::ReplicaCapture::open(
               scene_a.string(), cam_params.string(), options(no_limit, 0))
               .ok());
    CHECK(!vr_example::ReplicaCapture::open((root / "missing").string(),
                                            cam_params.string(),
                                            options(no_limit, 1))
               .ok());
    CHECK(!vr_example::ReplicaCapture::open(scene_a.string(),
                                            (root / "missing.json").string(),
                                            options(no_limit, 1))
               .ok());
  }

  // --- A moved-from capture is empty, not a shell that still claims frames --
  {
    auto r = vr_example::ReplicaCapture::open(
        scene_a.string(), cam_params.string(), options(no_limit, 1));
    CHECK(r.ok());
    vr_example::ReplicaCapture source = std::move(r).value();
    CHECK(source.start().ok());
    CHECK(source.poll().ok() && source.poll().ok());  // frames 0 and 1 gone

    vr_example::ReplicaCapture moved(std::move(source));
    // The source: nothing to play, not running, exhausted -- and pollable
    // without reaching into an emptied pose table.
    CHECK(source.frame_count() == 0);
    CHECK(source.exhausted());
    CHECK(source.poll().ok() && !source.poll().value().has_value());
    CHECK(source.preload().ok() && source.preload().value() == 0);
    CHECK(source.preloaded_bytes() == 0);
    // The destination resumes where the source was.
    CHECK(moved.frame_count() == 5);
    CHECK(play(moved, {2, 3, 4}) == 3);

    // Move-assign over a live capture: the old state goes, the new resumes.
    auto other_r = vr_example::ReplicaCapture::open(
        scene_b.string(), cam_params.string(), options(no_limit, 2));
    CHECK(other_r.ok());
    vr_example::ReplicaCapture other = std::move(other_r).value();
    CHECK(other.start().ok());
    CHECK(other.poll().ok());  // frame 0 of scene B
    moved.stop();
    CHECK(moved.start().ok());
    CHECK(moved.poll().ok());  // frame 0 of scene A again
    other = std::move(moved);
    CHECK(moved.frame_count() == 0 && moved.exhausted());
    CHECK(other.frame_count() == 5);
    CHECK(play(other, {1, 2, 3, 4}) == 4);

    // Self-move leaves the capture as it was. Laundered through a pointer so
    // -Wself-move under -Werror does not reject the test that proves it.
    vr_example::ReplicaCapture* self = &other;
    other = std::move(*self);
    CHECK(other.frame_count() == 5);
    CHECK(other.exhausted());
  }

  // --- A decode error leaves the position unchanged --------------------------
  // Corrupt frame 2 of scene A: frames 0 and 1 play, the third poll reports
  // the error, and so does the fourth -- the position did not move past the
  // frame that failed -- while exhausted() stays false. preload() reports the
  // same error rather than caching a partial sequence.
  {
    {
      std::ofstream bad(scene_a / "results" / frame_name("frame", 2, ".jpg"),
                        std::ios::binary | std::ios::trunc);
      bad << "not a jpeg";
    }
    auto r = vr_example::ReplicaCapture::open(
        scene_a.string(), cam_params.string(), options(no_limit, 1));
    CHECK(r.ok());
    vr_example::ReplicaCapture capture = std::move(r).value();
    CHECK(capture.frame_count() == 5);  // on disk, so still counted
    CHECK(!capture.preload().ok());
    CHECK(capture.preloaded_bytes() == 0);
    CHECK(capture.start().ok());
    for (const std::size_t index : {std::size_t{0}, std::size_t{1}}) {
      vr::Result<std::optional<sensor::CapturedFrame>> polled = capture.poll();
      CHECK(polled.ok() && polled.value().has_value());
      CHECK(frame_is(*polled.value(), index));
    }
    CHECK(!capture.poll().ok());
    CHECK(!capture.exhausted());
    CHECK(!capture.poll().ok());
  }

  std::printf("replica_capture_test: OK\n");
  return 0;
}
