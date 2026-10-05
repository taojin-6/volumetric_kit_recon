// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The GPU pass against the SDK's host path, on a real camera and a still
// scene: one capture through the host path (the SDK undistorts colour and
// registers depth to it), and raw through GpuFramePrep over each codec, H.265
// and MJPEG, the colour decoded onto the pass's device where the hardware
// leaves it there; and raw H.265 once more onto a device the decoder can keep
// nothing on, so every frame's colour falls back to the host and is counted.
//   - Colour: the two undistorted images, gains fitted per channel for the
//     exposure change between captures, line up best unshifted in the centre
//     and in every corner, where a wrong lens model moves them apart.
//   - Depth: the pass's undistorted depth, moved into the colour camera by
//     the factory extrinsic, lands where the SDK registered it: best
//     unshifted, and close in depth. A reversed extrinsic would shift it by
//     about f * baseline / z.
//
// Runs only when VR_ORBBEC_TEST_SERIAL names a camera (a primary or one
// running free); VR_ORBBEC_TEST_COLOR=WxH and VR_ORBBEC_TEST_FPS pick the
// colour mode (1280x720 at 30 by default; 4K H.265 runs at 25 at most). Keep
// the camera and the scene still.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "bare_device.hpp"
#include "buffer_readback.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/instance.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_capture.hpp"
#include "volumetric_kit/recon/sensor/utils/gpu_frame_prep.hpp"

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;
namespace sensor = volumetric_kit::recon::sensor;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

namespace {

constexpr int kSettleFrames = 45;  // auto exposure settles; the last is kept
constexpr auto kTimeout = std::chrono::seconds(30);

// One frame copied out of a capture: depth on its camera's grid, colour on
// its.
struct Frame {
  std::vector<float> depth;
  std::vector<std::uint32_t> color;
  vr::DepthCameraParams dcam{};
  vr::ColorCameraParams ccam{};
};

std::uint32_t g_fps = 30;

sensor::OrbbecCapture::Options options_for(
    const char* serial, std::uint32_t w, std::uint32_t h, bool raw,
    const vkc::Device* device = nullptr, vkc::Allocator* allocator = nullptr,
    sensor::OrbbecColorCodec codec = sensor::OrbbecColorCodec::Hevc) {
  sensor::OrbbecCapture::Options o;
  o.serial = serial;
  o.fps = g_fps;
  o.color_width = w;
  o.color_height = h;
  o.color_codec = codec;
  o.raw = raw;
  o.device = device;
  o.allocator = allocator;
  return o;
}

// Poll until `kSettleFrames` frames came, handing each to `keep`.
template <typename Poll, typename Keep>
int settle(Poll&& poll, Keep&& keep) {
  const auto deadline = std::chrono::steady_clock::now() + kTimeout;
  int seen = 0;
  while (seen < kSettleFrames) {
    if (std::chrono::steady_clock::now() > deadline) {
      std::fprintf(stderr, "FAIL: %d of %d frames before the timeout\n", seen,
                   kSettleFrames);
      return 1;
    }
    auto polled = poll();
    if (!polled) {
      std::fprintf(stderr, "FAIL: poll: %s\n",
                   polled.status().message().c_str());
      return 1;
    }
    if (!polled.value()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      continue;
    }
    ++seen;
    if (seen == kSettleFrames && keep(*polled.value()) != 0) return 1;
  }
  return 0;
}

int grab_host(const char* serial, std::uint32_t w, std::uint32_t h,
              Frame* out) {
  auto opened = sensor::OrbbecCapture::open(options_for(serial, w, h, false));
  if (!opened) {
    std::fprintf(stderr, "FAIL: open: %s\n", opened.status().message().c_str());
    return 1;
  }
  sensor::OrbbecCapture capture = std::move(opened).value();
  CHECK(capture.start().ok());
  return settle([&] { return capture.poll(); },
                [&](const sensor::CapturedFrame& f) {
                  const std::size_t dn =
                      std::size_t{f.depth_camera.width} * f.depth_camera.height;
                  const std::size_t cn =
                      std::size_t{f.color_camera.width} * f.color_camera.height;
                  out->depth.assign(f.depth, f.depth + dn);
                  out->color.assign(f.color, f.color + cn);
                  out->dcam = f.depth_camera;
                  out->ccam = f.color_camera;
                  return 0;
                });
}

// Raw frames decoded onto `decode_on`, prepared on `device`. A `decode_on`
// other than `device` is one the decoder keeps nothing on.
int grab_gpu(const char* serial, std::uint32_t w, std::uint32_t h,
             sensor::OrbbecColorCodec codec, vkc::Device& device,
             const vkc::Device& decode_on, vkc::Allocator& allocator,
             sensor::GpuFramePrep& prep, Frame* out) {
  const bool forced = &decode_on != &device;
  std::printf("raw %s%s:\n", sensor::to_string(codec),
              forced ? ", the device path forced to fail" : "");
  auto opened = sensor::OrbbecCapture::open(
      options_for(serial, w, h, true, &decode_on, &allocator, codec));
  if (!opened) {
    std::fprintf(stderr, "FAIL: open raw: %s\n",
                 opened.status().message().c_str());
    return 1;
  }
  sensor::OrbbecCapture capture = std::move(opened).value();
  CHECK(capture.start().ok());
  std::uint64_t frames = 0;
  std::uint64_t host_frames = 0;  // colour handed out on the host
  const int settled = settle(
      [&] {
        // Every frame handed out, as the stats count them, not only the one
        // `settle` keeps.
        auto polled = capture.poll_raw();
        if (polled && polled.value()) {
          ++frames;
          const sensor::YuvImage& c = polled.value()->color;
          host_frames += c.device == nullptr && c.image[0] == nullptr ? 1 : 0;
        }
        return polled;
      },
      [&](const sensor::RawFrame& raw) {
        const auto& d = raw.depth_camera;
        const auto& c = raw.color_camera;
        std::printf(
            "  depth lens %ux%u f %.2f %.2f c %.2f %.2f k %.4f %.4f %.4f "
            "%.4f %.4f %.4f p %.5f %.5f\n",
            d.width, d.height, d.fx, d.fy, d.cx, d.cy, d.lens.k1, d.lens.k2,
            d.lens.k3, d.lens.k4, d.lens.k5, d.lens.k6, d.lens.p1, d.lens.p2);
        std::printf(
            "  colour lens %ux%u f %.2f %.2f c %.2f %.2f k %.4f %.4f %.4f "
            "%.4f %.4f %.4f p %.5f %.5f\n",
            c.width, c.height, c.fx, c.fy, c.cx, c.cy, c.lens.k1, c.lens.k2,
            c.lens.k3, c.lens.k4, c.lens.k5, c.lens.k6, c.lens.p1, c.lens.p2);
        const vr::Vec4f t = raw.depth_cam_to_world[3];
        std::printf("  depth camera at (%.4f, %.4f, %.4f) m in the colour's\n",
                    t.x, t.y, t.z);
        // Where the hardware decoded it, the colour stays on the device; a
        // leg whose hardware leaves it there must find it there.
        const bool on_device =
            raw.color.device != nullptr || raw.color.image[0] != nullptr;
        std::printf("  colour %s\n",
                    raw.color.image[0] != nullptr ? "as images on the device"
                    : raw.color.device != nullptr ? "in a buffer on the device"
                                                  : "as host planes");
        const char* required = std::getenv("VR_TEST_HEVC_BACKEND");
        const std::string backend = required != nullptr ? required : "";
        if ((backend == "videotoolbox" ||
             (VR_TEST_WITH_CUDA && backend == "cuda")) &&
            !on_device && !forced) {
          std::fprintf(stderr, "FAIL: %s promised, colour on the host\n",
                       required);
          return 1;
        }
        auto prepared = prep.prepare(raw);
        if (!prepared) {
          std::fprintf(stderr, "FAIL: prepare: %s\n",
                       prepared.status().message().c_str());
          return 1;
        }
        const sensor::DeviceFrame& f = prepared.value();
        const std::size_t dn =
            std::size_t{f.depth_camera.width} * f.depth_camera.height;
        const std::size_t cn =
            std::size_t{f.color_camera.width} * f.color_camera.height;
        // Device-local: copied out rather than mapped.
        auto depth = vr_test::read_back<float>(device, allocator, *f.depth, dn);
        auto color =
            vr_test::read_back<std::uint32_t>(device, allocator, *f.color, cn);
        if (!depth || !color) {
          std::fprintf(
              stderr, "FAIL: read back: %s\n",
              (!depth ? depth.status() : color.status()).message().c_str());
          return 1;
        }
        out->depth = std::move(depth).value();
        out->color = std::move(color).value();
        out->dcam = f.depth_camera;
        out->ccam = f.color_camera;
        return 0;
      });
  if (settled != 0) return settled;
  // The stats count what the frames showed: each one whose colour came to
  // the host although the stream decodes onto a device, which is every one
  // when the device path cannot open.
  const std::uint64_t counted = capture.stats().host_pictures;
  std::printf("  %llu of %llu frames with colour on the host\n",
              static_cast<unsigned long long>(counted),
              static_cast<unsigned long long>(frames));
  if (counted != host_frames || (forced && host_frames != frames)) {
    std::fprintf(stderr,
                 "FAIL: stats count %llu host pictures; %llu of %llu frames "
                 "came to the host\n",
                 static_cast<unsigned long long>(counted),
                 static_cast<unsigned long long>(host_frames),
                 static_cast<unsigned long long>(frames));
    return 1;
  }
  return 0;
}

float channel(std::uint32_t c, int k) {
  return static_cast<float>((c >> (8 * k)) & 0xFF);
}

// Per-channel least-squares gain and offset taking `b` to `a` over pixels
// both images cover.
void fit_gains(const Frame& a, const Frame& b, float gain[3], float off[3]) {
  for (int k = 0; k < 3; ++k) {
    double sa = 0, sb = 0, sbb = 0, sab = 0, n = 0;
    for (std::size_t i = 0; i < a.color.size(); ++i) {
      if (a.color[i] == 0 || b.color[i] == 0) continue;
      const double x = channel(b.color[i], k), y = channel(a.color[i], k);
      sa += y;
      sb += x;
      sbb += x * x;
      sab += x * y;
      n += 1;
    }
    const double g = (n * sab - sa * sb) / (n * sbb - sb * sb);
    gain[k] = static_cast<float>(g);
    off[k] = static_cast<float>((sa - g * sb) / n);
  }
}

// RMS difference of `a` and `b` shifted by (dx, dy), gains applied, over the
// box [x0, x1) x [y0, y1).
double color_rms(const Frame& a, const Frame& b, const float gain[3],
                 const float off[3], int dx, int dy, int x0, int y0, int x1,
                 int y1) {
  const int w = static_cast<int>(a.ccam.width);
  double sum = 0, n = 0;
  for (int y = y0; y < y1; ++y) {
    for (int x = x0; x < x1; ++x) {
      const std::uint32_t ca = a.color[static_cast<std::size_t>(y * w + x)];
      const std::uint32_t cb =
          b.color[static_cast<std::size_t>((y + dy) * w + x + dx)];
      if (ca == 0 || cb == 0) continue;
      for (int k = 0; k < 3; ++k) {
        const double e = channel(ca, k) - (gain[k] * channel(cb, k) + off[k]);
        sum += e * e;
        n += 1;
      }
    }
  }
  return n > 0 ? std::sqrt(sum / n) : std::numeric_limits<double>::infinity();
}

// The shift of `b` in [-r, r]^2 that best matches `a` over a box.
void best_shift(const Frame& a, const Frame& b, const float gain[3],
                const float off[3], int x0, int y0, int x1, int y1, int r,
                int* bx, int* by, double* best) {
  *best = std::numeric_limits<double>::infinity();
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      const double e = color_rms(a, b, gain, off, dx, dy, x0, y0, x1, y1);
      if (e < *best) {
        *best = e;
        *bx = dx;
        *by = dy;
      }
    }
  }
}

int check_color(const Frame& host, const Frame& gpu) {
  CHECK(host.ccam.width == gpu.ccam.width &&
        host.ccam.height == gpu.ccam.height);
  float gain[3], off[3];
  fit_gains(host, gpu, gain, off);
  std::printf("  colour gains %.3f %.3f %.3f, offsets %.1f %.1f %.1f\n",
              gain[0], gain[1], gain[2], off[0], off[1], off[2]);
  const int w = static_cast<int>(host.ccam.width);
  const int h = static_cast<int>(host.ccam.height);
  // A 5 x 3 grid of boxes, corners included, where the lens moves pixels
  // most. A box whose best match is still poor saw the scene change between
  // the captures (a screen, a person) and says nothing about geometry.
  const int side = std::min(w, h) / 5;
  const int margin = 8;  // past the shift range
  int comparable = 0, edge_boxes = 0;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 5; ++col) {
      const int x0 = margin + col * (w - side - 2 * margin) / 4;
      const int y0 = margin + row * (h - side - 2 * margin) / 2;
      int bx = 0, by = 0;
      double best = 0;
      best_shift(host, gpu, gain, off, x0, y0, x0 + side, y0 + side, 3, &bx,
                 &by, &best);
      const double here =
          color_rms(host, gpu, gain, off, 0, 0, x0, y0, x0 + side, y0 + side);
      // Texture is what a shift can be read off: a flat box matches about as
      // well at any shift.
      const double moved = std::min(
          color_rms(host, gpu, gain, off, 2, 0, x0, y0, x0 + side, y0 + side),
          color_rms(host, gpu, gain, off, 0, 2, x0, y0, x0 + side, y0 + side));
      const bool changed = best > 12.0;
      const bool flat = moved < 1.5 * best;
      std::printf(
          "  colour box (%d, %d): rms %.2f unshifted, best %.2f at "
          "(%d, %d), %.2f at 2 px%s\n",
          col, row, here, best, bx, by, moved,
          changed ? " -- the scene changed here; skipped"
          : flat  ? " -- too flat to align; skipped"
                  : "");
      if (changed || flat) continue;
      ++comparable;
      if (row != 1 || col == 0 || col == 4) ++edge_boxes;
      CHECK(bx == 0 && by == 0);
    }
  }
  CHECK(comparable >= 8);
  CHECK(edge_boxes >= 5);  // the lens is tested where it bends most
  return 0;
}

// The pass's depth moved into the colour camera, the nearest kept, on the
// colour grid.
std::vector<float> reproject(const Frame& gpu) {
  const vr::DepthCameraParams& d = gpu.dcam;
  const vr::ColorCameraParams& c = gpu.ccam;
  const vr::Mat4f color_from_depth =
      glm::inverse(c.cam_to_world) * d.cam_to_world;
  std::vector<float> out(std::size_t{c.width} * c.height, 0.0f);
  for (std::uint32_t v = 0; v < d.height; ++v) {
    for (std::uint32_t u = 0; u < d.width; ++u) {
      const float z = gpu.depth[std::size_t{v} * d.width + u];
      if (!(z > 0.0f)) continue;
      const vr::Vec4f p =
          color_from_depth *
          vr::Vec4f((u - d.cx) / d.fx * z, (v - d.cy) / d.fy * z, z, 1.0f);
      if (!(p.z > 0.0f)) continue;
      const long uc = std::lround(p.x / p.z * c.fx + c.cx);
      const long vc = std::lround(p.y / p.z * c.fy + c.cy);
      if (uc < 0 || vc < 0 || uc >= static_cast<long>(c.width) ||
          vc >= static_cast<long>(c.height)) {
        continue;
      }
      float& slot = out[static_cast<std::size_t>(vc) * c.width +
                        static_cast<std::size_t>(uc)];
      if (slot == 0.0f || p.z < slot) slot = p.z;
    }
  }
  return out;
}

int check_depth(const Frame& host, const Frame& gpu) {
  const std::vector<float> moved = reproject(gpu);
  const int w = static_cast<int>(host.ccam.width);
  const int h = static_cast<int>(host.ccam.height);
  CHECK(host.dcam.width == host.ccam.width);  // registered at full size
  const auto median_error = [&](int dx) {
    std::vector<float> e;
    for (int y = 0; y < h; ++y) {
      for (int x = 40; x < w - 40; ++x) {
        const float a = host.depth[static_cast<std::size_t>(y * w + x)];
        const float b = moved[static_cast<std::size_t>(y * w + x + dx)];
        if (a > 0.0f && b > 0.0f) e.push_back(std::fabs(a - b));
      }
    }
    if (e.size() < 1000) return std::numeric_limits<float>::infinity();
    std::nth_element(e.begin(), e.begin() + e.size() / 2, e.end());
    return e[e.size() / 2];
  };
  int best_dx = 0;
  float best = std::numeric_limits<float>::infinity();
  for (int dx = -40; dx <= 40; ++dx) {
    const float e = median_error(dx);
    if (e < best) {
      best = e;
      best_dx = dx;
    }
  }
  const float here = median_error(0);
  std::printf("  depth median error %.2f mm unshifted, best %.2f mm at dx %d\n",
              here * 1000.0f, best * 1000.0f, best_dx);
  CHECK(std::abs(best_dx) <= 1);
  CHECK(here < 0.01f);
  return 0;
}

}  // namespace

int main() {
  const char* serial = std::getenv("VR_ORBBEC_TEST_SERIAL");
  if (serial == nullptr || *serial == '\0') {
    std::puts("sensor_orbbec_gpu_prep: VR_ORBBEC_TEST_SERIAL unset; skipping");
    return 0;
  }
  std::uint32_t w = 1280, h = 720;
  if (const char* size = std::getenv("VR_ORBBEC_TEST_COLOR")) {
    unsigned cw = 0, ch = 0;
    if (std::sscanf(size, "%ux%u", &cw, &ch) == 2) {
      w = cw;
      h = ch;
    }
  }
  if (const char* fps = std::getenv("VR_ORBBEC_TEST_FPS")) {
    if (std::atoi(fps) > 0) g_fps = static_cast<std::uint32_t>(std::atoi(fps));
  }
  auto instance = vkc::Instance::create({});
  CHECK(instance.ok());
  auto gpu = instance->select_physical_device(vr::device_requirements());
  CHECK(gpu.ok());
  auto device = vkc::Device::create(instance.value(), gpu.value(),
                                    vr::device_requirements());
  CHECK(device.ok());
  auto allocator = vkc::Allocator::create(instance->handle(), device.value());
  CHECK(allocator.ok());
  auto prep = sensor::GpuFramePrep::create(device.value(), allocator.value());
  CHECK(prep.ok());

  Frame host;
  if (grab_host(serial, w, h, &host) != 0) return 1;
  for (const auto codec :
       {sensor::OrbbecColorCodec::Hevc, sensor::OrbbecColorCodec::Mjpeg}) {
    Frame raw;
    if (grab_gpu(serial, w, h, codec, device.value(), device.value(),
                 allocator.value(), prep.value(), &raw) != 0) {
      return 1;
    }
    if (check_color(host, raw) != 0) return 1;
    if (check_depth(host, raw) != 0) return 1;
  }
  auto bare = vr_test::bare_device(instance.value(), device.value());
  CHECK(bare.ok());
  Frame fallback;
  if (grab_gpu(serial, w, h, sensor::OrbbecColorCodec::Hevc, device.value(),
               bare.value(), allocator.value(), prep.value(), &fallback) != 0) {
    return 1;
  }
  if (check_color(host, fallback) != 0) return 1;
  std::puts("sensor_orbbec_gpu_prep: OK");
  return 0;
}
