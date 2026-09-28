// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The Orbbec driver against a real camera: open, stream, and check every frame
// against the contract -- the two cameras it is stamped with, the depth it
// carries, the timestamps -- then restart, and move the capture around. Then
// the same contract with H.265 colour, whose colour must match MJPEG's.
//
// A camera is used only when VR_ORBBEC_TEST_SERIAL names one; unset, the test
// skips (exit 0), since the cameras are exclusive and a CI leg must not take
// one from a person using the rig. A sync secondary, which streams only with
// its primary, skips too. A build without the HEVC decoder first checks, with
// no camera, that H.265 colour is refused before discovery.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/sensor/camera_capture.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_capture.hpp"

namespace vr = volumetric_kit::recon;
namespace sensor = volumetric_kit::recon::sensor;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

#define CHECK_OK(expr)                                                        \
  do {                                                                        \
    const vr::Status _s = (expr);                                             \
    if (!_s.ok()) {                                                           \
      std::fprintf(stderr, "FAIL %s:%d: %s: %s\n", __FILE__, __LINE__, #expr, \
                   _s.message().c_str());                                     \
      return 1;                                                               \
    }                                                                         \
  } while (0)

namespace {

constexpr int kFrames = 30;
constexpr auto kFrameTimeout = std::chrono::seconds(20);

// Poll until `count` frames have passed `check`, or the timeout. Returns 0 on
// success; prints and returns 1 on any failure.
template <typename Check>
int stream(sensor::ICameraCapture& capture, int count, Check&& check) {
  const auto deadline = std::chrono::steady_clock::now() + kFrameTimeout;
  int seen = 0;
  while (seen < count) {
    auto polled = capture.poll();
    if (!polled.ok()) {
      std::fprintf(stderr, "FAIL: poll: %s\n",
                   polled.status().message().c_str());
      return 1;
    }
    if (!polled.value()) {
      if (std::chrono::steady_clock::now() > deadline) {
        std::fprintf(stderr, "FAIL: %d of %d frames within the timeout\n", seen,
                     count);
        return 1;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    if (check(*polled.value()) != 0) return 1;
    ++seen;
  }
  return 0;
}

#if VR_TEST_HEVC
// The mean colour of `count` frames, per pixel and channel, as 0..255.
int mean_color(sensor::ICameraCapture& capture, int count,
               std::vector<double>* mean) {
  return stream(capture, count, [&](const sensor::CapturedFrame& f) {
    const std::size_t n =
        static_cast<std::size_t>(f.color_camera.width) * f.color_camera.height;
    if (mean->empty()) mean->assign(3 * n, 0.0);
    CHECK(mean->size() == 3 * n);
    for (std::size_t i = 0; i < n; ++i) {
      const std::uint32_t c = f.color[i];
      for (int k = 0; k < 3; ++k) {
        (*mean)[3 * i + k] += ((c >> (8 * k)) & 0xffu) / double(count);
      }
    }
    return 0;
  });
}

// Per channel, the gain of the least-squares fit `a = gain * b + offset`,
// which absorbs an exposure change between two captures of a still view.
void fitted_gains(const std::vector<double>& a, const std::vector<double>& b,
                  double gains[3]) {
  for (int k = 0; k < 3; ++k) {
    double sx = 0, sy = 0, sxx = 0, sxy = 0, n = 0;
    for (std::size_t i = static_cast<std::size_t>(k); i < a.size(); i += 3) {
      sx += b[i];
      sy += a[i];
      sxx += b[i] * b[i];
      sxy += b[i] * a[i];
      n += 1;
    }
    gains[k] = (n * sxy - sx * sy) / (n * sxx - sx * sx);
  }
}
#endif

}  // namespace

int main() {
#if !VR_TEST_HEVC
  {
    // Without the decoder, H.265 colour is refused before the SDK is
    // touched: at once, with no camera to wait for.
    sensor::OrbbecCapture::Options hevc;
    hevc.serial = "not-a-camera";
    hevc.discovery_timeout_ms = 5000;
    hevc.color_codec = sensor::OrbbecColorCodec::Hevc;
    const auto t0 = std::chrono::steady_clock::now();
    const auto refused = sensor::OrbbecCapture::open(hevc);
    CHECK(refused.status().domain() == vr::Status::Code::Unsupported);
    CHECK(std::chrono::steady_clock::now() - t0 < std::chrono::seconds(1));
  }
#endif
  const char* serial = std::getenv("VR_ORBBEC_TEST_SERIAL");
  if (serial == nullptr || *serial == '\0') {
    std::printf("SKIP: set VR_ORBBEC_TEST_SERIAL to test against a camera\n");
    return 0;
  }

  sensor::OrbbecCapture::Options options;
  options.serial = serial;
  // A non-identity pose, so the test can tell it was carried rather than
  // defaulted.
  options.cam_to_world[3] = vr::Vec4f(0.25f, -0.5f, 1.0f, 1.0f);
  auto opened = sensor::OrbbecCapture::open(options);
  if (!opened.ok()) {
    std::fprintf(stderr, "FAIL: open %s: %s\n", serial,
                 opened.status().message().c_str());
    return 1;
  }
  sensor::OrbbecCapture capture = std::move(opened).value();
  const sensor::OrbbecDeviceInfo& info = capture.device_info();
  std::printf("%s  serial %s  firmware %s  via %s %s  sync %s\n",
              info.name.c_str(), info.serial.c_str(),
              info.firmware_version.c_str(), info.connection_type.c_str(),
              info.ip_address.c_str(), sensor::to_string(info.sync_mode));
  CHECK(info.serial == serial);
  if (sensor::waits_for_primary(info.sync_mode)) {
    std::printf(
        "SKIP: camera %s is a sync %s and streams only while its primary "
        "does; name a primary or standalone camera\n",
        serial, sensor::to_string(info.sync_mode));
    return 0;
  }

  const vr::ColorCameraParams cam = capture.color_camera();
  std::printf("colour camera %ux%u  fx %.3f fy %.3f cx %.3f cy %.3f\n",
              cam.width, cam.height, cam.fx, cam.fy, cam.cx, cam.cy);
  CHECK(cam.width == options.color_width && cam.height == options.color_height);
  CHECK(cam.fx > 0.0f && cam.fy > 0.0f);
  CHECK(cam.cam_to_world == options.cam_to_world);

  // Not started: an empty poll, not an error.
  {
    auto polled = capture.poll();
    CHECK(polled.ok() && !polled.value());
  }

  CHECK_OK(capture.start());
  CHECK_OK(capture.start());  // idempotent
  std::uint64_t last_ts = 0;
  double valid_fraction_sum = 0.0;
  const auto contract = [&](const sensor::CapturedFrame& f) {
    CHECK(f.depth != nullptr && f.has_color());
    // Registered: both cameras are the colour camera, down to the pose.
    CHECK(f.color_camera.width == cam.width &&
          f.color_camera.height == cam.height);
    CHECK(f.depth_camera.width == cam.width &&
          f.depth_camera.height == cam.height);
    CHECK(f.depth_camera.fx == cam.fx && f.depth_camera.fy == cam.fy);
    CHECK(f.depth_camera.cx == cam.cx && f.depth_camera.cy == cam.cy);
    CHECK(f.depth_camera.cam_to_world == options.cam_to_world);
    CHECK(f.color_camera.cam_to_world == options.cam_to_world);
    CHECK(f.depth_camera.min_depth == options.min_depth &&
          f.depth_camera.max_depth == options.max_depth);
    CHECK(f.color_encoding.transfer == vr::ColorEncoding::Transfer::Srgb);
    // Timestamps: present, and strictly increasing within the session.
    CHECK(f.timestamp_ns > last_ts);
    last_ts = f.timestamp_ns;
    // Depth: in metres, so a return is somewhere in a room, not in
    // millimetres (thousands) or kilometres (a scale applied twice).
    const std::size_t n =
        static_cast<std::size_t>(f.depth_camera.width) * f.depth_camera.height;
    std::size_t valid = 0;
    for (std::size_t i = 0; i < n; ++i) {
      const float d = f.depth[i];
      CHECK(std::isfinite(d) && d >= 0.0f);
      if (d > 0.0f) {
        CHECK(d > 0.05f && d < 20.0f);
        ++valid;
      }
    }
    valid_fraction_sum += static_cast<double>(valid) / n;
    return 0;
  };
  const auto t0 = std::chrono::steady_clock::now();
  if (stream(capture, kFrames, contract) != 0) return 1;
  const double seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
          .count();
  const sensor::OrbbecCaptureStats st = capture.stats();
  std::printf(
      "%d frames in %.2f s; %.1f%% of registered pixels carry depth; "
      "received %llu delivered %llu dropped %llu failed %llu\n",
      kFrames, seconds, 100.0 * valid_fraction_sum / kFrames,
      static_cast<unsigned long long>(st.received),
      static_cast<unsigned long long>(st.delivered),
      static_cast<unsigned long long>(st.dropped),
      static_cast<unsigned long long>(st.failed));
  CHECK(st.delivered == static_cast<std::uint64_t>(kFrames));
  // Every pair is counted once: handed out, replaced, or failed.
  CHECK(st.delivered + st.dropped + st.failed <= st.received);
  // Any real scene returns depth over a good part of the frame; an
  // all-but-empty registered image means registration landed off the frame.
  CHECK(valid_fraction_sum / kFrames > 0.2);

  // Stop, idempotently; then a restart streams again with fresh counters.
  capture.stop();
  capture.stop();
  {
    auto polled = capture.poll();
    CHECK(polled.ok() && !polled.value());
  }
  CHECK_OK(capture.start());
  CHECK(stream(capture, 3, [](const sensor::CapturedFrame&) { return 0; }) ==
        0);
  CHECK(capture.stats().delivered == 3u);

  // Moves: the capture keeps streaming wherever it lands, and the source is
  // left empty -- exhausted, refusing to start or poll, describing nothing.
  sensor::OrbbecCapture moved(std::move(capture));
  CHECK(capture.exhausted());  // NOLINT(bugprone-use-after-move)
  CHECK(!capture.poll().ok());
  CHECK(!capture.start().ok());
  CHECK(capture.device_info().serial.empty());
  CHECK(capture.color_camera().width == 0u);
  capture.stop();  // safe on a moved-from capture
  CHECK(!moved.exhausted());
  CHECK(stream(moved, 3, [](const sensor::CapturedFrame&) { return 0; }) == 0);

  // Move-assign over the (empty) source, then self-move through a pointer so
  // -Wself-move stays quiet under -Werror.
  capture = std::move(moved);
  CHECK(moved.exhausted());  // NOLINT(bugprone-use-after-move)
  sensor::OrbbecCapture* alias = &capture;
  capture = std::move(*alias);
  CHECK(!capture.exhausted());
  CHECK(stream(capture, 3, [](const sensor::CapturedFrame&) { return 0; }) ==
        0);

  // H.265 colour. MJPEG's mean colour first, from the capture still open.
#if VR_TEST_HEVC
  std::vector<double> mjpeg_mean;
  CHECK(mean_color(capture, 10, &mjpeg_mean) == 0);
#endif
  capture.stop();
  {
    sensor::OrbbecCapture closed = std::move(capture);
  }  // frees the camera
  options.color_codec = sensor::OrbbecColorCodec::Hevc;
  auto hevc_opened = sensor::OrbbecCapture::open(options);
#if VR_TEST_HEVC
  if (!hevc_opened.ok()) {
    std::fprintf(stderr, "FAIL: open %s with H.265 colour: %s\n", serial,
                 hevc_opened.status().message().c_str());
    return 1;
  }
  sensor::OrbbecCapture hevc = std::move(hevc_opened).value();
  CHECK(hevc.color_camera().fx == cam.fx && hevc.color_camera().cx == cam.cx);
  CHECK_OK(hevc.start());
  last_ts = 0;
  valid_fraction_sum = 0.0;
  if (stream(hevc, kFrames, contract) != 0) return 1;
  std::vector<double> hevc_mean;
  CHECK(mean_color(hevc, 10, &hevc_mean) == 0);
  const sensor::OrbbecCaptureStats hs = hevc.stats();
  std::printf(
      "H.265: received %llu delivered %llu dropped %llu failed %llu lost "
      "%llu\n",
      static_cast<unsigned long long>(hs.received),
      static_cast<unsigned long long>(hs.delivered),
      static_cast<unsigned long long>(hs.dropped),
      static_cast<unsigned long long>(hs.failed),
      static_cast<unsigned long long>(hs.lost));
  CHECK(hs.delivered == static_cast<std::uint64_t>(kFrames + 10));
  CHECK(hs.delivered + hs.dropped + hs.failed + hs.lost <= hs.received);
  CHECK(valid_fraction_sum / kFrames > 0.2);
  hevc.stop();
  // Decoded as the camera codes it (BT.601 full range), the colour is
  // MJPEG's to within the exposure change between the two captures: a gain
  // near 1 per channel. Read as limited range it would be ~0.88.
  double gains[3];
  fitted_gains(mjpeg_mean, hevc_mean, gains);
  std::printf("H.265 against MJPEG: fitted gains %.3f %.3f %.3f\n", gains[0],
              gains[1], gains[2]);
  for (const double g : gains) CHECK(g > 0.95 && g < 1.05);
#else
  // Built without the HEVC decoder: H.265 colour is refused, not faked.
  CHECK(hevc_opened.status().domain() == vr::Status::Code::Unsupported);
#endif

  std::printf("orbbec capture tests passed\n");
  return 0;
}
