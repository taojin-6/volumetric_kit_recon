// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The Orbbec driver against a real camera: open, stream, and check every frame
// against the contract -- the two cameras it is stamped with, the depth it
// carries, the timestamps -- then restart, and move the capture around.
//
// A camera is used only when one is named: set VR_ORBBEC_TEST_SERIAL to its
// serial. Unset, the test skips (exit 0). Discovery is a network broadcast and
// the rig's cameras are exclusive, so a CI leg on a machine that happens to
// share a network with a camera must not open it -- two build types running in
// parallel would fight over it, and a person using the rig would lose it
// mid-capture. The camera must stream on its own: a sync secondary delivers
// nothing without its primary, so naming one skips with a message saying so.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <string>
#include <thread>
#include <utility>

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

}  // namespace

int main() {
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
  const auto t0 = std::chrono::steady_clock::now();
  const int streamed =
      stream(capture, kFrames, [&](const sensor::CapturedFrame& f) {
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
        const std::size_t n = static_cast<std::size_t>(f.depth_camera.width) *
                              f.depth_camera.height;
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
      });
  if (streamed != 0) return 1;
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
  capture.stop();

  std::printf("orbbec capture tests passed\n");
  return 0;
}
