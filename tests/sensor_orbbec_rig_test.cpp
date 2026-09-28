// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// OrbbecRig against real cameras: poses read from a rig pose file, sets whose
// frames share a trigger and carry their own camera's pose, most of them
// complete, the one-frame-at-a-time path, restart, and moves.
//
// Cameras are used only when named: set VR_ORBBEC_TEST_RIG to their serials,
// comma-separated, primary included. Unset, the test skips (exit 0).

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_rig.hpp"
#include "volumetric_kit/recon/sensor/rig_poses.hpp"

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

constexpr int kSets = 90;
constexpr auto kTimeout = std::chrono::seconds(30);

}  // namespace

int main() {
  const char* env = std::getenv("VR_ORBBEC_TEST_RIG");
  if (env == nullptr || *env == '\0') {
    std::printf("SKIP: set VR_ORBBEC_TEST_RIG to the rig's serials\n");
    return 0;
  }
  std::vector<sensor::RigCameraPose> poses;
  {
    std::stringstream list(env);
    std::string serial;
    while (std::getline(list, serial, ',')) {
      sensor::RigCameraPose pose;
      pose.serial = serial;
      // A distinct translation per camera, so a frame stamped with the wrong
      // camera's pose shows.
      pose.cam_to_world[3] =
          vr::Vec4f(static_cast<float>(poses.size()), 0.5f, -1.0f, 1.0f);
      poses.push_back(pose);
    }
  }
  // Through the file, as the rig is fed in practice.
  const std::string path =
      std::string(VR_TEST_SCRATCH_DIR) + "/orbbec_rig_test.json";
  CHECK_OK(sensor::write_rig_poses(path, poses));
  auto read = sensor::read_rig_poses(path);
  CHECK(read.ok());

  sensor::OrbbecRig::Options options;
  options.cameras = read.value();
  auto opened = sensor::OrbbecRig::open(options);
  if (!opened.ok()) {
    std::fprintf(stderr, "FAIL: open: %s\n", opened.status().message().c_str());
    return 1;
  }
  sensor::OrbbecRig rig = std::move(opened).value();
  const std::size_t n = rig.camera_count();
  CHECK(n == poses.size());
  for (std::size_t i = 0; i < n; ++i) {
    const sensor::OrbbecDeviceInfo& info = rig.device_info(i);
    std::printf("  [%zu] %s  %s  %s  sync %s%s\n", i, info.serial.c_str(),
                info.ip_address.c_str(), info.firmware_version.c_str(),
                sensor::to_string(info.sync_mode),
                i == rig.primary() ? "  (started last)" : "");
    CHECK(info.serial == poses[i].serial);
    CHECK(rig.color_camera(i).cam_to_world == poses[i].cam_to_world);
  }
  CHECK(rig.device_info(rig.primary()).sync_mode ==
        sensor::OrbbecSyncMode::Primary);

  // Not started: an empty poll, not an error.
  {
    auto polled = rig.poll_set();
    CHECK(polled.ok() && !polled.value());
  }

  CHECK_OK(rig.start());
  CHECK_OK(rig.start());  // idempotent
  int sets = 0, complete = 0;
  std::uint64_t last_ts = 0, worst_skew_us = 0;
  double process_ms = 0.0;
  const auto deadline = std::chrono::steady_clock::now() + kTimeout;
  while (sets < kSets) {
    const auto t0 = std::chrono::steady_clock::now();
    auto polled = rig.poll_set();
    const auto t1 = std::chrono::steady_clock::now();
    if (!polled.ok()) {
      std::fprintf(stderr, "FAIL: poll_set: %s\n",
                   polled.status().message().c_str());
      return 1;
    }
    if (!polled.value()) {
      CHECK(std::chrono::steady_clock::now() < deadline);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    const sensor::OrbbecRigFrameSet& set = *polled.value();
    process_ms += std::chrono::duration<double, std::milli>(t1 - t0).count();
    CHECK(set.frames.size() == n);
    CHECK(set.count() > 0);
    CHECK(set.timestamp_ns > last_ts);
    last_ts = set.timestamp_ns;
    for (std::size_t i = 0; i < n; ++i) {
      if (!set.frames[i]) continue;
      const sensor::CapturedFrame& f = *set.frames[i];
      CHECK(f.depth != nullptr && f.has_color());
      // Posed by its own camera.
      CHECK(f.color_camera.cam_to_world == poses[i].cam_to_world);
      CHECK(f.depth_camera.cam_to_world == poses[i].cam_to_world);
      // On the trigger's clock, within the tolerance.
      const std::uint64_t skew_us = (f.timestamp_ns > set.timestamp_ns
                                         ? f.timestamp_ns - set.timestamp_ns
                                         : set.timestamp_ns - f.timestamp_ns) /
                                    1000;
      CHECK(skew_us <= options.sync_tolerance_us);
      worst_skew_us = std::max(worst_skew_us, skew_us);
    }
    ++sets;
    complete += set.complete() ? 1 : 0;
  }
  const sensor::OrbbecRigStats st = rig.stats();
  std::printf(
      "%d sets, %d complete (%.1f%%), worst skew %.2f ms, mean %.2f ms per "
      "poll_set that handed one out\n",
      sets, complete, 100.0 * complete / sets, worst_skew_us / 1000.0,
      process_ms / sets);
  for (std::size_t i = 0; i < n; ++i) {
    const sensor::OrbbecCaptureStats& c = st.cameras[i];
    std::printf(
        "  [%zu] received %llu delivered %llu dropped %llu failed %llu\n", i,
        static_cast<unsigned long long>(c.received),
        static_cast<unsigned long long>(c.delivered),
        static_cast<unsigned long long>(c.dropped),
        static_cast<unsigned long long>(c.failed));
    CHECK(c.delivered + c.dropped + c.failed <= c.received);
  }
  CHECK(st.sets == static_cast<std::uint64_t>(sets));
  CHECK(st.incomplete == static_cast<std::uint64_t>(sets - complete));
  // Measured on the rig over the cable: ~92% of four-camera triggers complete.
  CHECK(complete * 10 >= sets * 8);

  // One frame at a time: every frame carries one of the rig's poses.
  for (int k = 0; k < 3 * static_cast<int>(n);) {
    auto polled = rig.poll();
    CHECK(polled.ok());
    if (!polled.value()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    const vr::Mat4f& pose = polled.value()->color_camera.cam_to_world;
    CHECK(std::any_of(poses.begin(), poses.end(),
                      [&](const auto& p) { return p.cam_to_world == pose; }));
    ++k;
  }

  // Stop, idempotently; a restart streams again with fresh counters.
  rig.stop();
  rig.stop();
  {
    auto polled = rig.poll_set();
    CHECK(polled.ok() && !polled.value());
  }
  CHECK_OK(rig.start());
  for (int got = 0; got < 3;) {
    auto polled = rig.poll_set();
    CHECK(polled.ok());
    if (polled.value()) {
      ++got;
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  CHECK(rig.stats().sets == 3u);

  // Moves: the rig streams wherever it lands; the source is left empty.
  sensor::OrbbecRig moved(std::move(rig));
  CHECK(rig.exhausted());  // NOLINT(bugprone-use-after-move)
  CHECK(rig.camera_count() == 0u);
  CHECK(!rig.poll_set().ok());
  CHECK(!rig.start().ok());
  rig.stop();  // safe on a moved-from rig
  CHECK(!moved.exhausted());
  rig = std::move(moved);
  sensor::OrbbecRig* alias = &rig;
  rig = std::move(*alias);  // self-move, laundered past -Wself-move
  CHECK(rig.camera_count() == n);
  rig.stop();

  std::printf("orbbec rig tests passed\n");
  return 0;
}
