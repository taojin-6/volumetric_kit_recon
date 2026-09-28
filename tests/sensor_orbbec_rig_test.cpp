// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// OrbbecRig against real cameras: the rig's sync file checked against the
// cameras (and a differing one refused, with nothing written), poses read from
// a calibration file, sets whose frames share a trigger and carry their own
// camera's pose, most of them complete, restart, the one-frame-at-a-time path,
// the two readers refusing to mix, and moves.
//
// Cameras are used only when named: set VR_ORBBEC_TEST_RIG to the rig's sync
// configuration file (femto_mega_sync.json). Unset, the test skips (exit 0).
// The cameras must already match it; the test never writes to them.

#include <algorithm>
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
#include "volumetric_kit/recon/sensor/orbbec/orbbec_rig.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_sync_config.hpp"
#include "volumetric_kit/recon/sensor/rig_calibration.hpp"

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

bool near(const vr::Mat4f& a, const vr::Mat4f& b) {
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      if (std::fabs(a[c][r] - b[c][r]) > 1e-6f) return false;
    }
  }
  return true;
}

}  // namespace

int main() {
  const char* env = std::getenv("VR_ORBBEC_TEST_RIG");
  if (env == nullptr || *env == '\0') {
    std::printf("SKIP: set VR_ORBBEC_TEST_RIG to the rig's sync file\n");
    return 0;
  }
  auto sync = sensor::read_orbbec_sync_config(env);
  if (!sync.ok()) {
    std::fprintf(stderr, "FAIL: %s\n", sync.status().message().c_str());
    return 1;
  }
  // A distinct pose per camera, so a frame stamped with the wrong camera's
  // pose shows -- through a calibration file, as the rig is fed in practice.
  std::vector<sensor::RigCameraCalibration> poses;
  for (const sensor::OrbbecSyncDevice& d : sync.value().devices) {
    sensor::RigCameraCalibration c;
    c.serial = d.serial;
    c.cam_to_world[3] =
        vr::Vec4f(static_cast<float>(poses.size()), 0.5f, -1.0f, 1.0f);
    poses.push_back(c);
  }
  const std::string path =
      std::string(VR_TEST_SCRATCH_DIR) + "/orbbec_rig_test.json";
  CHECK_OK(sensor::write_rig_calibration(path, poses));
  auto read = sensor::read_rig_calibration(path);
  CHECK(read.ok());

  sensor::OrbbecRig::Options options;
  options.sync = sync.value();
  options.calibration = read.value();

  // A configuration the cameras do not match is refused, naming the camera,
  // and -- without apply_sync_config -- nothing is written.
  {
    sensor::OrbbecRig::Options differing = options;
    differing.sync.devices.back().sync.depth_delay_us += 40;
    const auto refused = sensor::OrbbecRig::open(differing);
    CHECK(!refused.ok() &&
          refused.status().domain() == vr::Status::Code::Unsupported);
    std::printf("  refused as expected: %s\n",
                refused.status().message().c_str());
    CHECK(refused.status().message().find(
              differing.sync.devices.back().serial) != std::string::npos);
  }

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
    CHECK(near(rig.color_camera(i).cam_to_world, poses[i].cam_to_world));
  }
  CHECK(rig.device_info(rig.primary()).sync_mode ==
        sensor::OrbbecSyncMode::Primary);
  CHECK(rig.device_info(n).serial.empty());  // past the end: empty, no throw

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
      CHECK(near(f.color_camera.cam_to_world, poses[i].cam_to_world));
      CHECK(near(f.depth_camera.cam_to_world, poses[i].cam_to_world));
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

  // poll() would drop the rest of a set poll_set() is reading through.
  {
    auto polled = rig.poll();
    CHECK(!polled.ok() &&
          polled.status().domain() == vr::Status::Code::InvalidArgument);
  }

  // Stop, idempotently; a restart streams again with fresh counters, and may
  // read the other way. One frame at a time: every frame carries one of the
  // rig's poses.
  rig.stop();
  rig.stop();
  {
    auto polled = rig.poll_set();
    CHECK(polled.ok() && !polled.value());
  }
  CHECK_OK(rig.start());
  const auto restart_deadline = std::chrono::steady_clock::now() + kTimeout;
  for (int k = 0; k < 3 * static_cast<int>(n);) {
    auto polled = rig.poll();
    CHECK(polled.ok());
    if (!polled.value()) {
      CHECK(std::chrono::steady_clock::now() < restart_deadline);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    const vr::Mat4f& pose = polled.value()->color_camera.cam_to_world;
    CHECK(std::any_of(poses.begin(), poses.end(), [&](const auto& p) {
      return near(p.cam_to_world, pose);
    }));
    ++k;
  }
  // Each set holds one to n frames, so 3n frames took 3 to 3n sets.
  const std::uint64_t restarted_sets = rig.stats().sets;
  CHECK(restarted_sets >= 3u && restarted_sets <= 3u * n);
  CHECK(!rig.poll_set().ok());  // and the other way round

  // Moves: the rig streams wherever it lands; the source is left empty.
  sensor::OrbbecRig moved(std::move(rig));
  CHECK(rig.exhausted());  // NOLINT(bugprone-use-after-move)
  CHECK(rig.camera_count() == 0u);
  CHECK(rig.device_info(0).serial.empty());
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
