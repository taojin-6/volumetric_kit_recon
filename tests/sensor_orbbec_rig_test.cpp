// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// OrbbecRig against real cameras: the rig's sync file checked against the
// cameras (and a differing one refused, with nothing written), poses read from
// a calibration file, sets whose frames share a trigger and carry their own
// camera's pose, most of them complete, restart, the one-frame-at-a-time path,
// the readers refusing to mix, and moves; then the same with H.265 colour,
// and raw over either codec, each camera's set prepared on the GPU.
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
#if VR_TEST_HEVC
#include "buffer_readback.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/instance.hpp"
#include "volumetric_kit/recon/camera/array_calibration.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/sensor/utils/gpu_frame_prep.hpp"
#endif
#include "volumetric_kit/recon/sensor/orbbec/orbbec_rig.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_sync_config.hpp"

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;
namespace sensor = volumetric_kit::recon::sensor;
namespace camera = volumetric_kit::recon::camera;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

#define CHECK_OK(expr)                                                        \
  do {                                                                        \
    const vkc::Status _s = (expr);                                            \
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
  camera::ArrayCalibration poses;
  for (const sensor::OrbbecSyncDevice& d : sync.value().devices) {
    camera::SensorCalibration c;
    c.id = d.serial;
    c.color_to_world = camera::Mat4d(1.0);
    (*c.color_to_world)[3] =
        glm::dvec4(static_cast<double>(poses.sensors.size()), 0.5, -1.0, 1.0);
    poses.sensors.push_back(c);
  }
  const std::string path =
      std::string(VR_TEST_SCRATCH_DIR) + "/orbbec_rig_test.json";
  CHECK_OK(camera::write_array_calibration(path, poses));
  auto read = camera::read_array_calibration(path);
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
          refused.status().domain() == vkc::Status::Code::Unsupported);
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
  CHECK(n == poses.sensors.size());
  for (std::size_t i = 0; i < n; ++i) {
    const sensor::OrbbecDeviceInfo& info = rig.device_info(i);
    std::printf("  [%zu] %s  %s  %s  sync %s%s\n", i, info.serial.c_str(),
                info.ip_address.c_str(), info.firmware_version.c_str(),
                sensor::to_string(info.sync_mode),
                i == rig.primary() ? "  (started last)" : "");
    CHECK(info.serial == poses.sensors[i].id);
    CHECK(near(rig.color_camera(i).cam_to_world,
               vr::Mat4f(*poses.sensors[i].color_to_world)));
  }
  CHECK(rig.device_info(rig.primary()).sync_mode ==
        sensor::OrbbecSyncMode::Primary);
  CHECK(rig.device_info(n).serial.empty());  // past the end: empty, no throw

  // Not started: an empty poll, not an error.
  {
    auto polled = rig.poll_set();
    CHECK(polled.ok() && !polled.value());
  }

  // Stream kSets sets and hold each to the contract; print and check the
  // counters. The same for either colour codec.
  const auto run_sets = [&](sensor::OrbbecRig& rig, const char* label) -> int {
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
        CHECK(near(f.color_camera.cam_to_world,
                   vr::Mat4f(*poses.sensors[i].color_to_world)));
        CHECK(near(f.depth_camera.cam_to_world,
                   vr::Mat4f(*poses.sensors[i].color_to_world)));
        // On the trigger's clock, within the tolerance.
        const std::uint64_t skew_us =
            (f.timestamp_ns > set.timestamp_ns
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
        "%s: %d sets, %d complete (%.1f%%), worst skew %.2f ms, mean %.2f ms "
        "per "
        "poll_set that handed one out\n",
        label, sets, complete, 100.0 * complete / sets, worst_skew_us / 1000.0,
        process_ms / sets);
    for (std::size_t i = 0; i < n; ++i) {
      const sensor::OrbbecCaptureStats& c = st.cameras[i];
      std::printf(
          "  [%zu] received %llu delivered %llu dropped %llu failed %llu lost "
          "%llu\n",
          i, static_cast<unsigned long long>(c.received),
          static_cast<unsigned long long>(c.delivered),
          static_cast<unsigned long long>(c.dropped),
          static_cast<unsigned long long>(c.failed),
          static_cast<unsigned long long>(c.lost));
      CHECK(c.delivered + c.dropped + c.failed + c.lost <= c.received);
      // H.265 loses only colour frames whose depth never came (under 2% on
      // the rig); gaps costing the stream to its next key frame lost 8-30%.
      CHECK(c.lost * 20 <= c.received);
    }
    CHECK(st.sets == static_cast<std::uint64_t>(sets));
    CHECK(st.incomplete == static_cast<std::uint64_t>(sets - complete));
    // Measured on the rig over the cable: ~92% of four-camera triggers
    // complete.
    CHECK(complete * 10 >= sets * 8);
    return 0;
  };
  if (run_sets(rig, "MJPEG") != 0) return 1;

  // poll() would drop the rest of a set poll_set() is reading through.
  {
    auto polled = rig.poll();
    CHECK(!polled.ok() &&
          polled.status().domain() == vkc::Status::Code::InvalidArgument);
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
    CHECK(std::any_of(poses.sensors.begin(), poses.sensors.end(),
                      [&](const auto& p) {
                        return near(vr::Mat4f(*p.color_to_world), pose);
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

  // The rig again with H.265 colour: the same contract, as complete.
  {
    sensor::OrbbecRig closed = std::move(rig);
  }  // frees the cameras
  options.color_codec = sensor::OrbbecColorCodec::Hevc;
  auto hevc_opened = sensor::OrbbecRig::open(options);
#if VR_TEST_HEVC
  if (!hevc_opened.ok()) {
    std::fprintf(stderr, "FAIL: open with H.265 colour: %s\n",
                 hevc_opened.status().message().c_str());
    return 1;
  }
  sensor::OrbbecRig hevc = std::move(hevc_opened).value();
  if (run_sets(hevc, "H.265") != 0) return 1;
  hevc.stop();
  // Not opened raw.
  CHECK(!hevc.raw_frames());
  CHECK(hevc.poll_raw_set().status().domain() ==
        vkc::Status::Code::InvalidArgument);
  CHECK(hevc.poll_raw().status().domain() ==
        vkc::Status::Code::InvalidArgument);

  // The rig raw, over either codec: whole sets as the cameras captured them,
  // each posed by its calibration, prepared on the GPU at once, one thread
  // per camera.
  {
    sensor::OrbbecRig closed = std::move(hevc);
  }  // frees the cameras
  vkc::Result<vkc::Instance> instance = vkc::Instance::create({});
  CHECK(instance.ok());
  vkc::Result<vkc::PhysicalDeviceInfo> gpu =
      instance.value().select_physical_device(vr::device_requirements());
  CHECK(gpu.ok());
  vkc::Result<vkc::Device> device = vkc::Device::create(
      instance.value(), gpu.value(), vr::device_requirements());
  CHECK(device.ok());
  vkc::Result<vkc::Allocator> allocator =
      vkc::Allocator::create(instance.value().handle(), device.value());
  CHECK(allocator.ok());
  std::vector<sensor::GpuFramePrep> preps;
  for (std::size_t c = 0; c < n; ++c) {
    auto prep = sensor::GpuFramePrep::create(device.value(), allocator.value());
    CHECK(prep.ok());
    preps.push_back(std::move(prep).value());
  }
  options.raw = true;
  for (const sensor::OrbbecColorCodec codec :
       {sensor::OrbbecColorCodec::Mjpeg, sensor::OrbbecColorCodec::Hevc}) {
    options.color_codec = codec;
    auto raw_opened = sensor::OrbbecRig::open(options);
    if (!raw_opened.ok()) {
      std::fprintf(stderr, "FAIL: open raw %s: %s\n", sensor::to_string(codec),
                   raw_opened.status().message().c_str());
      return 1;
    }
    sensor::OrbbecRig raw_rig = std::move(raw_opened).value();
    CHECK(raw_rig.raw_frames());
    CHECK_OK(raw_rig.start());
    CHECK(raw_rig.poll_set().status().domain() ==
          vkc::Status::Code::InvalidArgument);
    CHECK(raw_rig.poll().status().domain() ==
          vkc::Status::Code::InvalidArgument);
    // Each depth camera sits a few centimetres from its colour camera, by the
    // camera's own extrinsic, so a depth frame posed by another camera, or
    // without the extrinsic, shows.
    std::vector<std::optional<vr::Mat4f>> depth_in_color(
        raw_rig.camera_count());
    int prepared_sets = 0;
    std::uint64_t last_raw_ts = 0;
    const auto raw_deadline = std::chrono::steady_clock::now() + kTimeout;
    while (prepared_sets < 10 &&
           std::chrono::steady_clock::now() < raw_deadline) {
      auto polled = raw_rig.poll_raw_set();
      CHECK_OK(polled.status());
      if (!polled.value()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      const sensor::OrbbecRigRawSet& set = *polled.value();
      CHECK(set.frames.size() == raw_rig.camera_count());
      CHECK(set.timestamp_ns > last_raw_ts);
      last_raw_ts = set.timestamp_ns;
      for (std::size_t c = 0; c < set.frames.size(); ++c) {
        if (!set.frames[c]) continue;
        const sensor::RawFrame& f = *set.frames[c];
        CHECK(f.depth != nullptr && f.has_color());
        CHECK(near(f.color_cam_to_world,
                   vr::Mat4f(*poses.sensors[c].color_to_world)));
        const vr::Mat4f rel =
            glm::inverse(f.color_cam_to_world) * f.depth_cam_to_world;
        CHECK(glm::length(vr::Vec3f(rel[3])) < 0.1f);
        if (!depth_in_color[c]) depth_in_color[c] = rel;
        CHECK(near(rel, *depth_in_color[c]));
        const std::uint64_t skew_us =
            (f.timestamp_ns > set.timestamp_ns
                 ? f.timestamp_ns - set.timestamp_ns
                 : set.timestamp_ns - f.timestamp_ns) /
            1000;
        CHECK(skew_us <= options.sync_tolerance_us);
      }
      // A missing camera's slot stays empty.
      auto prepared = sensor::prepare_set(preps, set.frames);
      CHECK_OK(prepared.status());
      for (std::size_t c = 0; c < set.frames.size(); ++c) {
        const std::optional<sensor::DeviceFrame>& frame = prepared.value()[c];
        CHECK(frame.has_value() == set.frames[c].has_value());
        if (!frame) continue;
        CHECK(frame->has_color());
        auto depth = vr_test::read_back<float>(
            device.value(), allocator.value(), *frame->depth,
            std::size_t{frame->depth_camera.width} *
                frame->depth_camera.height);
        CHECK(depth.ok());
        CHECK(std::any_of(depth.value().begin(), depth.value().end(),
                          [](float d) { return d > 0.0f; }));
      }
      ++prepared_sets;
    }
    CHECK(prepared_sets == 10);

    // Restarted, one raw frame at a time through the contract, each posed by
    // one of the rig's cameras; and then the sets are refused.
    raw_rig.stop();
    CHECK_OK(raw_rig.start());
    const auto raw_frames_deadline =
        std::chrono::steady_clock::now() + kTimeout;
    for (int k = 0; k < 3 * static_cast<int>(n);) {
      auto polled = raw_rig.poll_raw();
      CHECK_OK(polled.status());
      if (!polled.value()) {
        CHECK(std::chrono::steady_clock::now() < raw_frames_deadline);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      const vr::Mat4f& pose = polled.value()->color_cam_to_world;
      CHECK(std::any_of(poses.sensors.begin(), poses.sensors.end(),
                        [&](const auto& p) {
                          return near(vr::Mat4f(*p.color_to_world), pose);
                        }));
      ++k;
    }
    CHECK(raw_rig.poll_raw_set().status().domain() ==
          vkc::Status::Code::InvalidArgument);
    raw_rig.stop();
  }  // frees the cameras
#else
  CHECK(hevc_opened.status().domain() == vkc::Status::Code::Unsupported);
#endif

  std::printf("orbbec rig tests passed\n");
  return 0;
}
