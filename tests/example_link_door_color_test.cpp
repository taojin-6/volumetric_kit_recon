// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// A door's session read live with its pictures
// (examples/common/link/rgbd_stream_sensor.hpp, RgbdStreamSensor::Door) from
// a writer that dials 127.0.0.1 (tests/link_dialler.hpp), as
// example_link_listen_color_test reads listen's: each picture the family's
// test pattern of its frame's index, coded in the test by sensor's HevcEncoder
// on this machine's hardware, each depth the same index's pattern depth, and
// session_check's checks reading both on the device (tests/link_pictures.hpp).
// - Every frame carries its own picture over the link. A colour record the
//   decode refuses makes the link let the connection go (one failure), and
//   the writer's next connection, starting at a key, goes to the same session
//   with no new arrival and brings the rest with their own pictures. The
//   recording, read back on the device, gives the same frames with the same
//   pictures. The report says rgbd_breach: none.
// Where no hardware coder opens, the test skips; VS_TEST_HEVC_BACKEND, which
// CI sets on the legs that code both ways (NVENC and NVDEC, VideoToolbox),
// makes it fail instead. In a build without sensor's video coders it checks
// nothing, and passes.

#include <cstdio>

#include "volumetric_kit/core/testing/gpu_test.hpp"
#include "volumetric_kit/sensor/frame/device_requirements.hpp"

#if VR_TEST_LINK_VIDEO

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "link/rgbd_stream_sensor.hpp"
#include "link_dialler.hpp"
#include "link_pictures.hpp"
#include "session_check.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/testing/net_loopback.hpp"
#include "volumetric_kit/core/testing/scratch_dir.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/sensor/testing/hevc_clip.hpp"
#include "volumetric_kit/sensor/video/hevc_decoder.hpp"
#include "volumetric_kit/sensor/video/hevc_encoder.hpp"

#endif

namespace {

namespace vkc = volumetric_kit::core;
namespace sensor = volumetric_kit::sensor;
using Gpu = vkc::test::GpuContext;

#if VR_TEST_LINK_VIDEO

using Code = vkc::Status::Code;
using vr_example::link::RgbdStreamSensor;
using vr_example::link::SessionHandle;
using vr_example::link::SessionReport;
using vr_test::Dialler;
using vr_test::PatternCoder;
using vr_test::PatternFrames;
using Door = RgbdStreamSensor::Door;

// 12 frames, keys at 0 and 6. Frames 0-3 whole, then frame 4's colour record
// refused by the decode: the link lets the connection go. Once the link has
// seen it go, the writer's next connection sends frames 6-11 from the key, a
// reconnect of the session. Every frame handed out has its own picture, live
// and from the recording.
int test_pictures(const Gpu& gpu, const vkc::test::Scratch& scratch) {
  constexpr std::uint32_t kW = 256;
  constexpr std::uint32_t kH = 144;
  const PatternFrames p =
      vr_test::color_frames(vr_test::phone_info(kW, kH), 12);
  vkc::Result<std::unique_ptr<PatternCoder>> coder =
      PatternCoder::create(gpu, kW, kH);
  CHECK(coder.ok());
  std::vector<sensor::HevcEncoder::CodedPicture> pictures;
  for (std::uint32_t i = 0; i < 12; ++i) {
    vkc::Result<sensor::HevcEncoder::CodedPicture> coded = coder.value()->code(
        i, i == 0 || i == 6,
        static_cast<std::int64_t>(p.frames[i].timestamp_ns));
    if (!coded.ok()) {
      std::fprintf(stderr, "%s\n", coded.status().message().c_str());
    }
    CHECK(coded.ok());
    pictures.push_back(std::move(coded).value());
  }
  CHECK(pictures[0].key && pictures[6].key && !pictures[5].key);
  const sensor::ColorCodes codes = coder.value()->codes();

  const std::string recording = (scratch.path / "pictures.vrst").string();
  const std::uint16_t port = vkc::test::closed_port();
  vkc::Result<std::unique_ptr<Door>> door = Door::open("127.0.0.1", port);
  if (!door.ok()) std::fprintf(stderr, "%s\n", door.status().message().c_str());
  CHECK(door.ok());
  std::atomic<SessionHandle*> watched{nullptr};
  std::atomic<bool> done{false};
  bool sent = false;
  std::thread writer([&] {
    {
      auto d = Dialler::dial(vr_test::session_of(p.info), port);
      if (!d.ok()) return;
      for (std::size_t i = 0; i < 4; ++i) {
        if (!vr_test::publish_frame(**d, p, codes, pictures[i], i).ok()) return;
      }
      sensor::HevcEncoder::CodedPicture broken = pictures[4];
      broken.access_unit.clear();
      if (!vr_test::publish_frame(**d, p, codes, broken, 4).ok()) return;
      if (!(*d)->let_go()) return;  // the link's
    }
    // Seen gone, so the next connection is a reconnect, not one superseding.
    if (!vr_test::wait_for([&] {
          const SessionHandle* h = watched.load();
          return h != nullptr && !h->connected();
        })) {
      return;
    }
    auto d = Dialler::dial(vr_test::session_of(p.info), port);
    if (!d.ok()) return;
    for (std::size_t i = 6; i < 12; ++i) {
      if (!vr_test::publish_frame(**d, p, codes, pictures[i], i).ok()) return;
    }
    sent = vr_test::wait_for([&] { return done.load(); });
  });
  std::shared_ptr<SessionHandle> handle;
  std::unique_ptr<RgbdStreamSensor> live;
  std::optional<Door::Arrival> arrival =
      door.value()->take_arrival(std::chrono::seconds(5));
  if (arrival) {
    handle = arrival->handle;
    watched = handle.get();
    vkc::Result<std::unique_ptr<RgbdStreamSensor>> admitted =
        door.value()->admit(std::move(*arrival), vr_test::on_device(gpu),
                            recording);
    if (admitted.ok()) {
      live = std::move(admitted).value();
    } else {
      std::fprintf(stderr, "admit: %s\n", admitted.status().message().c_str());
    }
  }
  std::vector<std::uint64_t> seqs;
  vkc::Result<SessionReport> report =
      live ? vr_test::check_frames(gpu, *live, 10, nullptr, &seqs)
           : vkc::Result<SessionReport>(vkc::Status::io_error("no session"));
  const bool no_arrival = !door.value()->take_arrival(std::chrono::seconds(0));
  done = true;
  writer.join();
  if (!report.ok()) {
    std::fprintf(stderr, "%s\n", report.status().message().c_str());
  }
  CHECK(live && report.ok() && sent && no_arrival);
  const SessionReport& r = report.value();
  std::fputs(vr_example::link::format(r).c_str(), stdout);
  CHECK((seqs == std::vector<std::uint64_t>{0, 1, 2, 3, 6, 7, 8, 9, 10, 11}));
  CHECK(r.ok() && r.own_picture == 10 && r.wrong == 0 &&
        r.no_picture_index == 0 && r.no_depth_index == 0);
  CHECK(r.stats.received == 10 && r.stats.delivered == 10 &&
        r.stats.failed == 0);
  CHECK(r.depth_spans == 1 && r.depth_lost == 2 && r.window_splits == 1);
  CHECK(r.link.connections == 2 && r.link.failures == 1 &&
        r.link.superseded == 0 && r.link.refused == 0 && r.link.recorded == 21);
  CHECK(r.latency_ms.has_value());
  CHECK(vr_example::link::format(r).find("rgbd_breach: none\n") !=
        std::string::npos);
  const Door::Stats stats = door.value()->stats();
  CHECK(stats.arrivals == 1 && stats.admitted == 1 && stats.reconnects == 1 &&
        stats.refused == 0);

  // The recording, read back on the device: the same frames, each with its
  // own picture.
  live.reset();
  vkc::Result<std::unique_ptr<RgbdStreamSensor>> file =
      RgbdStreamSensor::open_file(recording, vr_test::on_device(gpu));
  CHECK(file.ok());
  std::vector<std::uint64_t> file_seqs;
  vkc::Result<SessionReport> again = vr_test::check_frames(
      gpu, *file.value(), ~std::size_t{0}, nullptr, &file_seqs);
  CHECK(again.ok());
  CHECK(file_seqs == seqs);
  CHECK(again.value().ok() && again.value().own_picture == 10 &&
        !again.value().latency_ms);
  CHECK(vr_example::link::format(again.value()).find("rgbd_breach: none\n") !=
        std::string::npos);
  return 0;
}

int gpu_main(Gpu& gpu) {
  {
    vkc::Result<std::unique_ptr<PatternCoder>> probe =
        PatternCoder::create(gpu, 256, 144);
    sensor::HevcDecoder::Options options;
    options.device = &gpu.device;
    options.allocator = &gpu.allocator;
    vkc::Result<sensor::HevcDecoder> decoder =
        sensor::HevcDecoder::create(options);
    const vkc::Status why = !probe.ok() ? probe.status() : decoder.status();
    if (!why.ok()) {
      if (why.domain() != Code::Unsupported) {
        std::fprintf(stderr, "FAIL: %s\n", why.message().c_str());
        return 1;
      }
      const bool promised = sensor::testing::hevc_decoder_promised();
      std::fprintf(stderr, "no hardware coder (%s); %s\n",
                   why.message().c_str(), promised ? "failing" : "skipping");
      return promised ? 1 : vkc::test::kSkipExitCode;
    }
  }
  const vkc::test::Scratch scratch("recon_link_door_color_");
  if (test_pictures(gpu, scratch) != 0) return 1;
  std::printf("rgbd stream sensor door colour tests passed\n");
  return 0;
}

vkc::Result<vkc::DeviceRequirements> requirements() {
  return vkc::merge(sensor::device_requirements(),
                    sensor::hevc_encoder_device_requirements());
}

#else  // VR_TEST_LINK_VIDEO

int gpu_main(Gpu&) {
  std::printf("rgbd stream sensor door colour tests: no video coders\n");
  return 0;
}

vkc::Result<vkc::DeviceRequirements> requirements() {
  return sensor::device_requirements();
}

#endif  // VR_TEST_LINK_VIDEO

}  // namespace

int main() {
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  const vkc::Result<vkc::DeviceRequirements> reqs = requirements();
  if (!reqs.ok()) {
    std::fprintf(stderr, "FAIL: %s\n", reqs.status().message().c_str());
    return 1;
  }
  return vkc::test::run_on_gpu(reqs.value(), gpu_main);
}
