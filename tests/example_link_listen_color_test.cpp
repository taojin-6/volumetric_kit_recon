// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// A session's sensor read live with its pictures
// (examples/common/link/rgbd_stream_sensor.hpp, RgbdStreamSensor::listen)
// from a writer that dials 127.0.0.1 (tests/link_dialler.hpp): each picture
// the family's test pattern of its frame's index, coded in the test by
// sensor's HevcEncoder on this machine's hardware (VideoToolbox, NVENC) from
// targets written on the device, each depth the same index's pattern depth.
// session_check's checks (tools/session_check/session_check.hpp) read both
// indices on the device.
// - Every frame carries its own picture over the link. A colour record the
//   decode refuses makes the reader let the connection go (one failure), and
//   the writer's next connection, starting at a key, brings the rest with
//   their own pictures. The recording, read back on the device, gives the
//   same frames with the same pictures. The report says rgbd_breach: none.
// Where no hardware coder opens, the test skips; VS_TEST_HEVC_BACKEND, which
// CI sets on the legs that code both ways (NVENC and NVDEC, VideoToolbox),
// makes it fail instead. In a build without sensor's video coders it checks
// nothing more than the host test does, and passes.
//
// A sender for a hand check (README, "Checking a phone's stream"): with
// VR_TEST_SESSION_CHECK=<ipv4>:<port> set, the test instead dials that
// address and streams VR_TEST_SESSION_FRAMES frames (300 unless set) at
// 10 Hz, 1920x1440 pictures and ARKit's 256x192 depth, each frame's index in
// both, redialling once a second after a drop and coding a key at each join,
// as an iPad's writer does; session_check listening there checks them.

#include <cstdio>
#include <cstdlib>

#include "volumetric_kit/core/testing/gpu_test.hpp"
#include "volumetric_kit/sensor/frame/device_requirements.hpp"

#if VR_TEST_LINK_VIDEO

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "link/rgbd_stream_sensor.hpp"
#include "link_dialler.hpp"
#include "session_check.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/net/address.hpp"
#include "volumetric_kit/core/net/dial.hpp"
#include "volumetric_kit/core/testing/net_loopback.hpp"
#include "volumetric_kit/core/testing/scratch_dir.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/core/vulkan/sync.hpp"
#include "volumetric_kit/grid/stream/stream_reader.hpp"
#include "volumetric_kit/sensor/testing/hevc_clip.hpp"
#include "volumetric_kit/sensor/testing/nv12_pattern.hpp"
#include "volumetric_kit/sensor/testing/nv12_writer.hpp"
#include "volumetric_kit/sensor/video/color_codes.hpp"
#include "volumetric_kit/sensor/video/hevc_decoder.hpp"
#include "volumetric_kit/sensor/video/hevc_encoder.hpp"
#include "volumetric_kit/sensor/wire/rgbd_wire.hpp"

#endif

namespace {

namespace vkc = volumetric_kit::core;
namespace sensor = volumetric_kit::sensor;
using Gpu = vkc::test::GpuContext;

#if VR_TEST_LINK_VIDEO

namespace stream = volumetric_kit::grid::stream;
namespace testing = volumetric_kit::sensor::testing;
namespace wire = volumetric_kit::sensor::wire;
namespace gnet = volumetric_kit::grid::networking;
using Code = vkc::Status::Code;
using vr_example::link::RgbdStreamSensor;
using vr_example::link::SessionCheck;
using vr_example::link::SessionReport;
using vr_test::Dialler;
using vr_test::PatternFrames;
using vr_test::Record;

// What a frame's colour is to wire::encode_color, which asks only that it
// has one: the pictures are coded apart.
constexpr std::uint8_t kSomeColor = 0;

// As an iPad's pictures are tagged: BT.601's matrix, BT.709's transfer and
// primaries, full range, sited left.
constexpr sensor::ColorCodes kCodes{6, 1, 1, true, 0};

// The test pattern's pictures coded by sensor's HevcEncoder, each written
// into its target on the device (testing::Nv12Writer), as a producer's own
// kernel writes one, and coded before the next is sent.
class PatternCoder {
 public:
  static vkc::Result<std::unique_ptr<PatternCoder>> create(
      const Gpu& gpu, std::uint32_t width, std::uint32_t height) {
    sensor::HevcEncoder::Options options;
    options.width = width;
    options.height = height;
    options.qp = 26;
    options.color = kCodes;
    options.device = &gpu.device;
    options.allocator = &gpu.allocator;
    VKC_ASSIGN(sensor::HevcEncoder encoder,
               sensor::HevcEncoder::create(options));
    VKC_ASSIGN(testing::Nv12Writer writer,
               testing::Nv12Writer::create(gpu.device));
    VKC_ASSIGN(vkc::TimelineSemaphore written,
               vkc::TimelineSemaphore::create(gpu.device));
    return std::unique_ptr<PatternCoder>(
        new PatternCoder(gpu, std::move(encoder), std::move(writer),
                         std::move(written), width, height));
  }

  // Picture `index` of the pattern coded at `pts`: a key when asked or
  // owed.
  vkc::Result<sensor::HevcEncoder::CodedPicture> code(std::uint32_t index,
                                                      bool key,
                                                      std::int64_t pts) {
    vkc::CommandBatch batch(gpu_.device, gpu_.allocator);
    VKC_ASSIGN(const sensor::YuvImage target, encoder_.target(batch));
    VKC_TRY(writer_.record_upload(batch, gpu_.allocator,
                                  testing::test_pattern(width_, height_, index),
                                  target));
    const std::uint64_t value = ++value_;
    VKC_ASSIGN(vkc::PendingBatch pending,
               batch.submit_async({}, {{&written_, value}}));
    VKC_TRY(encoder_.send(key || encoder_.key_owed(), pts, {&written_, value}));
    sensor::HevcEncoder::CodedPicture coded;
    VKC_ASSIGN(const bool got, encoder_.receive(coded));
    VKC_TRY(pending.wait());
    if (!got) return vkc::Status::io_error("the encoder dropped a picture");
    return coded;
  }

  const sensor::ColorCodes& codes() const { return encoder_.facts().color; }

 private:
  PatternCoder(const Gpu& gpu, sensor::HevcEncoder encoder,
               testing::Nv12Writer writer, vkc::TimelineSemaphore written,
               std::uint32_t width, std::uint32_t height)
      : gpu_(gpu),
        encoder_(std::move(encoder)),
        writer_(std::move(writer)),
        written_(std::move(written)),
        width_(width),
        height_(height) {}

  const Gpu& gpu_;
  sensor::HevcEncoder encoder_;
  testing::Nv12Writer writer_;
  vkc::TimelineSemaphore written_;
  std::uint64_t value_ = 0;
  std::uint32_t width_;
  std::uint32_t height_;
};

// Frame `f`'s colour record on track 0: its picture `unit`, or its prefix
// alone, which the decode refuses, for an empty one.
struct ColorRecord {
  stream::RecordInfo info;
  std::array<std::uint8_t, wire::kColorPrefixBytes> prefix{};
  stream::RecordPayload payload = stream::RecordPayload::of(nullptr, 0);
};

vkc::Status color_record(const sensor::SensorInfo& info,
                         const sensor::RgbdFrame& f,
                         const sensor::ColorCodes& codes,
                         const std::vector<std::uint8_t>& unit, bool key,
                         ColorRecord* out) {
  VKC_ASSIGN(out->prefix, wire::encode_color(info, f, codes));
  out->info.track = 0;
  out->info.key = key;
  out->info.pts_ns = f.timestamp_ns;
  out->info.seq = f.sequence;
  out->info.set = f.sequence;
  out->payload = stream::RecordPayload::of(
      out->prefix.data(), out->prefix.size(),
      unit.empty() ? nullptr : unit.data(), unit.size());
  return {};
}

// Frame `i` of `p` whole: its colour record, then its depth record.
vkc::Status publish(Dialler& d, const PatternFrames& p,
                    const sensor::ColorCodes& codes,
                    const sensor::HevcEncoder::CodedPicture& picture,
                    std::size_t i) {
  ColorRecord c;
  VKC_TRY(color_record(p.info, p.frames[i], codes, picture.access_unit,
                       picture.key, &c));
  VKC_TRY(d.publish(c.info, c.payload));
  Record r;
  VKC_TRY(vr_test::depth_record(p.info, p.frames[i], 1, i, &r));
  return d.publish(r);
}

// The pattern frames of `info`, each with colour of its colour camera.
PatternFrames color_frames(const sensor::SensorInfo& info, std::size_t count) {
  PatternFrames p = vr_test::pattern_frames(info, count);
  for (sensor::RgbdFrame& f : p.frames) {
    f.color.host = &kSomeColor;
    f.color.width = info.color->size.width;
    f.color.height = info.color->size.height;
    f.color_camera = *info.color;
  }
  return p;
}

RgbdStreamSensor::Options on_device(const Gpu& gpu) {
  RgbdStreamSensor::Options options;
  options.device = &gpu.device;
  options.allocator = &gpu.allocator;
  return options;
}

// Every frame `s` hands out until `count` or its end, each checked.
vkc::Result<SessionReport> check_frames(const Gpu& gpu, RgbdStreamSensor& s,
                                        std::size_t count,
                                        std::atomic<std::size_t>* taken,
                                        std::vector<std::uint64_t>* seqs) {
  VKC_ASSIGN(std::unique_ptr<SessionCheck> check,
             SessionCheck::create(gpu.device, gpu.allocator, s.info()));
  VKC_TRY(s.start());
  const auto end = vr_test::Clock::now() + std::chrono::seconds(10);
  while (seqs->size() < count && !s.exhausted() &&
         vr_test::Clock::now() < end) {
    VKC_ASSIGN(std::optional<sensor::RgbdFrame> frame, s.poll());
    if (!frame) continue;
    VKC_TRY(check->take(*frame));
    seqs->push_back(frame->sequence);
    if (taken != nullptr) taken->store(seqs->size());
  }
  return check->report(s);
}

// 12 frames, keys at 0 and 6. Frames 0-3 whole, then frame 4's colour record
// refused by the decode: the reader lets the connection go. The writer's
// next connection sends frames 6-11 from the key. Every frame handed out has
// its own picture, live and from the recording.
int test_pictures(const Gpu& gpu, const vkc::test::Scratch& scratch) {
  constexpr std::uint32_t kW = 256;
  constexpr std::uint32_t kH = 144;
  const PatternFrames p = color_frames(vr_test::phone_info(kW, kH), 12);
  vkc::Result<std::unique_ptr<PatternCoder>> coder =
      PatternCoder::create(gpu, kW, kH);
  CHECK(coder.ok());
  std::vector<sensor::HevcEncoder::CodedPicture> pictures;
  for (std::uint32_t i = 0; i < 12; ++i) {
    vkc::Result<sensor::HevcEncoder::CodedPicture> coded = coder.value()->code(
        i, i == 0 || i == 6,
        static_cast<std::int64_t>(p.frames[i].timestamp_ns));
    if (!coded.ok())
      std::fprintf(stderr, "%s\n", coded.status().message().c_str());
    CHECK(coded.ok());
    pictures.push_back(std::move(coded).value());
  }
  CHECK(pictures[0].key && pictures[6].key && !pictures[5].key);
  const sensor::ColorCodes codes = coder.value()->codes();

  const std::string recording = (scratch.path / "pictures.vrst").string();
  const std::uint16_t port = vkc::test::closed_port();
  std::atomic<std::size_t> taken{0};
  std::atomic<bool> done{false};
  bool sent = false;
  std::thread writer([&] {
    {
      auto d = Dialler::dial(vr_test::session_of(p.info), port);
      if (!d.ok()) return;
      for (std::size_t i = 0; i < 4; ++i) {
        if (!publish(**d, p, codes, pictures[i], i).ok()) return;
      }
      sensor::HevcEncoder::CodedPicture broken = pictures[4];
      broken.access_unit.clear();
      if (!publish(**d, p, codes, broken, 4).ok()) return;
      if (!(*d)->let_go()) return;  // the reader's
    }
    auto d = Dialler::dial(vr_test::session_of(p.info), port);
    if (!d.ok()) return;
    for (std::size_t i = 6; i < 12; ++i) {
      if (!publish(**d, p, codes, pictures[i], i).ok()) return;
    }
    sent = vr_test::wait_for([&] { return done.load(); });
  });
  vkc::Result<std::unique_ptr<RgbdStreamSensor>> live =
      RgbdStreamSensor::listen("127.0.0.1", port, on_device(gpu), recording,
                               std::chrono::seconds(5));
  if (!live.ok()) std::fprintf(stderr, "%s\n", live.status().message().c_str());
  CHECK(live.ok());
  std::vector<std::uint64_t> seqs;
  vkc::Result<SessionReport> report =
      check_frames(gpu, *live.value(), 10, &taken, &seqs);
  done = true;
  writer.join();
  if (!report.ok())
    std::fprintf(stderr, "%s\n", report.status().message().c_str());
  CHECK(report.ok() && sent);
  const SessionReport& r = report.value();
  std::fputs(vr_example::link::format(r).c_str(), stdout);
  CHECK((seqs == std::vector<std::uint64_t>{0, 1, 2, 3, 6, 7, 8, 9, 10, 11}));
  CHECK(r.ok() && r.own_picture == 10 && r.wrong == 0 &&
        r.no_picture_index == 0 && r.no_depth_index == 0);
  CHECK(r.stats.received == 10 && r.stats.delivered == 10 &&
        r.stats.failed == 0);
  CHECK(r.depth_spans == 1 && r.depth_lost == 2 && r.window_splits == 1);
  CHECK(r.link.connections == 2 && r.link.failures == 1 &&
        r.link.refused == 0 && r.link.recorded == 21);
  CHECK(r.latency_ms.has_value());
  CHECK(vr_example::link::format(r).find("rgbd_breach: none\n") !=
        std::string::npos);

  // The recording, read back on the device: the same frames, each with its
  // own picture.
  live.value().reset();
  vkc::Result<std::unique_ptr<RgbdStreamSensor>> file =
      RgbdStreamSensor::open_file(recording, on_device(gpu));
  CHECK(file.ok());
  std::vector<std::uint64_t> file_seqs;
  vkc::Result<SessionReport> again =
      check_frames(gpu, *file.value(), ~std::size_t{0}, nullptr, &file_seqs);
  CHECK(again.ok());
  CHECK(file_seqs == seqs);
  CHECK(again.value().ok() && again.value().own_picture == 10 &&
        !again.value().latency_ms);
  CHECK(vr_example::link::format(again.value()).find("rgbd_breach: none\n") !=
        std::string::npos);
  return 0;
}

// The hand check's sender (above): `frames` frames to `host`:`port`.
int send_to(const Gpu& gpu, const std::string& host, std::uint16_t port,
            std::uint64_t frames) {
  constexpr std::uint32_t kW = 1920;
  constexpr std::uint32_t kH = 1440;
  const sensor::SensorInfo info = vr_test::phone_info(kW, kH);
  const stream::Session session = vr_test::session_of(info);
  vkc::Result<std::unique_ptr<PatternCoder>> coder =
      PatternCoder::create(gpu, kW, kH);
  if (!coder.ok()) {
    std::fprintf(stderr, "FAIL: %s\n", coder.status().message().c_str());
    return 1;
  }
  gnet::ServerOptions options;
  options.listen = false;
  options.replay_gop = false;
  options.max_viewers = 1;
  vkc::Result<std::unique_ptr<gnet::StreamServer>> server =
      gnet::StreamServer::start(session, options);
  if (!server.ok()) return 1;
  gnet::StreamServer& s = *server.value();
  std::uint64_t wanted = 0;  // key_wanted at the last key
  std::uint64_t keys = 0;
  std::uint64_t dials = 0;
  auto next_dial = vr_test::Clock::now();
  const auto start = vr_test::Clock::now();
  for (std::uint64_t i = 0; i < frames; ++i) {
    std::this_thread::sleep_until(
        start + std::chrono::milliseconds(100 * static_cast<std::int64_t>(i)));
    if (s.viewers() == 0 && vr_test::Clock::now() >= next_dial) {
      next_dial = vr_test::Clock::now() + std::chrono::seconds(1);
      ++dials;
      vkc::Result<vkc::UniqueFd> fd =
          volumetric_kit::core::net::dial(host, port, std::chrono::seconds(1));
      if (fd.ok()) (void)s.add_viewer(std::move(fd).value());
    }
    sensor::testing::PatternDepth depth =
        testing::pattern_depth(256, 192, static_cast<std::uint32_t>(i));
    sensor::RgbdFrame f;
    f.depth = depth.depth.data();
    f.depth_confidence = depth.confidence.data();
    f.min_confidence = 2;
    f.depth_camera = *info.depth;
    f.metres_per_unit = 0.001f;
    f.min_depth = 0.25f;
    f.max_depth = 5.0f;
    f.depth_noise = {0.001f, 0.01f, 0.0f};
    f.color.host = &kSomeColor;
    f.color.width = kW;
    f.color.height = kH;
    f.color_camera = *info.color;
    f.timestamp_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
    f.sequence = i;
    const bool key = s.key_wanted() != wanted && s.draining() == 0;
    vkc::Result<sensor::HevcEncoder::CodedPicture> coded =
        coder.value()->code(static_cast<std::uint32_t>(i), key,
                            static_cast<std::int64_t>(f.timestamp_ns));
    if (!coded.ok()) {
      std::fprintf(stderr, "FAIL: %s\n", coded.status().message().c_str());
      return 1;
    }
    if (coded->key) {
      ++keys;
      wanted = s.key_wanted();
    }
    ColorRecord c;
    Record r;
    if (!color_record(info, f, coder.value()->codes(), coded->access_unit,
                      coded->key, &c)
             .ok() ||
        !s.publish(c.info, c.payload).ok() ||
        !vr_test::depth_record(info, f, 1, i, &r).ok() ||
        !s.publish(r.info, r.payload).ok()) {
      std::fprintf(stderr, "FAIL: frame %llu not published\n",
                   static_cast<unsigned long long>(i));
      return 1;
    }
  }
  s.finish();
  std::printf(
      "sent %llu frames to %s:%u: %llu keys, %llu dials, %.1f s\n",
      static_cast<unsigned long long>(frames), host.c_str(),
      static_cast<unsigned>(port), static_cast<unsigned long long>(keys),
      static_cast<unsigned long long>(dials),
      std::chrono::duration<double>(vr_test::Clock::now() - start).count());
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
      const bool promised = testing::hevc_decoder_promised();
      std::fprintf(stderr, "no hardware coder (%s); %s\n",
                   why.message().c_str(), promised ? "failing" : "skipping");
      return promised ? 1 : vkc::test::kSkipExitCode;
    }
  }
  if (const char* to = std::getenv("VR_TEST_SESSION_CHECK"); to && *to) {
    const vkc::Result<vkc::net::HostPort> address =
        vkc::net::parse_host_port(to, 0);
    const char* frames = std::getenv("VR_TEST_SESSION_FRAMES");
    const std::optional<long> count =
        frames && *frames ? vr_example::link::count_of(frames) : 300L;
    if (!address.ok() || !count) {
      const std::string why =
          address.ok() ? std::string() : "; " + address.status().message();
      std::fprintf(stderr,
                   "FAIL: VR_TEST_SESSION_CHECK=<ipv4>:<port>, "
                   "VR_TEST_SESSION_FRAMES=<count>%s\n",
                   why.c_str());
      return 1;
    }
    return send_to(gpu, address->host, address->port,
                   static_cast<std::uint64_t>(*count));
  }
  const vkc::test::Scratch scratch("recon_link_listen_color_");
  if (test_pictures(gpu, scratch) != 0) return 1;
  std::printf("rgbd stream sensor listen colour tests passed\n");
  return 0;
}

vkc::Result<vkc::DeviceRequirements> requirements() {
  return vkc::merge(sensor::device_requirements(),
                    sensor::hevc_encoder_device_requirements());
}

#else  // VR_TEST_LINK_VIDEO

int gpu_main(Gpu&) {
  std::printf("rgbd stream sensor listen colour tests: no video coders\n");
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
