// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// The link tests' pictures, both sides (with link_dialler.hpp's writer): the
// family's test pattern of each frame's index, coded by sensor's HevcEncoder
// on this machine's hardware (VideoToolbox, NVENC) from targets written on the
// device, as a producer's own kernel writes one; a frame's colour record and
// its whole frame on a Dialler; and every frame a sensor hands out checked by
// session_check's checks (tools/session_check/session_check.hpp), which read
// both indices on the device. Needs sensor's video coders
// (sensor_video_testing) and vr_session_check.

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "link/rgbd_stream_sensor.hpp"
#include "link_dialler.hpp"
#include "session_check.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/testing/gpu_test.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/core/vulkan/sync.hpp"
#include "volumetric_kit/sensor/testing/nv12_pattern.hpp"
#include "volumetric_kit/sensor/testing/nv12_writer.hpp"
#include "volumetric_kit/sensor/video/color_codes.hpp"
#include "volumetric_kit/sensor/video/hevc_encoder.hpp"
#include "volumetric_kit/sensor/wire/rgbd_wire.hpp"

namespace vr_test {

// What a frame's colour is to wire::encode_color, which asks only that it
// has one: the pictures are coded apart.
inline constexpr std::uint8_t kSomeColor = 0;

// As an iPad's pictures are tagged: BT.601's matrix, BT.709's transfer and
// primaries, full range, sited left.
inline constexpr sensor::ColorCodes kCodes{6, 1, 1, true, 0};

// The test pattern's pictures coded by sensor's HevcEncoder, each written
// into its target on the device (Nv12Writer), as a producer's own kernel
// writes one, and coded before the next is sent.
class PatternCoder {
 public:
  using Gpu = vkc::test::GpuContext;

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
    VKC_ASSIGN(sensor::testing::Nv12Writer writer,
               sensor::testing::Nv12Writer::create(gpu.device));
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
    VKC_TRY(writer_.record_upload(
        batch, gpu_.allocator,
        sensor::testing::test_pattern(width_, height_, index), target));
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
               sensor::testing::Nv12Writer writer,
               vkc::TimelineSemaphore written, std::uint32_t width,
               std::uint32_t height)
      : gpu_(gpu),
        encoder_(std::move(encoder)),
        writer_(std::move(writer)),
        written_(std::move(written)),
        width_(width),
        height_(height) {}

  const Gpu& gpu_;
  sensor::HevcEncoder encoder_;
  sensor::testing::Nv12Writer writer_;
  vkc::TimelineSemaphore written_;
  std::uint64_t value_ = 0;
  std::uint32_t width_;
  std::uint32_t height_;
};

// Frame `f`'s colour record on track 0: its picture `unit`, or its prefix
// alone, which the decode refuses, for an empty one.
struct ColorRecord {
  stream::RecordInfo info;
  std::array<std::uint8_t, sensor::wire::kColorPrefixBytes> prefix{};
  stream::RecordPayload payload = stream::RecordPayload::of(nullptr, 0);
};

inline vkc::Status color_record(const sensor::SensorInfo& info,
                                const sensor::RgbdFrame& f,
                                const sensor::ColorCodes& codes,
                                const std::vector<std::uint8_t>& unit, bool key,
                                ColorRecord* out) {
  VKC_ASSIGN(out->prefix, sensor::wire::encode_color(info, f, codes));
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
inline vkc::Status publish_frame(
    Dialler& d, const PatternFrames& p, const sensor::ColorCodes& codes,
    const sensor::HevcEncoder::CodedPicture& picture, std::size_t i) {
  ColorRecord c;
  VKC_TRY(color_record(p.info, p.frames[i], codes, picture.access_unit,
                       picture.key, &c));
  VKC_TRY(d.publish(c.info, c.payload));
  Record r;
  VKC_TRY(depth_record(p.info, p.frames[i], 1, i, &r));
  return d.publish(r);
}

// The pattern frames of `info`, each with colour of its colour camera.
inline PatternFrames color_frames(const sensor::SensorInfo& info,
                                  std::size_t count) {
  PatternFrames p = pattern_frames(info, count);
  for (sensor::RgbdFrame& f : p.frames) {
    f.color.host = &kSomeColor;
    f.color.width = info.color->size.width;
    f.color.height = info.color->size.height;
    f.color_camera = *info.color;
  }
  return p;
}

// A sensor's options with its pictures decoded on `gpu`'s device.
inline vr_example::link::RgbdStreamSensor::Options on_device(
    const vkc::test::GpuContext& gpu) {
  vr_example::link::RgbdStreamSensor::Options options;
  options.device = &gpu.device;
  options.allocator = &gpu.allocator;
  return options;
}

// Every frame `s` hands out until `count` or its end, each checked; `taken`,
// if given, follows their count.
inline vkc::Result<vr_example::link::SessionReport> check_frames(
    const vkc::test::GpuContext& gpu, vr_example::link::RgbdStreamSensor& s,
    std::size_t count, std::atomic<std::size_t>* taken,
    std::vector<std::uint64_t>* seqs) {
  VKC_ASSIGN(std::unique_ptr<vr_example::link::SessionCheck> check,
             vr_example::link::SessionCheck::create(gpu.device, gpu.allocator,
                                                    s.info()));
  VKC_TRY(s.start());
  const auto end = Clock::now() + std::chrono::seconds(10);
  while (seqs->size() < count && !s.exhausted() && Clock::now() < end) {
    VKC_ASSIGN(std::optional<sensor::RgbdFrame> frame, s.poll());
    if (!frame) continue;
    VKC_TRY(check->take(*frame));
    seqs->push_back(frame->sequence);
    if (taken != nullptr) taken->store(seqs->size());
  }
  return check->report(s);
}

}  // namespace vr_test
