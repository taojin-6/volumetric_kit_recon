// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// The phones of the lane's tests (examples/common/live_mobile.hpp): records a
// script lists, each a 32 x 24 depth at 1.5 m with a confidence plane, posed
// by the device 1 cm along x a frame from 1 s; handed out as a recorded
// session hands them (FakePhone), or as a live link does, each after its
// latency and stamped with its arrival (LivePhone).

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/sensor/camera/geometry.hpp"
#include "volumetric_kit/sensor/rgbd_sensor.hpp"

namespace vr_test {

// A fake phone's depth: 32 x 24.
inline constexpr std::uint32_t kPhoneW = 32;
inline constexpr std::uint32_t kPhoneH = 24;

// How long after its capture a live phone's record comes.
inline constexpr std::uint64_t kLatency = 50'000'000;

struct PhoneFrame {
  std::uint64_t pts = 0;
  std::uint64_t seq = 0;
  bool noise = true;  // false: states none, which the tracker refuses
};

// Frames at 10 Hz from 1 s: `count` of them, seq 0 up.
inline std::vector<PhoneFrame> ten_hz(std::size_t count) {
  std::vector<PhoneFrame> out;
  for (std::size_t i = 0; i < count; ++i) {
    out.push_back({1'000'000'000 + i * 100'000'000, i});
  }
  return out;
}

// The device's pose of the frame at `pts`: 1 cm along x a frame.
inline volumetric_kit::recon::Mat4f device_pose(std::uint64_t pts) {
  volumetric_kit::recon::Mat4f m(1.0f);
  m[3] = volumetric_kit::recon::Vec4f(
      0.01f * float((pts - 1'000'000'000) / 100'000'000), 0.0f, 0.0f, 1.0f);
  return m;
}

// A phone whose records the script lists, as a recorded session hands them
// out: at once, in order, exhausted after the last.
class FakePhone final : public volumetric_kit::sensor::IRgbdSensor {
 public:
  explicit FakePhone(std::vector<PhoneFrame> script)
      : script_(std::move(script)) {
    info_.id = "phone";
    info_.clock = volumetric_kit::sensor::ClockDomain::Host;
    info_.fps = 10;
  }
  const volumetric_kit::sensor::SensorInfo& info() const noexcept override {
    return info_;
  }
  volumetric_kit::core::Status set_queue_depth(std::size_t) override {
    return {};
  }
  volumetric_kit::core::Status start() override {
    started_ = true;
    return {};
  }
  void stop() noexcept override { started_ = false; }
  volumetric_kit::core::Result<std::optional<volumetric_kit::sensor::RgbdFrame>>
  poll() override {
    namespace sensor = volumetric_kit::sensor;
    if (!started_ || next_ >= script_.size()) {
      return std::optional<sensor::RgbdFrame>{};
    }
    const PhoneFrame& p = script_[next_++];
    struct Pixels {
      std::vector<std::uint16_t> depth;
      std::vector<std::uint8_t> confidence;
    };
    auto pixels = std::make_shared<Pixels>();
    pixels->depth.assign(std::size_t{kPhoneW} * kPhoneH, 1500);
    pixels->confidence.assign(std::size_t{kPhoneW} * kPhoneH, 2);
    sensor::RgbdFrame f;
    f.depth = pixels->depth.data();
    f.depth_confidence = pixels->confidence.data();
    f.min_confidence = 2;  // as the lane's reader raises it
    f.depth_camera = {{kPhoneW, kPhoneH}, {40.0, 40.0, 15.5, 11.5}, {}};
    f.min_depth = 0.1f;
    f.max_depth = 5.0f;
    if (p.noise) f.depth_noise = {0.002f, 0.0f, 0.002f};
    f.color_to_world = sensor::camera::Mat4d(device_pose(p.pts));
    f.timestamp_ns = p.pts;
    f.sequence = p.seq;
    f.pixels = std::move(pixels);
    return std::optional<sensor::RgbdFrame>(std::move(f));
  }
  volumetric_kit::core::Status drain(
      std::vector<volumetric_kit::sensor::RgbdFrame>* out) override {
    if (fail_at_ && next_ == *fail_at_) {
      fail_at_.reset();
      return volumetric_kit::core::Status::io_error(
          "fake phone: a damaged record");
    }
    VKC_ASSIGN(std::optional<volumetric_kit::sensor::RgbdFrame> f, poll());
    if (f) out->push_back(std::move(*f));
    return {};
  }
  // The drain before record `n` returns an error, once.
  void fail_before(std::size_t n) { fail_at_ = n; }
  bool exhausted() const noexcept override { return next_ >= script_.size(); }
  volumetric_kit::sensor::SensorStats stats() const noexcept override {
    volumetric_kit::sensor::SensorStats s{};
    s.received = next_;
    s.delivered = next_;
    return s;
  }

 private:
  std::vector<PhoneFrame> script_;
  std::size_t next_ = 0;
  std::optional<std::size_t> fail_at_;
  bool started_ = false;
  volumetric_kit::sensor::SensorInfo info_;
};

// The script's records as a live link hands them out: each comes kLatency
// after its capture, and a drain takes those come by `*now`, oldest first,
// at most `per_drain` (the rest wait in the link), each stamped with its
// arrival.
class LivePhone final : public volumetric_kit::sensor::IRgbdSensor {
 public:
  LivePhone(const std::vector<PhoneFrame>& script, const std::uint64_t* now,
            std::size_t per_drain)
      : inner_(script), script_(script), now_(now), per_drain_(per_drain) {}
  const volumetric_kit::sensor::SensorInfo& info() const noexcept override {
    return inner_.info();
  }
  volumetric_kit::core::Status set_queue_depth(std::size_t) override {
    return {};
  }
  volumetric_kit::core::Status start() override { return inner_.start(); }
  void stop() noexcept override { inner_.stop(); }
  volumetric_kit::core::Result<std::optional<volumetric_kit::sensor::RgbdFrame>>
  poll() override {
    return std::optional<volumetric_kit::sensor::RgbdFrame>{};
  }
  volumetric_kit::core::Status drain(
      std::vector<volumetric_kit::sensor::RgbdFrame>* out) override {
    for (std::size_t n = 0; n < per_drain_ && next_ < script_.size() &&
                            script_[next_].pts + kLatency <= *now_;
         ++n, ++next_) {
      VKC_ASSIGN(std::optional<volumetric_kit::sensor::RgbdFrame> f,
                 inner_.poll());
      f->arrival_ns = *now_;
      out->push_back(std::move(*f));
    }
    return {};
  }
  bool exhausted() const noexcept override { return inner_.exhausted(); }
  volumetric_kit::sensor::SensorStats stats() const noexcept override {
    return inner_.stats();
  }

 private:
  FakePhone inner_;
  std::vector<PhoneFrame> script_;
  const std::uint64_t* now_;
  std::size_t per_drain_;
  std::size_t next_ = 0;
};

}  // namespace vr_test
