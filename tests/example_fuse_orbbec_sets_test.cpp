// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// fuse_orbbec's sets (examples/common/fuse_orbbec_sets.hpp). The cut: a run
// of N frames keeps a set's first frames in the array's order, past its
// missing ones, and lets the rest go, their pictures with them. The run over
// an array of pushed fake cameras: each set waited for, counted and cut, the
// run done at exactly N frames, a set missing a secondary counted as it came,
// and each secondary's skew to the primary over every frame it sent, the
// primary's none. Host only.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "fuse_orbbec_sets.hpp"
#include "test_check.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/recon/sensor/array/sensor_array.hpp"
#include "volumetric_kit/recon/sensor/rgbd_sensor.hpp"

namespace vkc = volumetric_kit::core;
namespace sensor = volumetric_kit::recon::sensor;

namespace {

// A set with a frame from each camera `present` names, numbered by camera,
// each holding a picture `pictures` watches.
sensor::FrameSet set_of(const std::vector<bool>& present,
                        std::vector<std::weak_ptr<const void>>* pictures) {
  sensor::FrameSet set;
  set.frames.resize(present.size());
  for (std::size_t c = 0; c < present.size(); ++c) {
    if (!present[c]) continue;
    sensor::RgbdFrame frame;
    frame.sequence = c;
    auto picture = std::make_shared<int>(0);
    pictures->push_back(picture);
    frame.pixels = std::move(picture);
    set.frames[c] = std::move(frame);
  }
  return set;
}

int cut() {
  std::vector<std::weak_ptr<const void>> pictures;
  // Room for the whole set: every frame stays.
  sensor::FrameSet set = set_of({true, false, true, true}, &pictures);
  CHECK(vr_example::cut_set(set, 5) == 3 && set.count() == 3);
  CHECK(vr_example::cut_set(set, 3) == 3 && set.count() == 3);

  // The first two, in the array's order, past the missing one; the third
  // goes, and its picture with it.
  pictures.clear();
  set = set_of({true, false, true, true}, &pictures);
  CHECK(vr_example::cut_set(set, 2) == 2 && set.frames.size() == 4);
  CHECK(set.frames[0] && set.frames[0]->sequence == 0 && !set.frames[1]);
  CHECK(set.frames[2] && set.frames[2]->sequence == 2 && !set.frames[3]);
  CHECK(!pictures[0].expired() && !pictures[1].expired());
  CHECK(pictures[2].expired());

  // None left: the set keeps a slot per camera, with no frame.
  pictures.clear();
  set = set_of({true, true}, &pictures);
  CHECK(vr_example::cut_set(set, 0) == 0 && set.frames.size() == 2);
  CHECK(set.count() == 0 && pictures[0].expired() && pictures[1].expired());
  return 0;
}

// A camera on the host's clock at 1 kHz whose frames the test pushes; the
// array's next drain hands them over.
class Pushed final : public sensor::IRgbdSensor {
 public:
  Pushed(std::string id, sensor::SyncRole role) {
    info_.id = std::move(id);
    info_.role = role;
    info_.clock = sensor::ClockDomain::Host;
    info_.fps = 1000;
  }

  void push(std::uint64_t timestamp_ns) {
    sensor::RgbdFrame frame;
    frame.timestamp_ns = timestamp_ns;
    frame.sequence = timestamp_ns;
    frames_.push_back(std::move(frame));
  }

  const sensor::SensorInfo& info() const noexcept override { return info_; }
  vkc::Status set_queue_depth(std::size_t) override { return {}; }
  vkc::Status start() override { return {}; }
  void stop() noexcept override { frames_.clear(); }
  vkc::Result<std::optional<sensor::RgbdFrame>> poll() override {
    if (frames_.empty()) return no_frame();
    sensor::RgbdFrame frame = std::move(frames_.back());
    frames_.clear();
    return some_frame(std::move(frame));
  }
  vkc::Status drain(std::vector<sensor::RgbdFrame>* out) override {
    for (sensor::RgbdFrame& frame : frames_) out->push_back(std::move(frame));
    frames_.clear();
    return {};
  }
  sensor::SensorStats stats() const noexcept override { return {}; }

 private:
  sensor::SensorInfo info_;
  std::vector<sensor::RgbdFrame> frames_;
};

// A run of 4 frames from a primary "P" and its secondary "S", in that order:
// a complete set (2 frames), one missing S (1), and a complete one cut to the
// last frame the run takes, P's.
int run() {
  auto primary = std::make_unique<Pushed>("P", sensor::SyncRole::Primary);
  auto secondary = std::make_unique<Pushed>("S", sensor::SyncRole::Secondary);
  Pushed* p = primary.get();
  Pushed* s = secondary.get();
  std::vector<std::unique_ptr<sensor::IRgbdSensor>> sensors;
  sensors.push_back(std::move(primary));
  sensors.push_back(std::move(secondary));
  sensor::SensorArray::Options options;
  options.tolerance_us = 200;  // under half of 1 kHz's period
  auto opened = sensor::SensorArray::open(std::move(sensors), options);
  CHECK(opened.ok());
  sensor::SensorArray array = std::move(opened).value();
  CHECK(array.start().ok() && array.primary() == 0);

  vr_example::RigSets sets(array, 4);
  vkc::StageMetrics stages;
  CHECK(!sets.done() && sets.sets() == 0 && sets.frames() == 0);
  const std::uint64_t base = 5'000'000'000;
  // Trigger `t`: P's frame, and S's `lag_ns` after it unless S is silent.
  const auto trigger = [&](std::uint64_t t,
                           std::optional<std::int64_t> lag_ns) {
    const std::uint64_t ts = base + t * 1'000'000;  // 1 kHz
    p->push(ts);
    if (lag_ns) s->push(ts + static_cast<std::uint64_t>(*lag_ns));
  };

  trigger(0, 80'000);
  vkc::Result<sensor::FrameSet> set = sets.next(&stages);
  CHECK(set.ok() && set.value().complete());
  CHECK(!sets.done() && sets.sets() == 1 && sets.frames() == 2);

  // S silent: the set comes once the array stops waiting for it, and counts
  // the one frame it holds.
  trigger(1, std::nullopt);
  set = sets.next(&stages);
  CHECK(set.ok() && set.value().count() == 1 && set.value().frames[0]);
  CHECK(!sets.done() && sets.sets() == 2 && sets.frames() == 3);

  // One frame left to take: the set is cut to P's, the first in the array's
  // order, and the run is done at exactly 4.
  trigger(2, -120'000);
  set = sets.next(&stages);
  CHECK(set.ok() && set.value().count() == 1 && set.value().frames[0]);
  CHECK(sets.done() && sets.sets() == 3 && sets.frames() == 4);

  // S's skew over the two frames it sent, the one the cut let go included;
  // the primary has none.
  const vr_example::RigSets::Skew& lag = sets.skew(1);
  CHECK(lag.frames == 2);
  CHECK(std::fabs(lag.sum_ms - (0.08 - 0.12)) < 1e-9);
  CHECK(std::fabs(lag.worst_ms - 0.12) < 1e-9);
  CHECK(sets.skew(0).frames == 0 && sets.skew(0).sum_ms == 0.0);

  // The poll that hands a set out is its one row.
  CHECK(stages.rows().size() == 1);
  CHECK(std::string(stages.rows()[0].name) == "poll");
  return 0;
}

}  // namespace

int main() {
  CHECK(cut() == 0);
  CHECK(run() == 0);
  std::puts("example_fuse_orbbec_sets: OK");
  return 0;
}
