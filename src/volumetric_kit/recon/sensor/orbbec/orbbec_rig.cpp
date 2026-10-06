// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/sensor/orbbec/orbbec_rig.hpp"

#include <chrono>
#include <exception>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <libobsensor/ObSensor.hpp>

#include "camera_stream.hpp"
#include "frame_conversion.hpp"
#include "rig_start_order.hpp"
#include "volumetric_kit/recon/sensor/trigger_grouper.hpp"

namespace volumetric_kit::recon::sensor {

namespace {

// Frames held per camera, in its mailbox and again in the grouper: a few
// frame periods of slack, so a poll slower than the cameras still finds one
// trigger's frames from all of them rather than each camera's newest.
constexpr std::size_t kQueueDepth = 4;

std::uint64_t now_us() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

}  // namespace

struct OrbbecRig::Impl {
  // Declared first so it is destroyed last: every SDK object belongs to it.
  std::shared_ptr<ob::Context> context;
  std::vector<std::unique_ptr<orbbec::CameraStream>> streams;
  std::vector<std::size_t> start_order;  // secondaries, then the primary
  std::size_t primary = 0;
  std::uint32_t clock_sync_interval_ms = 0;
  // Built by open once the primary is known: a grouper needs its anchor.
  std::optional<TriggerGrouper> grouper;

  // Pairs taken from a stream and not yet read or discarded, by the id the
  // grouper knows them by. Declared after `streams`, so they are released
  // before the streams go.
  struct Held {
    std::size_t camera;
    std::shared_ptr<ob::FrameSet> pair;
  };
  std::unordered_map<std::uint64_t, Held> held;
  std::uint64_t next_id = 1;
  std::vector<std::uint64_t> released;  // scratch

  bool running = false;
  std::uint64_t sets = 0;
  std::uint64_t incomplete = 0;

  ~Impl() { stop_all(); }

  // The next ready trigger's frames.
  core::Result<std::optional<OrbbecRigSet>> take();

  void release_ids() {
    for (const std::uint64_t id : released) {
      const auto it = held.find(id);
      if (it == held.end()) continue;
      streams[it->second.camera]->discard();
      held.erase(it);
    }
    released.clear();
  }

  void stop_all() noexcept {
    for (auto& s : streams) s->stop();
    running = false;
    if (grouper) grouper->clear(&released);
    release_ids();
    held.clear();
  }
};

OrbbecRig::OrbbecRig(std::unique_ptr<Impl> impl) noexcept
    : impl_(std::move(impl)) {}
OrbbecRig::OrbbecRig(OrbbecRig&& other) noexcept = default;
OrbbecRig& OrbbecRig::operator=(OrbbecRig&& other) noexcept = default;
OrbbecRig::~OrbbecRig() = default;

core::Result<OrbbecRig> OrbbecRig::open(const Options& options) {
  VKC_TRY(orbbec::validate(options));
  auto impl = std::make_unique<Impl>();
  std::vector<std::string> serials;
  for (const OrbbecSyncDevice& device : options.sync.devices) {
    serials.push_back(device.serial);
  }
  try {
    if (options.configure_sdk_logging) orbbec::configure_sdk_logging();
    impl->context = std::make_shared<ob::Context>();
    impl->context->enableNetDeviceEnumeration(true);
    VKC_ASSIGN(const auto devices,
               orbbec::discover(*impl->context, serials,
                                options.discovery_timeout_ms, "OrbbecRig"));
    for (std::size_t i = 0; i < devices.size(); ++i) {
      camera::Mat4d pose(1.0);
      const camera::SensorCalibration* sensor =
          camera::find_sensor(options.calibration, serials[i]);
      if (sensor != nullptr) pose = sensor->color_to_world;
      VKC_ASSIGN(auto stream, orbbec::CameraStream::create(
                                  impl->context, devices[i], options, pose,
                                  options.configure_sdk_logging, "OrbbecRig"));
      stream->set_queue_depth(kQueueDepth);
      impl->streams.push_back(std::move(stream));
    }
    // The sync configuration is the rig's hardware state: compare each camera
    // with it, and write it only where it differs, only when asked.
    std::string differences;
    for (std::size_t i = 0; i < devices.size(); ++i) {
      const OrbbecSyncSettings& wanted = options.sync.devices[i].sync;
      auto diff =
          orbbec::sync_differences(wanted, impl->streams[i]->sync_settings());
      if (!diff.empty() && options.apply_sync_config) {
        VKC_TRY(impl->streams[i]->apply_sync(wanted));
        diff =
            orbbec::sync_differences(wanted, impl->streams[i]->sync_settings());
      }
      if (diff.empty()) continue;
      differences += (differences.empty() ? "" : "; ") + serials[i] + " ";
      for (std::size_t k = 0; k < diff.size(); ++k) {
        differences += (k == 0 ? "" : ", ") + diff[k];
      }
    }
    if (!differences.empty()) {
      return core::Status::unsupported(
          "OrbbecRig: cameras differ from the sync configuration: " +
          differences +
          (options.apply_sync_config
               ? std::string("; they did not keep it when written")
               : std::string("; set apply_sync_config to write it")));
    }
    std::vector<OrbbecSyncMode> modes;
    for (const auto& stream : impl->streams) {
      modes.push_back(stream->info().sync_mode);
    }
    VKC_ASSIGN(impl->start_order, orbbec::rig_start_order(modes, serials));
  } catch (const std::exception& e) {  // ob::Error is one
    return orbbec::sdk_error("OrbbecRig", "opening the rig", e);
  }
  impl->primary = impl->start_order.back();
  impl->clock_sync_interval_ms = options.clock_sync_interval_ms;
  TriggerGrouper::Config grouping;
  grouping.cameras = options.sync.devices.size();
  grouping.anchor = impl->primary;
  grouping.tolerance_us = options.sync_tolerance_us;
  // A frame and a half: long enough for a late camera's frame to arrive,
  // short enough that a silent one costs one set, not several.
  grouping.max_wait_us = 1500000u / options.fps;
  grouping.queue_depth = kQueueDepth;
  impl->grouper.emplace(grouping);
  return OrbbecRig(std::move(impl));
}

std::size_t OrbbecRig::camera_count() const noexcept {
  return impl_ != nullptr ? impl_->streams.size() : 0;
}

const OrbbecDeviceInfo& OrbbecRig::device_info(std::size_t i) const noexcept {
  static const OrbbecDeviceInfo kEmpty{};
  return i < camera_count() ? impl_->streams[i]->info() : kEmpty;
}

const camera::CameraModel& OrbbecRig::color_camera(
    std::size_t i) const noexcept {
  static const camera::CameraModel kEmpty{};
  return i < camera_count() ? impl_->streams[i]->color_camera() : kEmpty;
}

const camera::Mat4d& OrbbecRig::color_to_world(std::size_t i) const noexcept {
  static const camera::Mat4d kIdentity(1.0);
  return i < camera_count() ? impl_->streams[i]->color_to_world() : kIdentity;
}

std::size_t OrbbecRig::primary() const noexcept {
  return impl_ != nullptr ? impl_->primary : 0;
}

OrbbecRigStats OrbbecRig::stats() const {
  OrbbecRigStats s;
  if (impl_ == nullptr) return s;
  s.sets = impl_->sets;
  s.incomplete = impl_->incomplete;
  for (const auto& stream : impl_->streams) {
    s.cameras.push_back(stream->stats());
  }
  return s;
}

bool OrbbecRig::exhausted() const noexcept {
  if (impl_ == nullptr) return true;
  for (const auto& s : impl_->streams) {
    if (s->disconnected()) return true;
  }
  return false;
}

core::Status OrbbecRig::start() {
  if (impl_ == nullptr) {
    return core::Status::invalid_argument(
        "OrbbecRig: start on a moved-from rig");
  }
  Impl& r = *impl_;
  if (r.running) {
    // Not "already started" once a camera has gone away: a running stream's
    // start() is OK, and a disconnected one's names the camera.
    for (const auto& stream : r.streams) {
      const core::Status started = stream->start();
      if (!started.ok()) {
        r.stop_all();
        return started;
      }
    }
    return {};
  }
  try {
    // Before any camera starts, so the first trigger is already on one clock.
    r.context->enableDeviceClockSync(r.clock_sync_interval_ms);
  } catch (const std::exception& e) {  // ob::Error is one
    return orbbec::sdk_error("OrbbecRig", "syncing the cameras' clocks", e);
  }
  for (const std::size_t i : r.start_order) {
    const core::Status started = r.streams[i]->start();
    if (!started.ok()) {
      r.stop_all();
      return started;
    }
  }
  r.sets = 0;
  r.incomplete = 0;
  r.running = true;
  return {};
}

void OrbbecRig::stop() noexcept {
  if (impl_ != nullptr) impl_->stop_all();
}

core::Result<std::optional<OrbbecRigSet>> OrbbecRig::poll_set() {
  if (impl_ == nullptr) {
    return core::Status::invalid_argument(
        "OrbbecRig: poll on a moved-from rig");
  }
  return impl_->take();
}

core::Result<std::optional<OrbbecRigSet>> OrbbecRig::Impl::take() {
  Impl& r = *this;
  if (!r.running) return std::optional<OrbbecRigSet>{};
  const std::uint64_t now = now_us();
  std::vector<std::shared_ptr<ob::FrameSet>> pairs;
  for (std::size_t c = 0; c < r.streams.size(); ++c) {
    pairs.clear();
    VKC_TRY(r.streams[c]->take_all(&pairs));
    for (auto& pair : pairs) {
      const std::uint64_t ts = orbbec::CameraStream::timestamp_us(*pair);
      if (ts == 0) {  // no clock to group it by
        r.streams[c]->discard();
        continue;
      }
      const std::uint64_t id = r.next_id++;
      r.held.emplace(id, Impl::Held{c, std::move(pair)});
      r.grouper->add(c, ts, id, now, &r.released);
    }
  }
  r.release_ids();

  // Every set holds the primary's frame, so the secondaries' frames of the
  // triggers before the primary's first frame reaches the host (~0.5 s of
  // them) never make one.
  const std::optional<TriggerGrouper::Group> group =
      r.grouper->take(now, &r.released);
  r.release_ids();
  if (!group) return std::optional<OrbbecRigSet>{};

  // A frame only holds its pair, so reading one costs no more than the
  // grouping; the per-camera work is the GPU pass's, which prepare_batch runs
  // in one batch.
  OrbbecRigSet set;
  set.timestamp_ns = group->timestamp_us * 1000;
  set.frames.resize(r.streams.size());
  core::Status failure;
  for (std::size_t c = 0; c < r.streams.size(); ++c) {
    if (!group->ids[c]) continue;
    const auto it = r.held.find(*group->ids[c]);
    std::shared_ptr<ob::FrameSet> pair = std::move(it->second.pair);
    r.held.erase(it);
    if (!failure.ok()) {  // an earlier camera failed; account for the rest
      r.streams[c]->discard();
      continue;
    }
    auto frame = r.streams[c]->read(pair);
    if (!frame.ok()) {
      failure = frame.status();
      continue;
    }
    set.frames[c] = std::move(frame).value();  // empty if the SDK failed
  }
  if (!failure.ok()) {
    // The set is not handed out, so neither are the frames read for it.
    for (std::size_t c = 0; c < r.streams.size(); ++c) {
      if (set.frames[c]) r.streams[c]->withdraw();
    }
    return failure;
  }
  ++r.sets;
  if (!set.complete()) ++r.incomplete;
  return std::optional<OrbbecRigSet>{std::move(set)};
}

}  // namespace volumetric_kit::recon::sensor
