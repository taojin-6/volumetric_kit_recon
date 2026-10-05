// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/sensor/orbbec/orbbec_rig.hpp"

#include <chrono>
#include <exception>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <libobsensor/ObSensor.hpp>

#include "camera_stream.hpp"
#include "frame_conversion.hpp"
#include "trigger_grouping.hpp"

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
  orbbec::TriggerGrouper grouper{orbbec::TriggerGrouper::Config{}};

  // Pairs taken from a stream and not yet processed or discarded, by the id
  // the grouper knows them by. Declared after `streams`, so they are released
  // before the streams go.
  struct Held {
    std::size_t camera;
    std::shared_ptr<ob::FrameSet> pair;
  };
  std::unordered_map<std::uint64_t, Held> held;
  std::uint64_t next_id = 1;
  std::vector<std::uint64_t> released;  // scratch

  bool running = false;
  // Which of the four polls this start's caller reads with. The others are
  // refused: a set one took would drop the frames another had left. A raw rig
  // reads only the raw two, a host rig only the others.
  enum class Reader {
    Unchosen,
    Sets,
    Frames,
    RawSets,
    RawFrames
  } reader = Reader::Unchosen;
  bool raw = false;
  std::uint64_t sets = 0;
  std::uint64_t incomplete = 0;
  // The set poll() or poll_raw() is handing out, and the next frame of it.
  OrbbecRigFrameSet current;
  OrbbecRigRawSet current_raw;
  std::size_t cursor = 0;

  ~Impl() { stop_all(); }

  core::Status read_with(Reader wanted) {
    const bool raw_reader =
        wanted == Reader::RawSets || wanted == Reader::RawFrames;
    if (raw != raw_reader) {
      return core::Status::invalid_argument(
          raw ? "OrbbecRig: opened for raw frames; take them with poll_raw_set "
                "or poll_raw"
              : "OrbbecRig: poll_raw_set and poll_raw need a rig opened with "
                "raw = true");
    }
    if (reader == Reader::Unchosen) reader = wanted;
    if (reader == wanted) return {};
    return core::Status::invalid_argument(
        "OrbbecRig: read with one of the set and frame polls until the next "
        "start");
  }

  // The next ready trigger's frames, each processed, or read raw.
  template <typename Frame>
  core::Result<std::optional<OrbbecRigSet<Frame>>> take();

  // The next frame of `set`, taking a new set when it is spent.
  template <typename Frame>
  core::Result<std::optional<Frame>> next(OrbbecRigSet<Frame>& set);

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
    grouper.clear(&released);
    release_ids();
    held.clear();
    current = OrbbecRigFrameSet{};
    current_raw = OrbbecRigRawSet{};
    cursor = 0;
  }
};

OrbbecRig::OrbbecRig(std::unique_ptr<Impl> impl) noexcept
    : impl_(std::move(impl)) {}
OrbbecRig::OrbbecRig(OrbbecRig&& other) noexcept = default;
OrbbecRig& OrbbecRig::operator=(OrbbecRig&& other) noexcept = default;
OrbbecRig::~OrbbecRig() = default;

core::Result<OrbbecRig> OrbbecRig::open(const Options& options) {
  VKC_TRY(orbbec::validate(options));
  VKC_TRY(orbbec::check_color_codec(options, "OrbbecRig"));
  auto impl = std::make_unique<Impl>();
  impl->raw = options.raw;
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
      Mat4f pose(1.0f);
      for (const RigCameraCalibration& c : options.calibration) {
        if (c.serial == serials[i]) pose = c.cam_to_world;
      }
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
  orbbec::TriggerGrouper::Config grouping;
  grouping.cameras = options.sync.devices.size();
  grouping.anchor = impl->primary;
  grouping.tolerance_us = options.sync_tolerance_us;
  // A frame and a half: long enough for a late camera's frame to arrive,
  // short enough that a silent one costs one set, not several.
  grouping.max_wait_us = 1500000u / options.fps;
  grouping.queue_depth = kQueueDepth;
  impl->grouper = orbbec::TriggerGrouper(grouping);
  return OrbbecRig(std::move(impl));
}

std::size_t OrbbecRig::camera_count() const noexcept {
  return impl_ != nullptr ? impl_->streams.size() : 0;
}

const OrbbecDeviceInfo& OrbbecRig::device_info(std::size_t i) const noexcept {
  static const OrbbecDeviceInfo kEmpty{};
  return i < camera_count() ? impl_->streams[i]->info() : kEmpty;
}

const ColorCameraParams& OrbbecRig::color_camera(std::size_t i) const noexcept {
  static const ColorCameraParams kEmpty{};
  return i < camera_count() ? impl_->streams[i]->color_camera() : kEmpty;
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
  r.reader = Impl::Reader::Unchosen;
  r.sets = 0;
  r.incomplete = 0;
  r.running = true;
  return {};
}

void OrbbecRig::stop() noexcept {
  if (impl_ != nullptr) impl_->stop_all();
}

core::Result<std::optional<OrbbecRigFrameSet>> OrbbecRig::poll_set() {
  if (impl_ == nullptr) {
    return core::Status::invalid_argument(
        "OrbbecRig: poll on a moved-from rig");
  }
  VKC_TRY(impl_->read_with(Impl::Reader::Sets));
  return impl_->take<CapturedFrame>();
}

core::Result<std::optional<OrbbecRigRawSet>> OrbbecRig::poll_raw_set() {
  if (impl_ == nullptr) {
    return core::Status::invalid_argument(
        "OrbbecRig: poll on a moved-from rig");
  }
  VKC_TRY(impl_->read_with(Impl::Reader::RawSets));
  return impl_->take<RawFrame>();
}

template <typename Frame>
core::Result<std::optional<OrbbecRigSet<Frame>>> OrbbecRig::Impl::take() {
  using Set = OrbbecRigSet<Frame>;
  Impl& r = *this;
  if (!r.running) return std::optional<Set>{};
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
      r.grouper.add(c, ts, id, now, &r.released);
    }
  }
  r.release_ids();

  // Every set holds the primary's frame, so the secondaries' frames of the
  // triggers before the primary's first frame reaches the host (~0.5 s of
  // them) never make one.
  const std::optional<orbbec::TriggerGrouper::Group> group =
      r.grouper.take(now, &r.released);
  r.release_ids();
  if (!group) return std::optional<Set>{};

  // TODO(sensor): process a host set's frames in parallel, one thread per
  // camera; one after another they take ~11 ms for four (the 2026-09-27
  // decision). A raw set's are views, and prepare_set runs their GPU passes
  // in parallel.
  Set set;
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
    // A raw frame is a view of the camera's own buffers, so it costs no more
    // than the grouping; the per-camera work is the GPU pass's.
    auto processed = [&] {
      if constexpr (std::is_same_v<Frame, RawFrame>) {
        return r.streams[c]->process_raw(pair);
      } else {
        return r.streams[c]->process(pair);
      }
    }();
    if (!processed.ok()) {
      failure = processed.status();
      continue;
    }
    set.frames[c] = processed.value();  // empty when the SDK failed on it
  }
  if (!failure.ok()) {
    // The set is not handed out, so neither are the frames processed for it.
    for (std::size_t c = 0; c < r.streams.size(); ++c) {
      if (set.frames[c]) r.streams[c]->withdraw();
    }
    return failure;
  }
  ++r.sets;
  if (!set.complete()) ++r.incomplete;
  return std::optional<Set>{std::move(set)};
}

template <typename Frame>
core::Result<std::optional<Frame>> OrbbecRig::Impl::next(
    OrbbecRigSet<Frame>& set) {
  for (;;) {
    while (cursor < set.frames.size()) {
      const std::optional<Frame>& frame = set.frames[cursor++];
      if (frame) return frame;
    }
    VKC_ASSIGN(std::optional<OrbbecRigSet<Frame>> taken, take<Frame>());
    if (!taken) return std::optional<Frame>{};
    set = std::move(*taken);
    cursor = 0;
  }
}

core::Result<std::optional<CapturedFrame>> OrbbecRig::poll() {
  if (impl_ == nullptr) {
    return core::Status::invalid_argument(
        "OrbbecRig: poll on a moved-from rig");
  }
  VKC_TRY(impl_->read_with(Impl::Reader::Frames));
  return impl_->next(impl_->current);
}

core::Result<std::optional<RawFrame>> OrbbecRig::poll_raw() {
  if (impl_ == nullptr) {
    return core::Status::invalid_argument(
        "OrbbecRig: poll on a moved-from rig");
  }
  VKC_TRY(impl_->read_with(Impl::Reader::RawFrames));
  return impl_->next(impl_->current_raw);
}

bool OrbbecRig::raw_frames() const noexcept {
  return impl_ != nullptr && impl_->raw;
}

}  // namespace volumetric_kit::recon::sensor
