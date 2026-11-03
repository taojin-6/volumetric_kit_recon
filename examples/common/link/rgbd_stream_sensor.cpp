// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "link/rgbd_stream_sensor.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <new>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "volumetric_kit/core/base/check.hpp"
#include "volumetric_kit/grid/stream/session.hpp"
#include "volumetric_kit/grid/stream/stream_reader.hpp"
#include "volumetric_kit/sensor/wire/rgbd_wire.hpp"

namespace vr_example::link {
namespace {

namespace stream = volumetric_kit::grid::stream;
namespace wire = volumetric_kit::sensor::wire;
using Code = vkc::Status::Code;
using FrameResult = vkc::Result<std::optional<sensor::RgbdFrame>>;

// A track's rate in frames a second, rounded; 0 when unknown.
std::uint32_t frames_per_second(stream::Rate rate) noexcept {
  if (rate.num == 0 || rate.den == 0) return 0;
  return static_cast<std::uint32_t>((std::uint64_t{rate.num} + rate.den / 2) /
                                    rate.den);
}

// A session id as a registration file writes it: 32 lowercase hex digits,
// byte 0 first.
std::string hex(const std::array<std::uint8_t, 16>& id) {
  constexpr char kDigits[] = "0123456789abcdef";
  std::string out;
  for (const std::uint8_t byte : id) {
    out += kDigits[byte >> 4];
    out += kDigits[byte & 0xf];
  }
  return out;
}

}  // namespace

struct RgbdStreamSensor::State {
  std::unique_ptr<stream::StreamReader> reader;
  std::uint16_t track = 0;  // the depth track's index
  sensor::SensorInfo info;
  Options options;
  bool running = false;
  bool at_end = false;
  stream::Position position;  // the next poll's
  // The last frame handed out since start, once there is one.
  bool handed_out = false;
  std::uint64_t last_sequence = 0;
  std::uint64_t last_time = 0;  // the last non-zero timestamp; 0 before one
  sensor::SensorStats stats;
  std::vector<Skipped> skipped;

  // The frame `record` of the depth track gives, or none for a record that
  // fails its CRC, the decode, or the order of the frames before it.
  FrameResult take(const stream::Record& record);

  // The file has ended: a finished file's footer counts the records its
  // writer wrote, so those past the last frame handed out are a span.
  void end();
};

vkc::Result<std::unique_ptr<RgbdStreamSensor>> RgbdStreamSensor::open_file(
    const std::string& path, Options options) try {
  constexpr const char* kApi = "RgbdStreamSensor::open_file";
  vkc::Result<std::unique_ptr<stream::StreamReader>> opened =
      stream::StreamReader::open(path);
  if (!opened.ok()) return opened.status().with_context(kApi);
  std::unique_ptr<stream::StreamReader> reader = std::move(opened).value();
  const std::string where = std::string(kApi) + ": " + path;
  const auto bad = [&](const std::string& why) {
    return vkc::Status::invalid_argument(where + ": " + why);
  };

  // The sensor's depth track: the session's only one, or the one named.
  const stream::Session& session = reader->session();
  std::optional<std::uint16_t> found;
  bool any = false;
  for (std::size_t i = 0; i < session.tracks.size(); ++i) {
    const stream::Track& track = session.tracks[i];
    if (track.kind != stream::TrackKind::RgbdDepth) continue;
    any = true;
    if (!options.sensor.empty() && track.sensor != options.sensor) continue;
    if (found) {
      return bad(options.sensor.empty()
                     ? "depth tracks of several sensors: name one "
                       "(Options::sensor)"
                     : "several depth tracks of sensor " + options.sensor);
    }
    found = static_cast<std::uint16_t>(i);
  }
  if (!any) return bad("no RgbdDepth track");
  if (!found) return bad("no depth track of sensor " + options.sensor);
  const stream::Track& track = session.tracks[*found];

  // What the sensor is: the params, with the track's id, clock and rate. Only
  // the depth track is read, so the sensor names no colour camera.
  vkc::Result<sensor::SensorInfo> decoded = wire::decode_sensor(
      track.params, track.sensor,
      track.clock == stream::kHostClock ? sensor::ClockDomain::Host
                                        : sensor::ClockDomain::Device,
      frames_per_second(track.rate));
  if (!decoded.ok()) {
    return decoded.status().with_context(where + ": track " +
                                         std::to_string(*found));
  }
  sensor::SensorInfo info = std::move(decoded).value();
  info.color.reset();

  if (options.registration) {
    const sensor::camera::TrackedRegistration& reg = *options.registration;
    const vkc::Status valid =
        sensor::camera::validate_tracked_registration(reg);
    if (!valid.ok()) return valid.with_context(where + ": the registration");
    if (info.pose != sensor::PoseSource::Tracked) {
      return bad("a registration for sensor " + info.id +
                 ", which is fixed: its poses are its calibration's");
    }
    if (reg.sensor != info.id) {
      return bad("the registration is sensor " + reg.sensor + "'s, not " +
                 info.id + "'s");
    }
    if (reg.session != session.id) {
      return bad("the registration is another session's: " + hex(reg.session) +
                 ", not the file's " + hex(session.id));
    }
    if (reg.world_epoch != 0) {
      return bad("the registration holds for world epoch " +
                 std::to_string(reg.world_epoch) +
                 ", and a session is one world, epoch 0");
    }
  }

  auto state = std::make_unique<State>();
  state->position = reader->begin();
  state->reader = std::move(reader);
  state->track = *found;
  state->info = std::move(info);
  state->options = std::move(options);
  return std::unique_ptr<RgbdStreamSensor>(
      new RgbdStreamSensor(std::move(state)));
} catch (const std::bad_alloc&) {
  return vkc::Status::out_of_memory("RgbdStreamSensor::open_file");
}

RgbdStreamSensor::RgbdStreamSensor(std::unique_ptr<State> state)
    : state_(std::move(state)) {}

RgbdStreamSensor::~RgbdStreamSensor() = default;

const sensor::SensorInfo& RgbdStreamSensor::info() const noexcept {
  return state_->info;
}

bool RgbdStreamSensor::exhausted() const noexcept { return state_->at_end; }

sensor::SensorStats RgbdStreamSensor::stats() const noexcept {
  return state_->stats;
}

const std::vector<RgbdStreamSensor::Skipped>& RgbdStreamSensor::skipped()
    const noexcept {
  return state_->skipped;
}

vkc::Status RgbdStreamSensor::set_queue_depth(std::size_t frames) {
  if (frames == 0 || state_->running) {
    return vkc::Status::invalid_argument(
        "RgbdStreamSensor::set_queue_depth: at least 1, before start");
  }
  return {};
}

vkc::Status RgbdStreamSensor::start() {
  State& s = *state_;
  if (!s.running) {
    s.running = true;
    s.handed_out = false;
    s.last_sequence = 0;
    s.last_time = 0;
    s.stats = {};
    s.skipped.clear();
  }
  return {};
}

void RgbdStreamSensor::stop() noexcept {
  State& s = *state_;
  s.running = false;
  s.at_end = false;
  s.position = s.reader->begin();
}

FrameResult RgbdStreamSensor::poll() try {
  State& s = *state_;
  if (!s.running || s.at_end) return no_frame();
  for (;;) {
    const stream::Position before = s.position;
    stream::Record record;
    const vkc::Result<bool> more = s.reader->next(&s.position, &record);
    if (!more.ok()) {
      // An error that leaves the reader where it was, such as a read past
      // the end of a file shortened since it opened, would come again at
      // every poll: the file ends there.
      if (s.position.offset <= before.offset) {
        s.end();
        return more.status();
      }
      // Damage, or a record of a later writer, which the reader is now past:
      // what it held is lost, and the next frame's sequence says so.
      const Code code = more.status().domain();
      if (code == Code::InvalidArgument || code == Code::Unsupported) continue;
      return more.status();
    }
    if (!more.value()) {
      s.end();
      return no_frame();
    }
    if (record.info.track != s.track) continue;
    ++s.stats.received;
    FrameResult frame = s.take(record);
    if (!frame.ok() || !frame.value()) {
      ++s.stats.failed;
      if (!frame.ok()) return frame;
      continue;
    }
    ++s.stats.delivered;
    return frame;
  }
} catch (const std::bad_alloc&) {
  return vkc::Status::out_of_memory("RgbdStreamSensor::poll");
}

FrameResult RgbdStreamSensor::State::take(const stream::Record& record) {
  const stream::RecordInfo& at = record.info;
  // A record out of order would break the contract's sequence or times.
  if (handed_out &&
      (at.seq <= last_sequence ||
       (at.pts_ns != 0 && last_time != 0 && at.pts_ns <= last_time))) {
    return no_frame();
  }
  // The frame holds its payload, which its depth and plane point into.
  auto payload = std::make_shared<std::vector<std::uint8_t>>();
  const vkc::Status read = reader->read(record, payload.get());
  if (!read.ok()) {
    if (read.domain() == Code::InvalidArgument) return no_frame();  // its CRC
    return read;
  }
  vkc::Result<sensor::RgbdFrame> decoded =
      wire::decode_depth(info, payload->data(), payload->size(), payload);
  if (!decoded.ok()) {
    if (decoded.status().domain() == Code::InvalidArgument) return no_frame();
    return decoded.status();
  }
  sensor::RgbdFrame frame = std::move(decoded).value();
  frame.timestamp_ns = at.pts_ns;
  frame.sequence = at.seq;
  if (frame.depth_confidence != nullptr &&
      frame.min_confidence < options.min_confidence) {
    frame.min_confidence = options.min_confidence;
  }
  if (options.registration) {
    frame.color_to_world =
        options.registration->tracking_to_world * frame.color_to_world;
  }
  // The records missing before it: after the last frame's, or from seq 0,
  // where the writer starts.
  const std::uint64_t expected = handed_out ? last_sequence + 1 : 0;
  if (at.seq > expected) {
    skipped.push_back({expected, at.seq - expected, last_time, at.pts_ns});
  }
  handed_out = true;
  last_sequence = at.seq;
  if (at.pts_ns != 0) last_time = at.pts_ns;
  return some_frame(std::move(frame));
}

void RgbdStreamSensor::State::end() {
  at_end = true;
  if (!reader->recovery().finished || track >= reader->stats().size()) return;
  const std::uint64_t written = reader->stats()[track].records;
  const std::uint64_t expected = handed_out ? last_sequence + 1 : 0;
  if (written > expected) {
    skipped.push_back({expected, written - expected, last_time, 0});
  }
}

vkc::Status RgbdStreamSensor::drain(std::vector<sensor::RgbdFrame>* out) {
  VKC_CHECK(out != nullptr,
            "RgbdStreamSensor::drain needs somewhere to put it");
  VKC_ASSIGN(std::optional<sensor::RgbdFrame> frame, poll());
  if (frame) {
    try {
      out->push_back(std::move(*frame));
    } catch (const std::bad_alloc&) {
      return vkc::Status::out_of_memory("RgbdStreamSensor::drain");
    }
  }
  return {};
}

}  // namespace vr_example::link
