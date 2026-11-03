// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "link/rgbd_stream_sensor.hpp"

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

}  // namespace

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
      return bad("the registration is another session's");
    }
    if (reg.world_epoch != 0) {
      return bad("the registration holds for world epoch " +
                 std::to_string(reg.world_epoch) +
                 ", and a session is one world, epoch 0");
    }
  }

  std::unique_ptr<RgbdStreamSensor> out(new RgbdStreamSensor());
  out->position_ = reader->begin();
  out->reader_ = std::move(reader);
  out->track_ = *found;
  out->info_ = std::move(info);
  out->options_ = std::move(options);
  return out;
} catch (const std::bad_alloc&) {
  return vkc::Status::out_of_memory("RgbdStreamSensor::open_file");
}

vkc::Status RgbdStreamSensor::set_queue_depth(std::size_t frames) {
  if (frames == 0 || running_) {
    return vkc::Status::invalid_argument(
        "RgbdStreamSensor::set_queue_depth: at least 1, before start");
  }
  return {};
}

vkc::Status RgbdStreamSensor::start() {
  if (!running_) {
    running_ = true;
    handed_out_ = false;
    last_sequence_ = 0;
    last_time_ = 0;
    stats_ = {};
    skipped_.clear();
  }
  return {};
}

void RgbdStreamSensor::stop() noexcept {
  running_ = false;
  at_end_ = false;
  position_ = reader_->begin();
}

FrameResult RgbdStreamSensor::poll() try {
  if (!running_ || at_end_) return no_frame();
  for (;;) {
    const stream::Position before = position_;
    stream::Record record;
    const vkc::Result<bool> more = reader_->next(&position_, &record);
    if (!more.ok()) {
      // Damage, or a record of a later writer, which the reader is now past:
      // what it held is lost, and the next frame's sequence says so.
      const Code code = more.status().domain();
      if ((code == Code::InvalidArgument || code == Code::Unsupported) &&
          position_.offset > before.offset) {
        continue;
      }
      return more.status();
    }
    if (!more.value()) {
      at_end_ = true;
      return no_frame();
    }
    if (record.info.track != track_) continue;
    ++stats_.received;
    FrameResult frame = take(record);
    if (!frame.ok() || !frame.value()) {
      ++stats_.failed;
      if (!frame.ok()) return frame;
      continue;
    }
    ++stats_.delivered;
    return frame;
  }
} catch (const std::bad_alloc&) {
  return vkc::Status::out_of_memory("RgbdStreamSensor::poll");
}

FrameResult RgbdStreamSensor::take(const stream::Record& record) {
  const stream::RecordInfo& at = record.info;
  // A record out of order would break the contract's sequence or times.
  if (handed_out_ &&
      (at.seq <= last_sequence_ ||
       (at.pts_ns != 0 && last_time_ != 0 && at.pts_ns <= last_time_))) {
    return no_frame();
  }
  // The frame holds its payload, which its depth and plane point into.
  auto payload = std::make_shared<std::vector<std::uint8_t>>();
  const vkc::Status read = reader_->read(record, payload.get());
  if (!read.ok()) {
    if (read.domain() == Code::InvalidArgument) return no_frame();  // its CRC
    return read;
  }
  vkc::Result<sensor::RgbdFrame> decoded =
      wire::decode_depth(info_, payload->data(), payload->size(), payload);
  if (!decoded.ok()) {
    if (decoded.status().domain() == Code::InvalidArgument) return no_frame();
    return decoded.status();
  }
  sensor::RgbdFrame frame = std::move(decoded).value();
  frame.timestamp_ns = at.pts_ns;
  frame.sequence = at.seq;
  if (frame.depth_confidence != nullptr &&
      frame.min_confidence < options_.min_confidence) {
    frame.min_confidence = options_.min_confidence;
  }
  if (options_.registration) {
    frame.color_to_world =
        options_.registration->tracking_to_world * frame.color_to_world;
  }
  if (handed_out_ && at.seq > last_sequence_ + 1) {
    skipped_.push_back({last_sequence_ + 1, at.seq - last_sequence_ - 1,
                        last_time_, at.pts_ns});
  }
  handed_out_ = true;
  last_sequence_ = at.seq;
  if (at.pts_ns != 0) last_time_ = at.pts_ns;
  return some_frame(std::move(frame));
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
