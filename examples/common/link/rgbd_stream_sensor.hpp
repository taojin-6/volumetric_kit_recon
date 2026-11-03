// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/link/rgbd_stream_sensor.hpp
/// @brief One sensor of a session in a `.vrst` file, as a
///        `sensor::IRgbdSensor`: the frames it handed its writer, rebuilt
///        from their depth records.
///
/// What it is (`info()`): its `RgbdDepth` track's params
/// (`sensor::wire::decode_sensor`), with the track's sensor id, clock and
/// rate. It reads the depth track alone, so it names no colour camera.
///
/// A frame is a depth record's half (`sensor::wire::decode_depth`): its depth
/// and confidence plane point into the record's payload, which its pixels
/// hold, so nothing is copied; its cameras, pose, units, range and noise are
/// the record's; its timestamp is the record's pts and its sequence the
/// record's seq, which the writer counts one up per record. A record lost on
/// the way, or one that fails its CRC, the decode, or the order of the frames
/// before it, gives no frame and leaves a gap in the sequence, reported as a
/// span (@ref RgbdStreamSensor::skipped). So the reader keeps the frame
/// contract: `testing::rgbd_breach` is empty.
///
/// A tracked sensor's poses are in its own world, and a registration places
/// them in the array's: `tracking_to_world * color_to_world`. A fixed
/// sensor's poses are handed out as written.
///
/// Frames go out as fast as they are polled: each poll reads the next one.
///
/// @code
/// link::RgbdStreamSensor::Options options;
/// VKC_ASSIGN(options.registration,
///            sensor::camera::read_tracked_registration(registration_path));
/// options.min_confidence = 2;  // the phone's high, whatever it fused at
/// VKC_ASSIGN(auto phone, link::RgbdStreamSensor::open_file(path, options));
/// VKC_TRY(phone->start());
/// while (!phone->exhausted()) {
///   VKC_ASSIGN(std::optional<sensor::RgbdFrame> frame, phone->poll());
///   if (!frame) continue;  // the file's end: exhausted() says so now
///   VKC_ASSIGN(const sensor::DeviceFrame prepared, prep.prepare(*frame));
/// }
/// @endcode
///
/// TODO: the pictures, decoded on the device and joined to the depth by set,
/// and a live link its writer dials.
///
/// @warning Not thread-safe: open, start, poll, drain and stop from one
///          thread.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/grid/stream/stream_reader.hpp"
#include "volumetric_kit/sensor/camera/tracked_registration.hpp"
#include "volumetric_kit/sensor/rgbd_frame.hpp"
#include "volumetric_kit/sensor/rgbd_sensor.hpp"

namespace vr_example::link {

namespace sensor = volumetric_kit::sensor;
namespace vkc = volumetric_kit::core;

/// @brief One sensor of a `.vrst` session, read back (above).
class RgbdStreamSensor final : public sensor::IRgbdSensor {
 public:
  /// @brief Which sensor, and where its poses go.
  struct Options {
    /// The sensor's id (`SensorInfo::id`); empty for a session's only one.
    std::string sensor;
    /// A tracked sensor's world in the array's: its sensor must be the
    /// track's, its session the file's (`Session::id`), its world epoch 0,
    /// since one session is one world. Empty hands out the poses in the
    /// sensor's own world.
    std::optional<sensor::camera::TrackedRegistration> registration;
    /// A floor in the producer's levels that a record with a confidence
    /// plane is raised to (`RgbdFrame::min_confidence`) where its own is
    /// lower: the lab's 2, ARKit's high, whatever floor the phone fused at.
    /// 0 keeps each record's; a record without a plane keeps 0.
    std::uint8_t min_confidence = 0;
  };

  /// @brief Depth records between two frames handed out that gave none:
  ///        lost on the way, or refused (above). The frames' sequences
  ///        jump over them.
  struct Skipped {
    std::uint64_t first = 0;  ///< The first one's seq.
    std::uint64_t count = 0;  ///< How many: 1 or more.
    /// The latest timestamp handed out before them; 0 for none.
    std::uint64_t after_ns = 0;
    /// The timestamp of the frame handed out after them.
    std::uint64_t before_ns = 0;
  };

  /// @brief Read the sensor of the session at @p path.
  /// @return The sensor, not started; `Status::Code::InvalidArgument` for a
  ///         session with no `RgbdDepth` track, several and no
  ///         @ref Options::sensor, none or several of that sensor, a
  ///         registration `validate_tracked_registration` refuses or that is
  ///         not for it (a fixed sensor, another sensor, session or epoch),
  ///         or params `decode_sensor` refuses; `Status::Code::Unsupported`
  ///         for a wire version this build does not read;
  ///         `StreamReader::open`'s errors otherwise, such as
  ///         `Status::Code::NotFound`. Messages begin
  ///         `"RgbdStreamSensor::open_file: "`.
  static vkc::Result<std::unique_ptr<RgbdStreamSensor>> open_file(
      const std::string& path, Options options);

  ~RgbdStreamSensor() override = default;
  RgbdStreamSensor(RgbdStreamSensor&&) = delete;
  RgbdStreamSensor& operator=(RgbdStreamSensor&&) = delete;

  /// @return The depth track's sensor, with no colour camera.
  const sensor::SensorInfo& info() const noexcept override { return info_; }

  /// @brief Accepts any depth of at least 1 before @ref start, and changes
  ///        nothing: no frame is held between polls, each reading the next.
  /// @return OK; `Status::Code::InvalidArgument` for 0, or once started.
  vkc::Status set_queue_depth(std::size_t frames) override;

  /// @brief Begin reading at the file's first record, the counters and
  ///        @ref skipped emptied. Idempotent: starting a running sensor
  ///        leaves its place alone.
  vkc::Status start() override;

  /// @brief Stop reading and rewind. Idempotent.
  void stop() noexcept override;

  /// @brief Hand out the next frame of the depth track.
  /// @return The frame; an empty optional at the file's end or while not
  ///         started; or the file's read error (`Status::Code::IoError`,
  ///         `Status::Code::OutOfMemory`), after which the next poll reads
  ///         on.
  vkc::Result<std::optional<sensor::RgbdFrame>> poll() override;

  /// @brief Append what @ref poll would return: the next frame, or nothing.
  /// @param out  Receives the frame; not null.
  /// @return OK, or @ref poll 's error.
  vkc::Status drain(std::vector<sensor::RgbdFrame>* out) override;

  /// @return `true` once a poll found the file's end.
  bool exhausted() const noexcept override { return at_end_; }

  /// @return The counters since the last @ref start: received, the depth
  ///         records found; delivered, the frames handed out; failed, the
  ///         records found that gave none.
  sensor::SensorStats stats() const noexcept override { return stats_; }

  /// @return The spans of records skipped since the last @ref start, in
  ///         order.
  const std::vector<Skipped>& skipped() const noexcept { return skipped_; }

 private:
  RgbdStreamSensor() = default;

  // The frame `record` of the depth track gives, or none for a record that
  // fails its CRC, the decode, or the order of the frames before it.
  vkc::Result<std::optional<sensor::RgbdFrame>> take(
      const volumetric_kit::grid::stream::Record& record);

  std::unique_ptr<volumetric_kit::grid::stream::StreamReader> reader_;
  std::uint16_t track_ = 0;  // the depth track's index
  sensor::SensorInfo info_;
  Options options_;
  bool running_ = false;
  bool at_end_ = false;
  volumetric_kit::grid::stream::Position position_;  // the next poll's
  // The last frame handed out since start, once there is one.
  bool handed_out_ = false;
  std::uint64_t last_sequence_ = 0;
  std::uint64_t last_time_ = 0;  // the last non-zero timestamp; 0 before one
  sensor::SensorStats stats_;
  std::vector<Skipped> skipped_;
};

}  // namespace vr_example::link
