// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/link/rgbd_stream_sensor.hpp
/// @brief One sensor of a session, from a `.vrst` file or a live link its
///        writer dials, as a `sensor::IRgbdSensor`: the frames it handed its
///        writer, rebuilt from their depth records and, given a device,
///        their pictures.
///
/// What it is (`info()`): its `RgbdDepth` track's params
/// (`sensor::wire::decode_sensor`), with the track's sensor id, clock and
/// rate. It names a colour camera only when it reads the sensor's
/// `RgbdColor` track, which it does given a device to decode on.
///
/// A frame is a depth record's half (`sensor::wire::decode_depth`): its depth
/// and confidence plane point into the record's payload, which its pixels
/// hold, so nothing is copied; its cameras, pose, units, range and noise are
/// the record's, a pose made in float on its device made rigid again where
/// float left it off (its homogeneous row within 1e-6 of [0, 0, 0, 1], as
/// ARKit's own transforms read, made exact; its rotation within 1e-4 of
/// orthonormal, as the device's map-step removal drifts, the nearest
/// rotation), which frame prep would refuse as not rigid; its
/// timestamp is the record's pts and its sequence the
/// record's seq, which the writer numbers from 0, one up per record. A record
/// that gives no frame keeps the frame contract (`testing::rgbd_breach` is
/// empty):
/// - One lost on the way (cut, or its header damaged), or failing its CRC or
///   the decode, leaves a gap in the sequence, reported as a span
///   (@ref RgbdStreamSensor::skipped): between two frames, before the first
///   (from seq 0), or after the last as a finished file's footer counts the
///   records. A file without its footer (a killed writer's) says nothing of
///   records after its last one.
/// - One out of order with the frames before it is counted failed. A seq not
///   above the last frame's repeats one, so it leaves no gap and no span; a
///   seq above it with a non-zero pts not above the last one's leaves a gap,
///   as any refused record.
///
/// **Its picture.** Reading colour, a frame goes out only with its own
/// picture: that of the colour record of its set and pts, decoded on the
/// device by sensor's `HevcDecoder` (VideoToolbox on Apple, NVDEC elsewhere),
/// with that record's colour camera and exposure
/// (`sensor::wire::decode_color`). Every access unit goes to the decoder in
/// order, a frame's that is then let go included, as sensor's `KeyGate` admits
/// it by the picture track's seq and the unit's own slice type
/// (`hevc::first_slice_type`): after a record lost on the way, refused, or
/// failing to decode, it waits for the next key, and restarts the decoder
/// there, once the pictures held back for display are out. A depth record
/// whose picture never comes, or comes described other than its record's
/// colour codes say (`video/frame_color.hpp`) or at another size than its
/// colour camera, gives no frame: counted failed, a gap in the sequence and a
/// span as any lost record. A picture with no depth record is let go. The
/// colour records that never reached the decoder are spans of their own
/// (@ref RgbdStreamSensor::skipped_color).
///
/// A tracked sensor's poses are in its own world, and a registration places
/// them in the array's: `tracking_to_world * color_to_world`. A fixed
/// sensor's poses are handed out as written.
///
/// From a file, frames go out as fast as they are polled: each poll reads
/// records until the next frame is whole, on the polling thread, decoding
/// included. Live (@ref RgbdStreamSensor::listen), a poll takes the records
/// that have come, on the same thread, and waits up to
/// @ref RgbdStreamSensor::kPollWait for more while no frame is whole.
///
/// @code
/// link::RgbdStreamSensor::Options options;
/// VKC_ASSIGN(options.registration,
///            sensor::camera::read_tracked_registration(registration_path));
/// options.min_confidence = 2;  // the phone's high, whatever it fused at
/// options.device = &device;    // decode the pictures; null reads depth alone
/// options.allocator = &allocator;
/// VKC_ASSIGN(auto phone, link::RgbdStreamSensor::open_file(path, options));
/// VKC_TRY(phone->start());
/// while (!phone->exhausted()) {
///   VKC_ASSIGN(std::optional<sensor::RgbdFrame> frame, phone->poll());
///   if (!frame) continue;  // the file's end: exhausted() says so now
///   VKC_ASSIGN(const sensor::DeviceFrame prepared, prep.prepare(*frame));
/// }
/// @endcode
///
/// @warning Not thread-safe: open, start, poll, drain and stop from one
///          thread.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/fwd.hpp"
#include "volumetric_kit/sensor/camera/tracked_registration.hpp"
#include "volumetric_kit/sensor/rgbd_frame.hpp"
#include "volumetric_kit/sensor/rgbd_sensor.hpp"

namespace vr_example::link {

namespace sensor = volumetric_kit::sensor;
namespace vkc = volumetric_kit::core;

/// @brief One sensor of a `.vrst` session, read back (above).
class RgbdStreamSensor final : public sensor::IRgbdSensor {
 public:
  /// @brief Which sensor, where its poses go, and where its pictures decode.
  struct Options {
    /// The sensor's id (`SensorInfo::id`); empty for a session's only one.
    std::string sensor;
    /// A tracked sensor's world in the array's: its sensor must be the
    /// track's, its session the stream's (`Session::id`), its world epoch 0,
    /// since one session is one world. Empty hands out the poses in the
    /// sensor's own world.
    std::optional<sensor::camera::TrackedRegistration> registration;
    /// A floor in the producer's levels that a record with a confidence
    /// plane is raised to (`RgbdFrame::min_confidence`) where its own is
    /// lower: the lab's 2, ARKit's high, whatever floor the phone fused at.
    /// 0 keeps each record's; a record without a plane keeps 0.
    std::uint8_t min_confidence = 0;
    /// Where the pictures decode, and are handed out (`HevcDecoder::Options`
    /// device): the sensor's `RgbdColor` track is read only with one, and a
    /// session without that track reads as depth alone. Null reads the depth
    /// alone. Borrowed: it must outlive the sensor and every frame.
    const vkc::Device* device = nullptr;
    /// What NVDEC's pictures are made through: required with a device off
    /// Apple. Borrowed, as @ref device.
    vkc::Allocator* allocator = nullptr;
    /// The queue families that read the pictures (`HevcDecoder::Options`).
    std::vector<std::uint32_t> queue_families;
  };

  /// @brief Records of one track that gave nothing where its numbers jump
  ///        over them: lost on the way, or refused (above).
  struct Skipped {
    std::uint64_t first = 0;  ///< The first one's seq.
    std::uint64_t count = 0;  ///< How many: 1 or more.
    /// The last non-zero timestamp of the records that counted before them,
    /// in seq order; 0 for none.
    std::uint64_t after_ns = 0;
    /// The timestamp of the record that counted after them; 0 for none.
    std::uint64_t before_ns = 0;
  };

  /// @brief What a live link has seen since @ref listen.
  struct LinkStats {
    /// Connections that brought the session, the first one included.
    std::uint64_t connections = 0;
    /// Connections let go before any of their records was taken: before the
    /// first session, one whose session @ref open_file would refuse; after
    /// it, another session's; or one whose session did not come within the
    /// idle limit (10 s). They do not count as failures.
    std::uint64_t refused = 0;
    /// Connections of the session lost: closed or reset by the writer, idle
    /// past 10 s, a chunk that failed its check, or let go after a picture
    /// failed to decode. A writer that finishes its take is one too, since
    /// its server sends no end. Three within 10 s end the stream
    /// (link/failure_cap.hpp).
    std::uint64_t failures = 0;
    /// Records a writer sent again on a later connection (a seq not above
    /// the last one taken on its track), skipped unseen and unrecorded.
    std::uint64_t repeated = 0;
    /// Records written to the recording.
    std::uint64_t recorded = 0;
    bool connected = false;  ///< A writer of the session is connected now.
  };

  /// The longest a live @ref poll waits for records while no frame is whole.
  static constexpr std::chrono::milliseconds kPollWait{20};

  /// @brief Read the sensor of the session at @p path.
  /// @return The sensor, not started; `Status::Code::InvalidArgument` for a
  ///         session with no `RgbdDepth` track, several and no
  ///         @ref Options::sensor, none or several of that sensor, a
  ///         registration `validate_tracked_registration` refuses or that is
  ///         not for it (a fixed sensor, another sensor, session or epoch),
  ///         or params `decode_sensor` refuses; with a device, for several
  ///         `RgbdColor` tracks of the sensor, or one on another clock than
  ///         its depth's or for params that name no colour camera;
  ///         `Status::Code::Unsupported` for a wire version or, with a
  ///         device, a picture codec this build does not read, or a device in
  ///         a build without sensor's video decoders; `StreamReader::open`'s
  ///         errors, such as `Status::Code::NotFound`, and with a picture
  ///         track `HevcDecoder::create`'s (`Status::Code::Unsupported`
  ///         where the device has no decoder path) otherwise. Messages
  ///         begin `"RgbdStreamSensor::open_file: "`.
  static vkc::Result<std::unique_ptr<RgbdStreamSensor>> open_file(
      const std::string& path, Options options);

  /// @brief Wait for a writer to dial @p host : @p port, one numeric IPv4
  ///        address of this host, never the wildcard, and return once its
  ///        session has come.
  ///
  /// The writer serves with no listener of its own (grid's
  /// `ServerOptions::listen` false) and dials this address
  /// (`core::net::dial`); each connection is read as grid's
  /// `StreamClient::adopt` reads one. Each session that comes is checked as
  /// @ref open_file checks a file's, and the first it takes is the stream's:
  /// one it refuses as `Status::Code::InvalidArgument` (no depth track of
  /// the sensor, a registration for another session) is a connection
  /// refused, and the wait goes on, so a stray writer never closes the
  /// listener. After the connection drops, or a picture fails to decode (the
  /// key gate would wait for a key that the writer makes only for a reader
  /// that joins), the connection is let go and the next one taken, which
  /// starts at a key. Three such failures within 10 s end the stream
  /// (link/failure_cap.hpp). A connection whose session differs from the
  /// first one's in any field is refused, and does not count. A record a
  /// writer sends again is skipped; one lost between connections is a gap,
  /// as in a file. The spans of a start count from the first record found
  /// after it, not from seq 0: a start takes the records that come from then
  /// on.
  ///
  /// A live stream has no end of its own: the writer's server sends none,
  /// so a writer that finishes its take reads as a connection dropped, and
  /// the next one is waited for. The caller ends the stream: it stops, or
  /// destroys the sensor.
  ///
  /// With @p record_path, every record that comes, on any track, goes into a
  /// new session file there through grid's `StreamWriter`, one record at a
  /// time, under the first session's header: one valid file across every
  /// reconnect, finished when the stream ends or the sensor is destroyed,
  /// which @ref open_file reads back frame for frame. A write that fails is
  /// returned once by @ref poll and ends the recording, finished as far as
  /// it was written; the stream goes on.
  ///
  /// Each frame's `arrival_ns` is when a poll took its depth record off the
  /// link: records that waited in the socket, as while the caller set up
  /// after this returned, carry the time they were read.
  /// @param wait  The longest to wait for the first session; positive.
  /// @return The sensor, not started; `Status::Code::InvalidArgument`, before
  ///         listening, for an empty, wildcard or non-numeric host, port 0,
  ///         a @p wait of 0 or less, a registration that holds for no
  ///         session (`validate_tracked_registration` refuses it, or its
  ///         world epoch is not 0), or anything already at @p record_path,
  ///         and once the session has come for a recording
  ///         `StreamWriter::create` refuses; `Status::Code::IoError` for an
  ///         address that cannot be bound, or when no session has come
  ///         within @p wait, naming the last one refused; and at once,
  ///         @ref open_file's other errors for a session, such as
  ///         `Status::Code::Unsupported` for one this build or device cannot
  ///         read (its wire version or picture codec, no decoder path).
  ///         Messages begin `"RgbdStreamSensor::listen: "`.
  static vkc::Result<std::unique_ptr<RgbdStreamSensor>> listen(
      const std::string& host, std::uint16_t port, Options options,
      std::string record_path = {},
      std::chrono::milliseconds wait = std::chrono::minutes(5));

  ~RgbdStreamSensor() override;
  RgbdStreamSensor(RgbdStreamSensor&&) = delete;
  RgbdStreamSensor& operator=(RgbdStreamSensor&&) = delete;

  /// @return The depth track's sensor, its colour camera kept only when the
  ///         picture track is read.
  const sensor::SensorInfo& info() const noexcept override;

  /// @brief Accepts any depth of at least 1 before @ref start, and changes
  ///        nothing: each poll reads on to the next frame.
  /// @return OK; `Status::Code::InvalidArgument` for 0, or once started.
  vkc::Status set_queue_depth(std::size_t frames) override;

  /// @brief Begin reading at the file's first record, or live at the
  ///        records that come from now on, the counters, @ref skipped and
  ///        @ref skipped_color emptied. Idempotent: starting a running
  ///        sensor leaves its place alone.
  vkc::Status start() override;

  /// @brief Stop reading and rewind, the frames and pictures held let go
  ///        and the decoder reset. Live, the connection is let go too, not
  ///        as a failure, and the next start takes the writer's next one;
  ///        the recording goes on. Idempotent.
  void stop() noexcept override;

  /// @brief Hand out the next frame, whole.
  /// @return The frame; an empty optional at the file's end, live when none
  ///         is whole within @ref kPollWait, or while not started; or the
  ///         file's read error (`Status::Code::IoError`,
  ///         `Status::Code::OutOfMemory`). After one in a record's payload,
  ///         which costs that record, the next poll reads on; one the
  ///         reader cannot get past (`StreamReader::next`'s, such as a file
  ///         shortened since it was opened), or the decoder's other than
  ///         `Status::Code::IoError` (a stream it refuses, a failed device
  ///         path), ends the file there, and the polls after it hand out the
  ///         frames already whole. Live, the stream ends so too, and a
  ///         connection's failure is never returned; the recording's write
  ///         error is, once.
  vkc::Result<std::optional<sensor::RgbdFrame>> poll() override;

  /// @brief Append what @ref poll would return: the next frame, or nothing.
  /// @param out  Receives the frame; not null.
  /// @return OK, or @ref poll 's error.
  vkc::Status drain(std::vector<sensor::RgbdFrame>* out) override;

  /// @return `true` once the file has ended, at its end or by an error, or
  ///         the live stream has (three failures within 10 s, or the
  ///         decoder's), and every frame whole by then is handed out. A
  ///         writer that finishes never ends a live stream alone
  ///         (@ref listen).
  bool exhausted() const noexcept override;

  /// @return The counters since the last @ref start: received, the depth
  ///         records found; delivered, the frames handed out; failed, the
  ///         records found that gave none, those whose picture failed
  ///         included.
  sensor::SensorStats stats() const noexcept override;

  /// @return The spans of depth records that gave no frame since the last
  ///         @ref start, in order: one before a frame once that frame is
  ///         handed out, and one after the last frame once the file has
  ///         ended.
  const std::vector<Skipped>& skipped() const noexcept;

  /// @return The spans of colour records whose access unit never reached the
  ///         decoder since the last @ref start, by the picture track's seq:
  ///         lost on the way, refused (their CRC, the decode, or a unit the
  ///         decoder would not take), or let go by the key gate while it
  ///         waited for a key. Each is found as @ref skipped finds its spans,
  ///         its times the pts of the units decoded before and after it, in
  ///         decode order: with B-frames, not the latest pts seen. Empty when
  ///         the picture track is not read.
  const std::vector<Skipped>& skipped_color() const noexcept;

  /// @return The live link's counts since @ref listen; all 0 for a file.
  const LinkStats& link_stats() const noexcept;

 private:
  struct State;  // the records' source, its place, the decoder, the join

  explicit RgbdStreamSensor(std::unique_ptr<State> state);

  std::unique_ptr<State> state_;  // never null
};

}  // namespace vr_example::link
