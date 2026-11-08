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
/// **Many sessions on one address.** A @ref RgbdStreamSensor::Door listens
/// for a whole run and routes each connection by the session it brings: each
/// session it admits is a sensor of its own, read live as above, its
/// reconnects handed to it through its @ref SessionHandle.
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
///          thread; @ref RgbdStreamSensor::say and the @ref SessionHandle
///          from any.

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/fwd.hpp"
#include "volumetric_kit/grid/networking/stream_server.hpp"
#include "volumetric_kit/grid/stream/session.hpp"
#include "volumetric_kit/sensor/camera/tracked_registration.hpp"
#include "volumetric_kit/sensor/rgbd_frame.hpp"
#include "volumetric_kit/sensor/rgbd_sensor.hpp"

namespace vr_example::link {

namespace gnet = volumetric_kit::grid::networking;
namespace sensor = volumetric_kit::sensor;
namespace stream = volumetric_kit::grid::stream;
namespace vkc = volumetric_kit::core;

/// @brief How a @ref RgbdStreamSensor::Door admits: how many sessions it
///        holds, and the limits that keep strays from holding it.
struct DoorOptions {
  /// Sessions held at once, each from its arrival until its sensor is
  /// destroyed; at least 1. A new session past them is refused.
  std::size_t max_sessions = 1;
  /// Connections waiting for their session at once; at least 1. One more
  /// lets the oldest go.
  std::size_t max_pending = 8;
  /// Of them, from one peer address; at least 1. One more lets that peer's
  /// oldest go.
  std::size_t max_pending_per_peer = 2;
  /// The longest a connection may take to bring its session; positive.
  std::chrono::milliseconds handshake{3000};
  /// How long a session that ended by its failure cap or its decoder leaves
  /// its id refused, from when its sensor is destroyed; 0 or more.
  std::chrono::milliseconds cool_down{10'000};
  /// The ids remembered as refused, each a session
  /// @ref RgbdStreamSensor::Door::admit refused for what it is; past them the
  /// oldest is forgotten.
  std::size_t max_remembered = 256;
  /// How long an id refused must go unrefused for its next refusal to be
  /// logged again, its reason the same; 0 or more, 0 logging each one.
  std::chrono::milliseconds quiet{30'000};
};

/// @brief A session a @ref RgbdStreamSensor::Door holds, shared by the door,
///        the session's sensor and whoever watches it or tells its writer
///        notes: the newest connection the door routed to it, waiting for
///        the sensor; its presence; and the note for its writer.
///
/// The door makes one for each new session and hands it out with the
/// session's @ref RgbdStreamSensor::Door::Arrival; the sensor admitted from
/// it takes the connections the door routes to it from then on. A copy may
/// outlive the sensor and the door: it reads as ended once the sensor goes,
/// or, never admitted, once the arrival or the door does, and a note said to
/// it goes nowhere. @ref RgbdStreamSensor::listen's sensor has one of its own,
/// for its presence and its notes (@ref RgbdStreamSensor::handle).
///
/// Thread-safe: every call from any thread, and none waits on a socket.
class SessionHandle {
 public:
  /// @brief Why a session's stream ended.
  enum class End : std::uint8_t {
    None,        ///< It has not.
    FailureCap,  ///< Three failures within 10 s (link/failure_cap.hpp).
    Decoder,     ///< Its pictures' decoder failed for good.
    Closed,      ///< Its sensor was destroyed, or never made.
  };

  SessionHandle(const SessionHandle&) = delete;
  SessionHandle& operator=(const SessionHandle&) = delete;

  /// @return Whether a connection of the session is read now: from when its
  ///         sensor takes one until it lets it go.
  bool connected() const noexcept;

  /// @return Why the stream ended, the first reason; End::None while it
  ///         goes on.
  End ended() const noexcept;

  /// @brief Tell the session's writer @p note: the newest wins, sent on its
  ///        sensor's polling thread's next pass, and again to each later
  ///        connection of the session (@ref RgbdStreamSensor::say).
  ///
  /// Never waits: one lock, held for a move, with no socket call under it.
  /// A note no writer can take goes unsaid, as one said once the sensor is
  /// gone.
  /// @param note  Its tone and its line (grid's `kMaxNoteLineBytes` at most).
  void say(gnet::ViewerNote note) noexcept;

 protected:
  /// @brief Not connected, not ended, no note.
  SessionHandle() = default;
  /// @brief Not virtual: a handle is only ever made as the link's own kind,
  ///        which `std::shared_ptr` deletes as made.
  ~SessionHandle() = default;

  std::atomic<bool> connected_{false};  ///< What @ref connected says.
  std::atomic<End> end_{End::None};     ///< What @ref ended says.
  /// The note box: what @ref say left, until the sensor's next pass takes
  /// it.
  std::mutex said_mutex_;
  std::optional<gnet::ViewerNote> said_;  ///< Under said_mutex_.
};

/// @brief One sensor of a `.vrst` session, read back (above).
class RgbdStreamSensor final : public sensor::IRgbdSensor {
 public:
  class Door;  // many sessions on one address, each its own sensor (below)

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

  /// @brief What a live link has seen since @ref listen, or since a
  ///        @ref Door admitted its session.
  struct LinkStats {
    /// Connections that brought the session, the first one included.
    std::uint64_t connections = 0;
    /// Connections let go before any of their records was taken: before the
    /// first session, one whose session @ref open_file would refuse; after
    /// it, another session's; or one whose session did not come within the
    /// idle limit (10 s). They do not count as failures. A door's session
    /// has none: the door refuses before it (@ref Door::Stats::refused).
    std::uint64_t refused = 0;
    /// Connections of the session lost: closed or reset by the writer, idle
    /// past 10 s, a chunk that failed its check, or let go after a picture
    /// failed to decode. A writer that finishes its take is one too, since
    /// its server sends no end. Three within 10 s end the stream, a door's
    /// session alone (link/failure_cap.hpp).
    std::uint64_t failures = 0;
    /// A door's session: connections let go for a newer one the door routed
    /// to it (@ref Door), the one being read or one still waiting. They do
    /// not count as failures. Always 0 for @ref listen.
    std::uint64_t superseded = 0;
    /// Records a writer sent again on a later connection (a seq not above
    /// the last one taken on its track), skipped unseen and unrecorded.
    std::uint64_t repeated = 0;
    /// Records written to the recording.
    std::uint64_t recorded = 0;
    bool connected = false;  ///< A writer of the session is connected now.
  };

  /// The longest a live @ref poll waits for records while no frame is whole.
  static constexpr std::chrono::milliseconds kPollWait{20};

  /// How long a door's session's connection must have brought nothing for a
  /// newer one from another peer address to take its place (@ref Door).
  static constexpr std::chrono::milliseconds kSupersedeSilence{1000};

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

  /// @return The live link's counts since @ref listen, or since a @ref Door
  ///         admitted its session; all 0 for a file. The polling thread's:
  ///         another reads the session's presence through @ref handle.
  const LinkStats& link_stats() const noexcept;

  /// @return The session's id (grid's `Session::id`): the file's, the first
  ///         live session's, which every later connection must match, or the
  ///         one a @ref Door admitted.
  const std::array<std::uint8_t, 16>& session_id() const noexcept;

  /// @brief Tell a live writer @p note, up the socket it dialled (grid's
  ///        `StreamClient::send_note`): the newest wins, sent on the polling
  ///        thread's next pass, and again to each later connection of the
  ///        session. A file has no writer: nothing happens.
  ///
  /// Never waits, from any thread: one lock, held for a move, with no socket
  /// call under it. A note no writer can take goes unsaid: the stream goes on.
  /// The same as @ref SessionHandle::say on @ref handle.
  /// @param note  Its tone and its line (grid's `kMaxNoteLineBytes` at most).
  void say(gnet::ViewerNote note) noexcept;

  /// @return The live session's handle, for its presence and its notes from
  ///         any thread, which may outlive the sensor: a @ref Door's for a
  ///         session it admitted, or the link's own for @ref listen; null for
  ///         a file.
  std::shared_ptr<SessionHandle> handle() const noexcept;

 private:
  struct State;  // the records' source, its place, the decoder, the join

  explicit RgbdStreamSensor(std::unique_ptr<State> state);

  std::unique_ptr<State> state_;  // never null
};

/// @brief Writers' sessions on one address for a whole run, each its own
///        sensor: the listener and its thread, which route each connection
///        by the session it brings.
///
/// @ref open binds one numeric IPv4 address of this host, never the
/// wildcard, and returns; from then on one thread accepts there (core's
/// `net::Listener`, grid's `StreamClient::adopt`), until the door is
/// destroyed. A connection waits for its session: at most
/// @ref DoorOptions::max_pending at once and
/// @ref DoorOptions::max_pending_per_peer from one address, one more letting
/// the oldest go (@ref Stats::crowded), each read in turn without waiting,
/// until its session's chunk has come or @ref DoorOptions::handshake has
/// passed (@ref Stats::strays). Then it goes by the session's id (grid's
/// `Session::id`) and every byte of its `VRST` chunk:
/// - A held session's id and bytes: a reconnect, handed to its
///   @ref SessionHandle at the cap too, which its sensor takes as its newest.
/// - A held session's id with other bytes, or one whose stream has ended;
///   an id cooling down, or remembered as refused: refused.
/// - A new id with a slot free: the slot is the session's from then on, and
///   its @ref Arrival waits for @ref take_arrival.
/// - A new id with none free: refused.
///
/// A refused connection is closed right after its session's chunk, which a
/// writer that redials takes as a dial that failed. It is counted
/// (@ref Stats::refused) and logged once an id a spell, which lasts until
/// that id arrives, is refused for another reason, or goes
/// @ref DoorOptions::quiet unrefused; it is never an error.
///
/// A slot is held from the arrival until the session's sensor is destroyed,
/// or its arrival is, unadmitted. A session that ended by its failure cap or
/// its decoder (@ref SessionHandle::End) leaves its id refused for
/// @ref DoorOptions::cool_down after; then the id arrives as a new session.
/// A session @ref admit refused for what it is leaves its id remembered as
/// refused.
///
/// **A door's session.** Its sensor reads as
/// @ref RgbdStreamSensor::listen's does, but takes its connections from its
/// @ref SessionHandle, each already checked: a record sent again is skipped,
/// the recording is one file across every connection, and the failure cap
/// ends that session alone. A newer connection takes the place of the one
/// being read: from the same peer address at once, from another once the
/// current one has brought nothing for
/// @ref RgbdStreamSensor::kSupersedeSilence, as a writer that changed
/// networks dials again only once it has lost its reader. Neither counts as
/// a failure (@ref RgbdStreamSensor::LinkStats::superseded). With the door
/// gone, it reads the connection it has.
///
/// @code
/// VKC_ASSIGN(auto door, link::RgbdStreamSensor::Door::open(host, port));
/// // the opener's thread
/// while (running) {
///   std::optional<link::RgbdStreamSensor::Door::Arrival> arrival =
///       door->take_arrival(std::chrono::milliseconds(100));
///   if (!arrival) continue;
///   std::shared_ptr<link::SessionHandle> handle = arrival->handle;
///   vkc::Result<std::unique_ptr<link::RgbdStreamSensor>> phone =
///       door->admit(std::move(*arrival), options, record_path);
///   if (!phone.ok()) continue;  // its slot is free again
///   // poll *phone.value() on a thread of its own; handle->say from any
/// }
/// @endcode
///
/// @warning @ref take_arrival, @ref admit and @ref stats from any thread,
///          none past the destructor.
class RgbdStreamSensor::Door {
 private:
  struct Lease;  // a hold on a session's slot, which the door watches weakly

 public:
  /// @brief A new session's first connection, its slot held: for
  ///        @ref admit, or dropped, which frees the slot.
  struct Arrival {
    /// @brief Holds nothing.
    Arrival();
    Arrival(Arrival&&);
    Arrival& operator=(Arrival&&) = delete;
    Arrival(const Arrival&) = delete;
    Arrival& operator=(const Arrival&) = delete;
    ~Arrival();

    stream::Session session;  ///< What its connection brought.
    std::string peer;         ///< The connection's peer, "a.b.c.d:port".
    /// The session's, its connection waiting in it: the door routes the
    /// session's reconnects to it from now on.
    std::shared_ptr<SessionHandle> handle;
    /// Holds the session's slot: a lease of the door's own kind, which no
    /// one else can make or share; the arrival's, then its sensor's.
    std::unique_ptr<Lease> slot;
  };

  /// @brief What the door has done since @ref open.
  struct Stats {
    std::uint64_t accepted = 0;  ///< Connections accepted.
    /// Connections let go while waiting for their session, to make room
    /// (@ref DoorOptions::max_pending, @ref DoorOptions::max_pending_per_peer).
    std::uint64_t crowded = 0;
    /// Connections let go before their session came: closed, reset, bytes
    /// that are not a session's, or past @ref DoorOptions::handshake.
    std::uint64_t strays = 0;
    /// Connections closed right after their session's chunk (above).
    std::uint64_t refused = 0;
    std::uint64_t reconnects = 0;  ///< Connections routed to a held session.
    std::uint64_t arrivals = 0;    ///< New sessions, each an @ref Arrival.
    std::uint64_t admitted = 0;    ///< Arrivals @ref admit made a sensor of.
    std::size_t held = 0;          ///< Sessions holding a slot now.
  };

  /// @brief Listen on @p host : @p port, one numeric IPv4 address of this
  ///        host, never the wildcard, and return once bound, its thread
  ///        accepting.
  ///
  /// Nothing is kept from a call that fails, so a later one can bind again.
  /// @return The door; `Status::Code::InvalidArgument` for an empty,
  ///         wildcard or non-numeric host, port 0, or a limit of
  ///         @p options out of its range; `Status::Code::IoError` for an
  ///         address that cannot be bound (in use, or not this host's), or a
  ///         thread that cannot start; `Status::Code::OutOfMemory`. Messages
  ///         begin `"RgbdStreamSensor::Door::open: "`.
  static vkc::Result<std::unique_ptr<Door>> open(const std::string& host,
                                                 std::uint16_t port,
                                                 DoorOptions options = {});

  /// @brief Stop accepting, the thread joined, the listener closed and the
  ///        connections waiting let go, an arrival not admitted ended
  ///        (@ref SessionHandle::End::Closed) with its connection. The
  ///        sensors admitted read on.
  ~Door();
  Door(const Door&) = delete;
  Door& operator=(const Door&) = delete;

  /// @brief The next new session, waiting up to @p wait for one.
  /// @param wait  The longest to wait; 0 or less takes one already there.
  /// @return The session's arrival, or nothing by then.
  std::optional<Arrival> take_arrival(std::chrono::milliseconds wait);

  /// @brief A sensor of @p arrival 's session, read as @p options say, its
  ///        connections the door's from then on (above): the session checked
  ///        as @ref RgbdStreamSensor::open_file checks a file's, and with a
  ///        device, its decoder made.
  ///
  /// @p arrival is consumed, even on a failure, which frees its slot.
  /// With @p record_path, the session is recorded there as
  /// @ref RgbdStreamSensor::listen records it. A recording that cannot be
  /// made there (anything already at it, or `StreamWriter::create`'s error)
  /// costs the session nothing: the error is logged, the sensor reads
  /// unrecorded, and @ref RgbdStreamSensor::poll returns the error once, as
  /// a write that fails.
  /// @param arrival      An arrival of this door's @ref take_arrival.
  /// @param options      Which sensor, where its poses go, where its
  ///                     pictures decode.
  /// @param record_path  Where to record the session; empty records nothing.
  /// @return The sensor, not started; `Status::Code::InvalidArgument` for an
  ///         arrival that is not this door's or was admitted, which is left
  ///         as it was, or for a registration that holds for no session;
  ///         @ref RgbdStreamSensor::open_file's errors for the session, such
  ///         as `Status::Code::InvalidArgument` for one it refuses, and
  ///         `Status::Code::Unsupported` for one this build or device cannot
  ///         read. A refusal for what the session is (its tracks, their
  ///         params) leaves its id remembered as refused; one for what
  ///         @p options ask of it (a sensor it lacks or must name, a
  ///         registration not for it, a decoder the device cannot make) does
  ///         not. Messages begin `"RgbdStreamSensor::Door::admit: "`.
  vkc::Result<std::unique_ptr<RgbdStreamSensor>> admit(
      Arrival&& arrival, Options options, std::string record_path = {});

  /// @return The counts since @ref open.
  Stats stats() const;

 private:
  struct Impl;  // the listener, the thread, the sessions held, the arrivals

  explicit Door(std::unique_ptr<Impl> impl) noexcept;

  std::unique_ptr<Impl> impl_;  // never null
};

}  // namespace vr_example::link
