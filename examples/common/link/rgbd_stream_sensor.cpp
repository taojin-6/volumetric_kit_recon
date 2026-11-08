// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "link/rgbd_stream_sensor.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <exception>
#include <filesystem>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <new>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "link/failure_cap.hpp"
#include "volumetric_kit/core/base/check.hpp"
#include "volumetric_kit/core/base/log.hpp"
#include "volumetric_kit/core/base/unique_fd.hpp"
#include "volumetric_kit/core/net/address.hpp"
#include "volumetric_kit/core/net/dial.hpp"
#include "volumetric_kit/core/net/socket.hpp"
#include "volumetric_kit/grid/networking/stream_client.hpp"
#include "volumetric_kit/grid/stream/chunks.hpp"
#include "volumetric_kit/grid/stream/session.hpp"
#include "volumetric_kit/grid/stream/stream_reader.hpp"
#include "volumetric_kit/grid/stream/stream_writer.hpp"
#include "volumetric_kit/sensor/camera/camera_model.hpp"
#include "volumetric_kit/sensor/video/color_codes.hpp"
#include "volumetric_kit/sensor/video/decoded_picture.hpp"
#include "volumetric_kit/sensor/video/hevc_syntax.hpp"
#include "volumetric_kit/sensor/video/key_gate.hpp"
#include "volumetric_kit/sensor/wire/rgbd_wire.hpp"

#if VR_EXAMPLE_LINK_VIDEO
#include "volumetric_kit/sensor/video/frame_color.hpp"
#include "volumetric_kit/sensor/video/hevc_decoder.hpp"
#include "volumetric_kit/sensor/yuv_image.hpp"
#endif

namespace vr_example::link {
namespace {

namespace net = volumetric_kit::core::net;
namespace wire = volumetric_kit::sensor::wire;
namespace hevc = volumetric_kit::sensor::hevc;
using Code = vkc::Status::Code;
using FrameResult = vkc::Result<std::optional<sensor::RgbdFrame>>;

// A pose made in float on its device is rigid only to float's rounding: its
// homogeneous row up to a few ulp off [0, 0, 0, 1] (ARKit's own transforms
// read up to 3 ulp off 1 in the corner), and a rotation composed step on step
// drifting off orthonormal (the device's map-step removal: 1e-5 after six
// steps). Within these it is made rigid again -- the row exact, the rotation
// the nearest one -- so frame prep, which holds the row to 1e-12 and the
// rotation to 1e-6, takes the frame. A pose further off is handed on as
// written, for the prep to refuse.
constexpr double kFloatRowRounding = 1e-6;
constexpr double kFloatRotationDrift = 1e-4;

void exact_rigid(sensor::camera::Mat4d& m) noexcept {
  if (!(std::fabs(m[0][3]) <= kFloatRowRounding &&
        std::fabs(m[1][3]) <= kFloatRowRounding &&
        std::fabs(m[2][3]) <= kFloatRowRounding &&
        std::fabs(m[3][3] - 1.0) <= kFloatRowRounding)) {
    return;
  }
  glm::dmat3 r(m);
  const glm::dmat3 gram = glm::transpose(r) * r;
  double drift = 0.0;
  for (int c = 0; c < 3; ++c) {
    for (int k = 0; k < 3; ++k) {
      drift = std::max(drift, std::fabs(gram[c][k] - (c == k ? 1.0 : 0.0)));
    }
  }
  if (!(drift <= kFloatRotationDrift)) return;  // further off: as written
  m[0][3] = 0.0;
  m[1][3] = 0.0;
  m[2][3] = 0.0;
  m[3][3] = 1.0;
  // Orthonormal to double's rounding already (a calibration's): as written.
  if (drift <= 1e-12) return;
  // The polar factor by Newton's iteration, which squares the drift a step.
  for (int i = 0; i < 4; ++i) r = 0.5 * (r + glm::inverse(glm::transpose(r)));
  for (int c = 0; c < 3; ++c) {
    for (int k = 0; k < 3; ++k) m[c][k] = r[c][k];
  }
}
using Clock = std::chrono::steady_clock;

// How long listen's wait for the first session reads at a time.
constexpr std::chrono::milliseconds kSessionSlice{10};
// How long a failed accept waits before the next.
constexpr std::chrono::milliseconds kAcceptRetry{10};
// How long a door's accept waits a pass, between its reads of the
// connections waiting for their session.
constexpr std::chrono::milliseconds kDoorAccept{5};
// The ids a door keeps the last refusal it logged of, the oldest forgotten.
constexpr std::size_t kSaidIds = 256;

// What the join holds, at most, of each kind: depth frames waiting for their
// pictures, pictures waiting for their depth records, and access units sent
// whose pictures have not come. H.265 holds at most 16 pictures back for
// display, and an FFmpeg before 7.1 hands out one picture an access unit, so
// a picture can trail its unit by as many again; past that it never comes.
// The writer puts each frame's colour record just before its depth record,
// so neither waits for more than that.
constexpr std::size_t kMaxHeld = 32;

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

// What a colour record says of its picture, beside the access unit.
struct ColorFacts {
  std::uint64_t set = 0;
  std::uint64_t pts = 0;
  sensor::camera::CameraModel camera;
  sensor::ColorCodes codes;
  sensor::ColorExposure exposure;
};

// A decoded picture and its record, waiting for the depth record of its set.
struct Picture {
  ColorFacts facts;
  sensor::DecodedPicture decoded;
};

// A depth record's frame, in its set, waiting for its picture.
struct Waiting {
  std::uint64_t set = 0;
  sensor::RgbdFrame frame;
};

// A track's place in its seq numbers, for its spans: the last record that
// counted and its time, and how far the records found reach.
struct Run {
  bool any = false;
  std::uint64_t last = 0;
  std::uint64_t time = 0;   // the last non-zero time; 0 before one
  std::uint64_t found = 0;  // one past the greatest seq found
  // Live, the first seq found since the start, which the spans count from:
  // a start takes the records that come from then on. A file's count from 0.
  std::optional<std::uint64_t> origin;

  // A record numbered `seq` was found; `live`, it may be the origin.
  void find(std::uint64_t seq, bool live) {
    found = std::max(found, seq + 1);
    if (live && !origin) origin = seq;
  }

  // `seq` counts at `pts`: a span for the records it jumps over.
  void take(std::uint64_t seq, std::uint64_t pts,
            std::vector<RgbdStreamSensor::Skipped>* spans) {
    const std::uint64_t expected = any ? last + 1 : origin.value_or(0);
    if (seq > expected) spans->push_back({expected, seq - expected, time, pts});
    if (!any || seq > last) last = seq;
    any = true;
    if (pts != 0) time = pts;
  }

  // The track has ended after `written` records (0 when the file does not
  // say): a span for those past the last that counted.
  void end(std::uint64_t written,
           std::vector<RgbdStreamSensor::Skipped>* spans) const {
    const std::uint64_t reach = std::max(written, found);
    const std::uint64_t expected = any ? last + 1 : origin.value_or(0);
    if (reach > expected) {
      spans->push_back({expected, reach - expected, time, 0});
    }
  }
};

#if VR_EXAMPLE_LINK_VIDEO
// Whether `p` was decoded as its record says, as sensor's decoders describe
// a picture by its stream's codes (video/frame_color.hpp), to a colour that
// can be named, at its colour camera's size.
bool as_recorded(const Picture& p) {
  const sensor::YuvImage& yuv = p.decoded.yuv;
  const sensor::ColorCodes& codes = p.facts.codes;
  const sensor::YcbcrWeights weights =
      sensor::ycbcr_weights(sensor::resolve_matrix(codes.matrix, yuv.height));
  const std::optional<sensor::ColorEncoding> encoding =
      sensor::resolve_encoding(codes.transfer, codes.primaries);
  return yuv.kr == weights.kr && yuv.kb == weights.kb &&
         yuv.full_range == codes.full_range &&
         yuv.chroma_location ==
             sensor::resolve_chroma_location(codes.chroma_location) &&
         encoding && p.decoded.encoding && *encoding == *p.decoded.encoding &&
         yuv.width == p.facts.camera.size.width &&
         yuv.height == p.facts.camera.size.height;
}
#endif

// `now + wait`, held at the clock's end for a wait past it.
Clock::time_point after(std::chrono::milliseconds wait) {
  const Clock::time_point now = Clock::now();
  if (wait >= std::chrono::duration_cast<std::chrono::milliseconds>(
                  Clock::time_point::max() - now)) {
    return Clock::time_point::max();
  }
  return now + wait;
}

// What is left until `deadline`, in whole milliseconds; 0 past it.
std::chrono::milliseconds left(Clock::time_point deadline) {
  const Clock::time_point now = Clock::now();
  if (deadline <= now) return std::chrono::milliseconds{0};
  if (deadline == Clock::time_point::max()) {
    return std::chrono::milliseconds::max();
  }
  return std::chrono::ceil<std::chrono::milliseconds>(deadline - now);
}

// A session as its `VRST` chunk carries it: two sessions are one when these
// bytes are, every field of each.
vkc::Result<std::vector<std::uint8_t>> session_bytes(
    const stream::Session& session) {
  VKC_ASSIGN(const stream::ChunkEncoder encoder,
             stream::ChunkEncoder::create(session));
  return encoder.session_chunk();
}

void warn(const std::string& message) {
  vkc::log_message(vkc::LogLevel::Warning, "vr", message);
}

void inform(const std::string& message) {
  vkc::log_message(vkc::LogLevel::Info, "vr", message);
}

// The first 8 hex digits of a session id, which name it in messages.
std::string hex8(const std::array<std::uint8_t, 16>& id) {
  return hex(id).substr(0, 8);
}

// A wait on a condition variable, held to an hour: wait_for would overflow
// at milliseconds::max(), which left() gives for no deadline.
std::chrono::milliseconds bounded(std::chrono::milliseconds wait) {
  return std::min<std::chrono::milliseconds>(wait, std::chrono::hours(1));
}

// What a registration says alone, before any session: it is valid, and for
// world epoch 0, since a session is one world. Refusals named after `where`.
vkc::Status check_registration(const sensor::camera::TrackedRegistration& reg,
                               const std::string& where) {
  const vkc::Status valid = sensor::camera::validate_tracked_registration(reg);
  if (!valid.ok()) return valid.with_context(where + ": the registration");
  if (reg.world_epoch != 0) {
    return vkc::Status::invalid_argument(
        where + ": the registration holds for world epoch " +
        std::to_string(reg.world_epoch) +
        ", and a session is one world, epoch 0");
  }
  return {};
}

// A record read with its connection's session, or held until it is taken.
struct Taken {
  stream::Record record;
  std::vector<std::uint8_t> payload;
};

// A connection that brought its session, as a door hands it to the session's
// sensor: the client, its peer, and the record read with the session, if one
// came.
struct Connection {
  std::unique_ptr<gnet::StreamClient> client;
  std::string peer;           // "a.b.c.d:port"
  std::uint32_t address = 0;  // the peer's, which "the same peer" compares
  std::optional<Taken> first;
};

// A SessionHandle with the door's side and the sensor's: the newest
// connection, handed from the door's thread to the polling one, and what
// sets the presence. Every call from any thread; none waits on a socket.
class SharedSession final : public SessionHandle {
 public:
  // The door's: `connection` is the session's newest, and one still waiting
  // is let go for it. Null; or, once the stream has ended, `connection` back.
  std::unique_ptr<Connection> offer(std::unique_ptr<Connection> connection) {
    std::unique_ptr<Connection> older;  // closed outside the lock
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (end_.load(std::memory_order_acquire) != End::None) {
        return connection;
      }
      older = std::move(newest_);
      newest_ = std::move(connection);
    }
    if (older) replaced_.fetch_add(1, std::memory_order_relaxed);
    offered_.notify_one();
    return nullptr;
  }

  // The sensor's: the newest connection, waiting up to `wait` for one; null
  // when none came by then, or once the stream has ended.
  std::unique_ptr<Connection> take(std::chrono::milliseconds wait) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!newest_ && wait.count() > 0) {
      offered_.wait_for(lock, bounded(wait),
                        [&] { return newest_ != nullptr; });
    }
    return std::move(newest_);
  }

  // The peer address of the connection waiting, if one is.
  std::optional<std::uint32_t> waiting_from() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!newest_) return std::nullopt;
    return newest_->address;
  }

  // Connections let go for a newer one while they waited.
  std::uint64_t replaced() const noexcept {
    return replaced_.load(std::memory_order_relaxed);
  }

  void set_connected(bool connected) noexcept {
    connected_.store(connected, std::memory_order_release);
  }

  // The stream ends for `why`, unless it already has: the connection waiting
  // is let go, and none is offered after.
  void end(End why) noexcept {
    std::unique_ptr<Connection> waiting;  // closed outside the lock
    {
      std::lock_guard<std::mutex> lock(mutex_);
      End none = End::None;
      end_.compare_exchange_strong(none, why, std::memory_order_acq_rel);
      waiting = std::move(newest_);
    }
    connected_.store(false, std::memory_order_release);
    offered_.notify_all();
  }

  // What say left, taken.
  std::optional<gnet::ViewerNote> take_note() noexcept {
    std::lock_guard<std::mutex> lock(said_mutex_);
    return std::exchange(said_, std::nullopt);
  }

 private:
  mutable std::mutex mutex_;
  std::condition_variable offered_;
  std::unique_ptr<Connection> newest_;  // under mutex_
  std::atomic<std::uint64_t> replaced_{0};
};

// One session's records over a live link, across its writer's connections:
// where they come from (listen's own listener, or a door through the
// session's handle), the connection, the session, the records taken on each
// track, the recording, the failure cap and what the writer is told. All on
// the polling thread; what another thread sees is the handle's.
class Link {
 public:
  // listen's: connections from `listener`, the session the first one that
  // `await_session` takes brings.
  Link(std::unique_ptr<net::Listener> listener, std::string name)
      : handle_(std::make_shared<SharedSession>()),
        listener_(std::move(listener)),
        name_(std::move(name)) {}

  // A door's: the connections of `session` it routes to `handle`, each of
  // which brought the session, recorded to `recording` (or not, null);
  // `slot` held until the link goes.
  Link(std::shared_ptr<SharedSession> handle, std::shared_ptr<void> slot,
       const stream::Session& session,
       std::unique_ptr<stream::StreamWriter> recording, std::string name)
      : slot_(std::move(slot)),
        handle_(std::move(handle)),
        routed_(true),
        name_(std::move(name)),
        session_(session),
        last_(session.tracks.size()),
        recording_(std::move(recording)) {}

  ~Link() { end(SessionHandle::End::Closed); }
  Link(const Link&) = delete;
  Link& operator=(const Link&) = delete;

  // listen's: takes connections until one brings a session that `check`
  // takes by `deadline`; it is the session from then on. A session `check`
  // refuses as `InvalidArgument` is a connection refused, and the wait goes
  // on; its other errors end it.
  vkc::Status await_session(
      Clock::time_point deadline,
      const std::function<vkc::Status(const stream::Session&)>& check) {
    std::string refused;  // the last refusal's reason
    const auto none = [&] {
      return vkc::Status::io_error(
          name_ + ": no session within the wait" +
          (refused.empty() ? std::string() : "; the last refused: " + refused));
    };
    while (!session_) {
      if (!client_) {
        VKC_TRY(accept(deadline));
        if (!client_) {
          if (Clock::now() < deadline) continue;
          return none();
        }
      }
      // In slices: a read returns at a record, not at the session, which a
      // writer that has no frame yet sends alone.
      stream::Record record;
      std::vector<std::uint8_t> payload;
      const vkc::Result<bool> read = client_->next(
          std::min(left(deadline), kSessionSlice), &record, &payload);
      if (!read.ok()) {
        refused = read.status().message();
        refuse(refused);
        continue;
      }
      if (client_->session() != nullptr) {
        const vkc::Status taken = check(*client_->session());
        if (!taken.ok()) {
          if (taken.domain() != Code::InvalidArgument) {
            return taken.with_context(name_);
          }
          refused = taken.message();
          refuse(refused);
          continue;
        }
        session_ = *client_->session();
        VKC_ASSIGN(session_bytes_, session_bytes(*session_));
        last_.assign(session_->tracks.size(), std::nullopt);
        join();
        if (read.value()) pending_ = Taken{record, std::move(payload)};
      } else if (Clock::now() >= deadline) {
        return none();
      }
    }
    return {};
  }

  const stream::Session& session() const { return *session_; }

  // Every record taken from now on goes into a new session file at `path`.
  vkc::Status record_to(const std::string& path) {
    VKC_ASSIGN(recording_, stream::StreamWriter::create(path, *session_));
    return {};
  }

  // A door's recording that could not be made: `why`, returned once by the
  // next pass, as a write that failed.
  void unrecorded(vkc::Status why) {
    if (error_.ok()) error_ = std::move(why);
  }

  // The session's next record by `deadline`, with its payload: true with
  // one; false when none came by then, or once the stream has ended. A
  // connection's failure is never returned; the recording's write error
  // is, once.
  vkc::Result<bool> next(Clock::time_point deadline, stream::Record* record,
                         std::vector<std::uint8_t>* payload) {
    if (routed_) count_superseded();
    for (;;) {
      if (!error_.ok()) return std::exchange(error_, vkc::Status{});
      if (ended_) return false;
      if (pending_) {
        *record = pending_->record;
        *payload = std::move(pending_->payload);
        pending_.reset();
        if (take(*record, *payload)) return true;
        continue;
      }
      if (routed_) supersede();
      if (!client_) {
        VKC_TRY(accept(deadline));
        if (!client_) return false;
        if (pending_) continue;  // the record that came with a door's session
      }
      send_note();
      bool progressed = false;
      const vkc::Result<bool> read =
          client_->next(left(deadline), record, payload, &progressed);
      if (progressed) heard_ = Clock::now();
      if (!read.ok()) {
        if (accepted_) {
          fail(read.status().message());
        } else {
          refuse(read.status().message());
        }
        if (Clock::now() >= deadline) return false;
        continue;
      }
      if (!accepted_ && client_->session() != nullptr) {
        const vkc::Result<std::vector<std::uint8_t>> bytes =
            session_bytes(*client_->session());
        if (!bytes.ok()) {
          refuse(bytes.status().message());
          continue;
        }
        if (bytes.value() != session_bytes_) {
          refuse("another session than the first one's");
          continue;
        }
        join();
      }
      if (read.value() && accepted_ && take(*record, *payload)) return true;
      // A record sent again is skipped against the deadline too, so a
      // replayed GOP never holds the poll past it.
      if (Clock::now() >= deadline) return false;
    }
  }

  // The connection failed, or its stream did: let it go and take the
  // writer's next; kLinkFailures within kLinkFailureWindow end the stream.
  void fail(const std::string& why) {
    warn("RgbdStreamSensor: " + name_ + ": the writer's connection failed (" +
         why + "); waiting for its next");
    let_go();
    ++stats_.failures;
    if (cap_.fail(Clock::now())) {
      warn("RgbdStreamSensor: " + name_ + ": " + std::to_string(kLinkFailures) +
           " failures within " + std::to_string(kLinkFailureWindow.count()) +
           " s: the stream ends");
      end(SessionHandle::End::FailureCap);
    }
  }

  // The connection goes, not as a failure: the next is taken as the
  // writer redials.
  void let_go() noexcept {
    client_.reset();
    pending_.reset();
    accepted_ = false;
    stats_.connected = false;
    handle_->set_connected(false);
  }

  // No more connections, the handle ended for `why`, and the recording
  // finished.
  void end(SessionHandle::End why) noexcept {
    ended_ = true;
    let_go();
    listener_.reset();
    handle_->end(why);
    if (recording_) {
      vkc::Status finished = recording_->finish();
      recording_.reset();
      if (!finished.ok()) {
        try {
          warn("RgbdStreamSensor: " + name_ +
               ": the recording's footer: " + finished.message());
        } catch (...) {
        }
        if (error_.ok()) error_ = std::move(finished);
      }
    }
  }

  bool ended() const noexcept { return ended_; }
  const RgbdStreamSensor::LinkStats& stats() const noexcept { return stats_; }
  const std::shared_ptr<SharedSession>& handle() const noexcept {
    return handle_;
  }

 private:
  // The next connection by `deadline`, if one comes, to read: listen's from
  // its listener; a door's the newest it routed, which brought the session.
  vkc::Status accept(Clock::time_point deadline) {
    if (routed_) {
      std::unique_ptr<Connection> taken = handle_->take(left(deadline));
      if (!taken) return {};
      client_ = std::move(taken->client);
      peer_ = std::move(taken->peer);
      address_ = taken->address;
      pending_ = std::move(taken->first);
      join();
      return {};
    }
    if (!listener_) return {};
    vkc::Result<vkc::UniqueFd> fd = listener_->accept(left(deadline));
    if (!fd.ok()) {
      // Out of descriptors, say, which Linux keeps failing at once: wait a
      // little, and the next try takes it again.
      warn("RgbdStreamSensor: " + name_ + ": " + fd.status().message());
      std::this_thread::sleep_for(std::min(left(deadline), kAcceptRetry));
      return {};
    }
    if (!fd.value().valid()) return {};
    vkc::Result<std::unique_ptr<gnet::StreamClient>> client =
        gnet::StreamClient::adopt(std::move(fd).value());
    if (!client.ok()) {
      ++stats_.refused;  // reset as it was taken
      return {};
    }
    client_ = std::move(client).value();
    return {};
  }

  // A door's: a newer connection it routed takes the place of the one being
  // read, one from the same peer address at once, one from another once this
  // one has brought nothing for kSupersedeSilence. Not a failure.
  // TODO: door.csv's row for each connection superseded, with its peer: this
  // runs on the polling thread and door.csv is the door's thread's alone, so
  // the rows go back to it, through a queue on the SharedSession, say.
  void supersede() {
    if (!client_) return;
    const std::optional<std::uint32_t> from = handle_->waiting_from();
    if (!from) return;
    if (*from != address_ &&
        Clock::now() - heard_ < RgbdStreamSensor::kSupersedeSilence) {
      return;
    }
    inform("RgbdStreamSensor: " + name_ + ": a newer connection takes " +
           peer_ + "'s place");
    let_go();
    ++superseded_;
    count_superseded();
  }

  // The ones let go while read, and the ones that waited (the handle's).
  void count_superseded() noexcept {
    stats_.superseded = superseded_ + handle_->replaced();
  }

  // The connection brought the session: its records count from now on, and it
  // is told the newest note, as a client is one connection.
  void join() {
    accepted_ = true;
    stats_.connected = true;
    handle_->set_connected(true);
    heard_ = Clock::now();
    ++stats_.connections;
    note_unsent_ = newest_note_.has_value();
  }

  // The newest note said, to a connection that brought the session. Whatever
  // send_note returns touches no connection, and what fails on the socket the
  // read reports, so a note never costs the stream.
  void send_note() {
    if (std::optional<gnet::ViewerNote> said = handle_->take_note()) {
      newest_note_ = std::move(said);
      note_unsent_ = true;
    }
    if (!note_unsent_ || !accepted_) return;
    note_unsent_ = false;
    (void)client_->send_note(*newest_note_);
  }

  void refuse(const std::string& why) {
    warn("RgbdStreamSensor: " + name_ + ": a connection refused: " + why);
    client_.reset();
    ++stats_.refused;
  }

  // Whether `record` is new on its track, and so taken: recorded, its seq
  // the track's last. A seq not above the last is a record sent again.
  bool take(const stream::Record& record,
            const std::vector<std::uint8_t>& payload) {
    std::optional<std::uint64_t>& last = last_[record.info.track];
    if (last && record.info.seq <= *last) {
      ++stats_.repeated;
      return false;
    }
    last = record.info.seq;
    if (recording_) {
      vkc::Status written =
          recording_->append(record.info, payload.data(), payload.size());
      if (written.ok()) {
        ++stats_.recorded;
      } else {
        // A refused record leaves nothing written, so the file is finished
        // as it stands; after an I/O failure the finish returns that
        // failure, and the file reads as a killed writer's.
        (void)recording_->finish();
        recording_.reset();
        error_ = std::move(written).with_context(
            "RgbdStreamSensor: recording " + name_ + "'s session");
      }
    }
    return true;
  }

  // A door's slot, held while the link lives: first, so it goes last, once
  // the connection is closed and the recording finished.
  std::shared_ptr<void> slot_;
  // Presence, notes and, for a door's session, its connections: what another
  // thread shares. Never null.
  std::shared_ptr<SharedSession> handle_;
  bool routed_ = false;  // a door's: connections from handle_, not listener_
  std::unique_ptr<net::Listener> listener_;  // listen's
  std::string name_;  // what each message names: "host:port", and the session
  std::unique_ptr<gnet::StreamClient> client_;
  std::string peer_;           // a door's client_'s, "a.b.c.d:port"
  std::uint32_t address_ = 0;  // its peer's address
  Clock::time_point heard_{};  // when client_ last brought anything
  bool accepted_ = false;      // client_ brought the session
  std::optional<stream::Session> session_;
  std::vector<std::uint8_t> session_bytes_;  // its VRST chunk; listen's
  std::optional<Taken> pending_;  // a record that came with the session
  std::vector<std::optional<std::uint64_t>> last_;  // per track
  std::unique_ptr<stream::StreamWriter> recording_;
  FailureCap cap_;
  bool ended_ = false;
  vkc::Status error_;  // the recording's, returned once
  RgbdStreamSensor::LinkStats stats_;
  std::uint64_t superseded_ = 0;  // let go while read, for a newer one
  std::optional<gnet::ViewerNote> newest_note_;
  bool note_unsent_ = false;
};

}  // namespace

struct RgbdStreamSensor::State {
  // Where the records come from: a file's reader, or a live link.
  std::unique_ptr<stream::StreamReader> reader;
  std::unique_ptr<Link> link;
  std::vector<std::uint8_t> live_payload;  // the live record's, read with it
  std::uint64_t arrival_ns = 0;            // when the live record came
  std::array<std::uint8_t, 16> session{};  // its id
  std::uint16_t track = 0;                 // the depth track's index
  std::optional<std::uint16_t> picture_track;  // the picture track's, when read
  sensor::SensorInfo info;
  Options options;
  bool running = false;
  bool file_ended = false;    // its end, or an error, found
  bool at_end = false;        // and every frame whole by then handed out
  stream::Position position;  // the next poll's
  sensor::SensorStats stats;
  std::vector<Skipped> skipped;
  std::vector<Skipped> skipped_color;

  // The depth records: the last one taken, which the next must follow, and
  // the frames handed out, which the spans count from.
  bool taken = false;
  std::uint64_t taken_sequence = 0;
  std::uint64_t taken_time = 0;  // the last non-zero one; 0 before one
  std::uint64_t taken_set = 0;
  std::optional<std::uint64_t> depth_set;  // the greatest set found
  Run depth;

  // The colour records: the units sent to the decoder, by the number sent
  // with each, and the picture track's place among those decoded.
#if VR_EXAMPLE_LINK_VIDEO
  std::optional<sensor::HevcDecoder> decoder;
#endif
  sensor::KeyGate gate;
  bool picture_lost = false;  // the gate waits for a key since the last record
  std::int64_t next_unit = 0;
  std::map<std::int64_t, ColorFacts> in_flight;
  std::optional<std::uint64_t> picture_set;  // the last picture's
  Run color;

  // The join: depth frames and pictures in set order, and the frames whole.
  std::deque<Waiting> waiting;
  std::deque<Picture> pictures;
  std::deque<sensor::RgbdFrame> ready;

  // `record`'s payload: read from the file, or the live one taken.
  vkc::Status read_payload(const stream::Record& record,
                           std::vector<std::uint8_t>* out);

  // The decoder lost a picture, or a unit never reached it: the gate waits
  // for the next key, which live means the next connection.
  void lose_picture() noexcept {
    gate.lose();
    picture_lost = true;
  }

  // The frame `record` of the depth track gives, or none for a record that
  // fails its CRC, the decode, or the order of the records before it.
  FrameResult take(const stream::Record& record);

  // A record of the picture track: its access unit sent to the decoder as
  // the key gate admits it, and the pictures that come out taken. The
  // decoder's error other than IoError, which ends the file.
  vkc::Status take_color(const stream::Record& record);

  // Every picture the decoder has ready.
  vkc::Status receive_pictures();

  // A send failed: the pictures ready are taken, an IoError costs the units
  // up to the next key, and any other error ends the file.
  vkc::Status send_failed(vkc::Status why);

  // The stream ends, so the pictures the decoder holds back for display come
  // out. The decoder's error other than IoError.
  vkc::Status end_stream();

  // The decoder has failed for good: the file ends here.
  vkc::Status decoder_failed(vkc::Status why);

  // Depth frames and pictures matched by set, as far as they can be: whole
  // frames to `ready`, the rest let go.
  void join();

  // The next whole frame, its span counted.
  FrameResult hand_out();

  // The file has ended: what the decoder holds comes out, and nothing more
  // is waited for. The decoder's error other than IoError.
  vkc::Status end_file();

  // Every frame is handed out: the spans past the last ones.
  void finish();

  // Back to the file's first record, the join and the decoder emptied.
  void rewind() noexcept;

  // The sensor of `session` read as `options` say, with no source yet;
  // refusals named after `where`. `asked`, given, is set for a refusal of
  // what `options` ask of the session (a sensor it lacks or must name, a
  // registration not for it, a decoder the device cannot make), not of what
  // the session is.
  static vkc::Result<std::unique_ptr<State>> open(
      const stream::Session& session, Options options, const std::string& where,
      bool* asked = nullptr);
};

vkc::Result<std::unique_ptr<RgbdStreamSensor::State>>
RgbdStreamSensor::State::open(const stream::Session& session, Options options,
                              const std::string& where, bool* asked) {
  const auto bad = [&](const std::string& why) {
    return vkc::Status::invalid_argument(where + ": " + why);
  };
  // A refusal of what the options ask, not of the session.
  const auto refuse_asked = [&](vkc::Status why) {
    if (asked != nullptr) *asked = true;
    return why;
  };

  // The sensor's depth track: the session's only one, or the one named.
  std::optional<std::uint16_t> found;
  bool any = false;
  for (std::size_t i = 0; i < session.tracks.size(); ++i) {
    const stream::Track& track = session.tracks[i];
    if (track.kind != stream::TrackKind::RgbdDepth) continue;
    any = true;
    if (!options.sensor.empty() && track.sensor != options.sensor) continue;
    if (found) {
      if (options.sensor.empty()) {
        return refuse_asked(
            bad("depth tracks of several sensors: name one (Options::sensor)"));
      }
      return bad("several depth tracks of sensor " + options.sensor);
    }
    found = static_cast<std::uint16_t>(i);
  }
  if (!any) return bad("no RgbdDepth track");
  if (!found) {
    return refuse_asked(bad("no depth track of sensor " + options.sensor));
  }
  const stream::Track& track = session.tracks[*found];

  // What the sensor is: the params, with the track's id, clock and rate.
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

  if (options.registration) {
    const sensor::camera::TrackedRegistration& reg = *options.registration;
    if (vkc::Status valid = check_registration(reg, where); !valid.ok()) {
      return refuse_asked(std::move(valid));
    }
    if (info.pose != sensor::PoseSource::Tracked) {
      return refuse_asked(
          bad("a registration for sensor " + info.id +
              ", which is fixed: its poses are its calibration's"));
    }
    if (reg.sensor != info.id) {
      return refuse_asked(bad("the registration is sensor " + reg.sensor +
                              "'s, not " + info.id + "'s"));
    }
    if (reg.session != session.id) {
      return refuse_asked(
          bad("the registration is another session's: " + hex(reg.session) +
              ", not the stream's " + hex(session.id)));
    }
  }

  auto state = std::make_unique<State>();

  // With a device, the sensor's picture track, if it has one, decoded there.
  if (options.device != nullptr) {
#if VR_EXAMPLE_LINK_VIDEO
    std::optional<std::uint16_t> color;
    for (std::size_t i = 0; i < session.tracks.size(); ++i) {
      const stream::Track& t = session.tracks[i];
      if (t.kind != stream::TrackKind::RgbdColor || t.sensor != info.id) {
        continue;
      }
      if (color) return bad("several picture tracks of sensor " + info.id);
      color = static_cast<std::uint16_t>(i);
    }
    if (color) {
      const stream::Track& t = session.tracks[*color];
      const std::string at = where + ": track " + std::to_string(*color);
      if (!info.color) {
        return bad("a picture track of sensor " + info.id +
                   ", whose params name no colour camera");
      }
      if (t.clock != track.clock) {
        return bad("sensor " + info.id + "'s colour on clock " +
                   std::to_string(t.clock) + ", its depth on clock " +
                   std::to_string(track.clock));
      }
      const vkc::Result<wire::ColorCodec> codec =
          wire::decode_color_params(t.params);
      if (!codec.ok()) return codec.status().with_context(at);
      sensor::HevcDecoder::Options decoding;
      decoding.device = options.device;
      decoding.allocator = options.allocator;
      decoding.queue_families = options.queue_families.data();
      decoding.queue_family_count =
          static_cast<std::uint32_t>(options.queue_families.size());
      vkc::Result<sensor::HevcDecoder> decoder =
          sensor::HevcDecoder::create(decoding);
      if (!decoder.ok()) {
        return refuse_asked(
            decoder.status().with_context(at + ": opening the decoder"));
      }
      state->decoder.emplace(std::move(decoder).value());
      state->picture_track = color;
    }
#else
    return vkc::Status::unsupported(
        where +
        ": a device to decode pictures on, in a build without sensor's video "
        "decoders (sensor_video)");
#endif
  }
  // Without the picture track read, the sensor names no colour camera.
  if (!state->picture_track) info.color.reset();

  state->session = session.id;
  state->track = *found;
  state->info = std::move(info);
  state->options = std::move(options);
  return state;
}

vkc::Result<std::unique_ptr<RgbdStreamSensor>> RgbdStreamSensor::open_file(
    const std::string& path, Options options) try {
  constexpr const char* kApi = "RgbdStreamSensor::open_file";
  vkc::Result<std::unique_ptr<stream::StreamReader>> opened =
      stream::StreamReader::open(path);
  if (!opened.ok()) return opened.status().with_context(kApi);
  std::unique_ptr<stream::StreamReader> reader = std::move(opened).value();
  VKC_ASSIGN(std::unique_ptr<State> state,
             State::open(reader->session(), std::move(options),
                         std::string(kApi) + ": " + path));
  state->position = reader->begin();
  state->reader = std::move(reader);
  return std::unique_ptr<RgbdStreamSensor>(
      new RgbdStreamSensor(std::move(state)));
} catch (const std::bad_alloc&) {
  return vkc::Status::out_of_memory("RgbdStreamSensor::open_file");
}

vkc::Result<std::unique_ptr<RgbdStreamSensor>> RgbdStreamSensor::listen(
    const std::string& host, std::uint16_t port, Options options,
    std::string record_path, std::chrono::milliseconds wait) try {
  constexpr const char* kApi = "RgbdStreamSensor::listen";
  const std::string name = host + ":" + std::to_string(port);
  const std::string where = std::string(kApi) + ": " + name;
  if (port == 0) {
    return vkc::Status::invalid_argument(
        where + ": port 0, which no writer can be told to dial");
  }
  if (wait.count() <= 0) {
    return vkc::Status::invalid_argument(where + ": a wait of " +
                                         std::to_string(wait.count()) +
                                         " ms, which must be positive");
  }
  // What no session can mend is refused before any writer dials.
  if (options.registration) {
    VKC_TRY(check_registration(*options.registration, where));
  }
  if (!record_path.empty()) {
    std::error_code ignored;
    if (std::filesystem::exists(
            std::filesystem::symlink_status(record_path, ignored))) {
      return vkc::Status::invalid_argument(
          where + ": the recording: something is already at " + record_path);
    }
  }
  vkc::Result<std::unique_ptr<net::Listener>> listening =
      net::Listener::create(host, port);
  if (!listening.ok()) return listening.status().with_context(kApi);
  auto link = std::make_unique<Link>(std::move(listening).value(), name);
  // Each session that comes is opened as a file's; one refused is a
  // connection refused, and the next is waited for.
  std::unique_ptr<State> state;
  const auto open = [&](const stream::Session& session) -> vkc::Status {
    vkc::Result<std::unique_ptr<State>> opened =
        State::open(session, options, "its session");
    if (!opened.ok()) return opened.status();
    state = std::move(opened).value();
    return {};
  };
  if (vkc::Status came = link->await_session(after(wait), open); !came.ok()) {
    return came.with_context(kApi);
  }
  if (!record_path.empty()) {
    if (vkc::Status made = link->record_to(record_path); !made.ok()) {
      return made.with_context(where + ": the recording");
    }
  }
  state->link = std::move(link);
  return std::unique_ptr<RgbdStreamSensor>(
      new RgbdStreamSensor(std::move(state)));
} catch (const std::bad_alloc&) {
  return vkc::Status::out_of_memory("RgbdStreamSensor::listen");
}

bool SessionHandle::connected() const noexcept {
  return connected_.load(std::memory_order_acquire);
}

SessionHandle::End SessionHandle::ended() const noexcept {
  return end_.load(std::memory_order_acquire);
}

void SessionHandle::say(gnet::ViewerNote note) noexcept {
  std::lock_guard<std::mutex> lock(said_mutex_);
  said_ = std::move(note);
}

// The door watches the token, weakly: the slot frees once it goes, with the
// arrival or, admitted, with the sensor's link.
struct RgbdStreamSensor::Door::Lease {
  std::shared_ptr<void> token = std::make_shared<char>('\0');
};

RgbdStreamSensor::Door::Arrival::Arrival() = default;
RgbdStreamSensor::Door::Arrival::Arrival(Arrival&&) = default;
RgbdStreamSensor::Door::Arrival::~Arrival() = default;

struct RgbdStreamSensor::Door::Impl {
  using Id = std::array<std::uint8_t, 16>;

  // A connection waiting for its session.
  struct Waiting {
    std::unique_ptr<gnet::StreamClient> client;
    net::Endpoint peer;
    Clock::time_point since;  // accepted
  };

  // A session holding a slot, from its arrival until its holder goes.
  struct Held {
    Id id{};
    std::vector<std::uint8_t> bytes;  // its VRST chunk
    std::shared_ptr<SharedSession> handle;
    std::weak_ptr<void> slot;  // its lease's token
    bool claimed = false;      // admit has taken it
  };

  // An id's refusal logged: its reason, and when it was last refused.
  struct Said {
    Id id{};
    std::string why;
    Clock::time_point last;
  };

  Impl() = default;
  // An arrival not admitted ends with the door, its connection let go; a
  // session admitted reads on.
  ~Impl() {
    {
      std::lock_guard<std::mutex> lock(mutex);
      stopping = true;
    }
    arrived.notify_all();
    if (thread.joinable()) thread.join();
    std::lock_guard<std::mutex> lock(mutex);
    for (Held& h : held) {
      if (!h.claimed || h.slot.expired()) {
        h.handle->end(SessionHandle::End::Closed);
      }
    }
  }
  Impl(const Impl&) = delete;
  Impl& operator=(const Impl&) = delete;

  // The thread's loop: free the slots let go, take a connection, read the
  // ones waiting, until the door goes.
  void run() noexcept;
  void reap();
  // Under mutex: each slot let go freed, its id cooling down if its session
  // ended by its failure cap or its decoder; the cool-downs past dropped.
  void reap_locked(Clock::time_point now);
  void accept();
  // Room for one more connection from `address`: the ones let go for it.
  std::vector<Waiting> make_room(std::uint32_t address);
  void read_pending();
  void route(Waiting waiting, std::optional<Taken> first);
  void stray(const Waiting& waiting, const std::string& why);
  void refuse(const std::string& peer, const Id& id, const std::string& why);

  // Under mutex: the session held under `id`, if one is; how many hold a
  // slot.
  Held* find(const Id& id) {
    for (Held& h : held) {
      if (h.id == id) return &h;
    }
    return nullptr;
  }
  std::size_t holding() const {
    return static_cast<std::size_t>(
        std::count_if(held.begin(), held.end(),
                      [](const Held& h) { return !h.slot.expired(); }));
  }

  // Set by open, read only after.
  std::string name;  // "host:port", what each message names
  DoorOptions options;

  // The thread's alone.
  std::unique_ptr<net::Listener> listener;
  std::deque<Waiting> pending;
  std::deque<Said> said;

  // Shared with take_arrival, admit and stats.
  mutable std::mutex mutex;
  std::condition_variable arrived;
  std::atomic<bool> stopping{false};  // set under mutex, for arrived
  std::vector<Held> held;             // under mutex
  std::deque<Arrival> arrivals;       // under mutex
  std::deque<std::pair<Id, Clock::time_point>> cooling;  // until; under mutex
  std::deque<std::pair<Id, std::string>> remembered;     // and why; under mutex
  Stats stats;                                           // under mutex

  std::thread thread;  // started last, by open
};

void RgbdStreamSensor::Door::Impl::run() noexcept {
  while (!stopping.load(std::memory_order_acquire)) {
    try {
      reap();
      accept();
      read_pending();
    } catch (const std::exception& e) {
      // Out of memory, as a rule: the connections waiting go, and the door
      // goes on.
      pending.clear();
      try {
        warn("RgbdStreamSensor::Door: " + name + ": " + e.what());
      } catch (...) {
      }
      std::this_thread::sleep_for(kAcceptRetry);
    }
  }
}

void RgbdStreamSensor::Door::Impl::reap() {
  std::lock_guard<std::mutex> lock(mutex);
  reap_locked(Clock::now());
}

void RgbdStreamSensor::Door::Impl::reap_locked(Clock::time_point now) {
  for (auto it = held.begin(); it != held.end();) {
    if (!it->slot.expired()) {
      ++it;
      continue;
    }
    const SessionHandle::End why = it->handle->ended();
    if (why == SessionHandle::End::FailureCap ||
        why == SessionHandle::End::Decoder) {
      cooling.emplace_back(it->id, now + options.cool_down);
    }
    it->handle->end(SessionHandle::End::Closed);  // an arrival dropped
    it = held.erase(it);
  }
  while (!cooling.empty() && cooling.front().second <= now) {
    cooling.pop_front();
  }
}

void RgbdStreamSensor::Door::Impl::accept() {
  vkc::Result<vkc::UniqueFd> fd = listener->accept(kDoorAccept);
  if (!fd.ok()) {
    // Out of descriptors, say, which Linux keeps failing at once: wait a
    // little, and the next try takes it again.
    warn("RgbdStreamSensor::Door: " + name + ": " + fd.status().message());
    std::this_thread::sleep_for(kAcceptRetry);
    return;
  }
  if (!fd.value().valid()) return;
  const vkc::Result<net::Endpoint> peer =
      net::peer_of("RgbdStreamSensor::Door", fd.value().get());
  vkc::Result<std::unique_ptr<gnet::StreamClient>> client =
      gnet::StreamClient::adopt(std::move(fd).value());
  // Counted once placed, so the counts read together, and before the ones
  // let go for room close, as this returns.
  std::vector<Waiting> crowded;
  const bool placed = peer.ok() && client.ok();  // else reset as it was taken
  if (placed) {
    crowded = make_room(peer.value().ipv4);
    pending.push_back({std::move(client).value(), peer.value(), Clock::now()});
  }
  std::lock_guard<std::mutex> lock(mutex);
  ++stats.accepted;
  if (!placed) ++stats.strays;
  stats.crowded += crowded.size();
}

std::vector<RgbdStreamSensor::Door::Impl::Waiting>
RgbdStreamSensor::Door::Impl::make_room(std::uint32_t address) {
  const auto from = [&](const Waiting& w) { return w.peer.ipv4 == address; };
  std::vector<Waiting> crowded;
  if (static_cast<std::size_t>(
          std::count_if(pending.begin(), pending.end(), from)) >=
      options.max_pending_per_peer) {
    const auto oldest = std::find_if(pending.begin(), pending.end(), from);
    crowded.push_back(std::move(*oldest));
    pending.erase(oldest);
  }
  if (pending.size() >= options.max_pending) {
    crowded.push_back(std::move(pending.front()));
    pending.pop_front();
  }
  return crowded;
}

void RgbdStreamSensor::Door::Impl::read_pending() {
  for (auto it = pending.begin(); it != pending.end();) {
    Taken first;
    const vkc::Result<bool> read = it->client->next(
        std::chrono::milliseconds{0}, &first.record, &first.payload);
    if (!read.ok()) {
      stray(*it, read.status().message());
      it = pending.erase(it);
      continue;
    }
    if (it->client->session() != nullptr) {
      Waiting came = std::move(*it);
      it = pending.erase(it);
      std::optional<Taken> with;
      if (read.value()) with = std::move(first);
      route(std::move(came), std::move(with));
      continue;
    }
    if (Clock::now() - it->since >= options.handshake) {
      stray(*it, "no session within " +
                     std::to_string(options.handshake.count()) + " ms");
      it = pending.erase(it);
      continue;
    }
    ++it;
  }
}

void RgbdStreamSensor::Door::Impl::stray(const Waiting& waiting,
                                         const std::string& why) {
  vkc::log_message(vkc::LogLevel::Debug, "vr",
                   "RgbdStreamSensor::Door: " + name + ": " +
                       net::to_string(waiting.peer) +
                       " let go before its session: " + why);
  std::lock_guard<std::mutex> lock(mutex);
  ++stats.strays;
}

void RgbdStreamSensor::Door::Impl::route(Waiting waiting,
                                         std::optional<Taken> first) {
  const stream::Session& session = *waiting.client->session();
  const Id id = session.id;
  const std::string peer = net::to_string(waiting.peer);
  // A refused connection is closed as this returns, once its refusal is
  // counted, so a writer that sees it close finds it counted.
  const vkc::Result<std::vector<std::uint8_t>> bytes = session_bytes(session);
  if (!bytes.ok()) {
    refuse(peer, id, bytes.status().message());
    return;
  }
  auto connection = std::make_unique<Connection>();
  connection->peer = peer;
  connection->address = waiting.peer.ipv4;
  connection->first = std::move(first);
  std::string why;  // a refusal's reason
  {
    std::unique_lock<std::mutex> lock(mutex);
    const Clock::time_point now = Clock::now();
    // A slot let go since the last pass is freed first, as the pass would,
    // so the id goes by what that did: cooling down, or free.
    reap_locked(now);
    const auto remembered_as =
        std::find_if(remembered.begin(), remembered.end(),
                     [&](const auto& entry) { return entry.first == id; });
    if (Held* h = find(id)) {
      if (h->slot.expired() || h->handle->ended() != SessionHandle::End::None) {
        why = "its session has ended";
      } else if (h->bytes != bytes.value()) {
        why = "another session under the id of one held";
      } else {
        // A reconnect, counted before its sensor can take it.
        connection->client = std::move(waiting.client);
        connection = h->handle->offer(std::move(connection));
        if (!connection) {
          ++stats.reconnects;
          return;
        }
        why = "its session has ended";
      }
    } else if (std::any_of(cooling.begin(), cooling.end(), [&](const auto& c) {
                 return c.first == id && c.second > now;
               })) {
      why = "its session ended by its failures or its decoder within " +
            std::to_string(options.cool_down.count()) + " ms";
    } else if (remembered_as != remembered.end()) {
      why = "refused before: " + remembered_as->second;
    } else if (holding() >= options.max_sessions) {
      why = std::to_string(options.max_sessions) + " session(s) held, the most";
    } else {
      // A new session: its handle, its connection waiting there, its slot
      // and its arrival.
      Arrival arrival;
      arrival.session = session;
      arrival.peer = peer;
      auto handle = std::make_shared<SharedSession>();
      arrival.slot = std::make_unique<Lease>();
      held.push_back({id, bytes.value(), handle, arrival.slot->token, false});
      connection->client = std::move(waiting.client);
      (void)handle->offer(std::move(connection));  // new: never ended
      arrival.handle = std::move(handle);
      arrivals.push_back(std::move(arrival));
      ++stats.arrivals;
      lock.unlock();
      arrived.notify_one();
      // A new spell: its next refusal is logged.
      said.erase(std::remove_if(said.begin(), said.end(),
                                [&](const Said& s) { return s.id == id; }),
                 said.end());
      return;
    }
  }
  refuse(peer, id, why);
}  // a refused connection closes here, right after its session's chunk

void RgbdStreamSensor::Door::Impl::refuse(const std::string& peer, const Id& id,
                                          const std::string& why) {
  {
    std::lock_guard<std::mutex> lock(mutex);
    ++stats.refused;
  }
  // TODO: door.csv's row for each refused connection, with its peer, written
  // here on the door's thread once the live runs keep a door; and one for
  // each superseded (Link::supersede, and a waiting one route's offer
  // replaces).
  // Logged once an id a spell, until it arrives, is refused for another
  // reason or goes options.quiet unrefused: a writer that redials each
  // second is named once.
  const Clock::time_point now = Clock::now();
  const auto it = std::find_if(said.begin(), said.end(),
                               [&](const Said& s) { return s.id == id; });
  if (it != said.end()) {
    if (it->why == why && now - it->last < options.quiet) {
      it->last = now;
      return;
    }
    said.erase(it);
  }
  if (said.size() >= kSaidIds) said.pop_front();
  said.push_back({id, why, now});
  warn("RgbdStreamSensor::Door: " + name + ": session " + hex8(id) + " from " +
       peer + " refused: " + why);
}

vkc::Result<std::unique_ptr<RgbdStreamSensor::Door>>
RgbdStreamSensor::Door::open(const std::string& host, std::uint16_t port,
                             DoorOptions options) try {
  constexpr const char* kApi = "RgbdStreamSensor::Door::open";
  const std::string name = host + ":" + std::to_string(port);
  const std::string where = std::string(kApi) + ": " + name;
  if (port == 0) {
    return vkc::Status::invalid_argument(
        where + ": port 0, which no writer can be told to dial");
  }
  if (options.max_sessions == 0 || options.max_pending == 0 ||
      options.max_pending_per_peer == 0) {
    return vkc::Status::invalid_argument(
        where +
        ": a limit of 0 sessions or connections waiting, which must be at "
        "least 1");
  }
  if (options.handshake.count() <= 0) {
    return vkc::Status::invalid_argument(
        where + ": a handshake of " +
        std::to_string(options.handshake.count()) +
        " ms, which must be positive");
  }
  if (options.cool_down.count() < 0) {
    return vkc::Status::invalid_argument(
        where + ": a cool-down of " +
        std::to_string(options.cool_down.count()) +
        " ms, which must be 0 or more");
  }
  if (options.quiet.count() < 0) {
    return vkc::Status::invalid_argument(where + ": a quiet of " +
                                         std::to_string(options.quiet.count()) +
                                         " ms, which must be 0 or more");
  }
  vkc::Result<std::unique_ptr<net::Listener>> listening =
      net::Listener::create(host, port);
  if (!listening.ok()) return listening.status().with_context(kApi);
  auto impl = std::make_unique<Impl>();
  impl->name = name;
  impl->options = options;
  impl->listener = std::move(listening).value();
  try {
    impl->thread = std::thread([door = impl.get()] { door->run(); });
  } catch (const std::system_error& e) {
    return vkc::Status::io_error(where +
                                 ": cannot start its thread: " + e.what());
  }
  return std::unique_ptr<Door>(new Door(std::move(impl)));
} catch (const std::bad_alloc&) {
  return vkc::Status::out_of_memory("RgbdStreamSensor::Door::open");
}

RgbdStreamSensor::Door::Door(std::unique_ptr<Impl> impl) noexcept
    : impl_(std::move(impl)) {}

RgbdStreamSensor::Door::~Door() = default;

std::optional<RgbdStreamSensor::Door::Arrival>
RgbdStreamSensor::Door::take_arrival(std::chrono::milliseconds wait) {
  Impl& d = *impl_;
  std::unique_lock<std::mutex> lock(d.mutex);
  if (wait.count() > 0) {
    d.arrived.wait_for(lock, bounded(wait), [&] {
      return !d.arrivals.empty() || d.stopping.load(std::memory_order_acquire);
    });
  }
  if (d.arrivals.empty()) return std::nullopt;
  std::optional<Arrival> taken(std::move(d.arrivals.front()));
  d.arrivals.pop_front();
  return taken;
}

vkc::Result<std::unique_ptr<RgbdStreamSensor>> RgbdStreamSensor::Door::admit(
    Arrival&& arrival, Options options, std::string record_path) try {
  Impl& d = *impl_;
  const std::string name = d.name + " session " + hex8(arrival.session.id);
  const std::string where = "RgbdStreamSensor::Door::admit: " + name;
  // This door's arrival, by its lease, which no other can hold: claimed, so
  // the door no longer ends it, and the handle as the door made it.
  std::shared_ptr<SharedSession> handle;
  if (arrival.slot) {
    const std::shared_ptr<void>& token = arrival.slot->token;
    std::lock_guard<std::mutex> lock(d.mutex);
    for (Impl::Held& h : d.held) {
      if (h.handle == arrival.handle && !h.slot.owner_before(token) &&
          !token.owner_before(h.slot)) {
        h.claimed = true;
        handle = h.handle;
        break;
      }
    }
  }
  if (!handle) {
    return vkc::Status::invalid_argument(
        where + ": not an arrival of this door waiting for its admission");
  }
  // From here the arrival is consumed: a failure frees its slot, and a
  // session refused for what it is leaves its id remembered.
  const auto failed = [&](vkc::Status why, bool remember) {
    if (remember && d.options.max_remembered != 0) {
      // Its reason as the door's refusals name it, after the session.
      std::string reason = why.message();
      if (reason.rfind(where + ": ", 0) == 0) reason.erase(0, where.size() + 2);
      std::lock_guard<std::mutex> lock(d.mutex);
      if (d.remembered.size() >= d.options.max_remembered) {
        d.remembered.pop_front();
      }
      d.remembered.emplace_back(arrival.session.id, std::move(reason));
    }
    handle->end(SessionHandle::End::Closed);
    arrival.handle.reset();
    arrival.slot.reset();
    return why;
  };
  // What no session can mend is the caller's, never remembered.
  if (options.registration) {
    if (vkc::Status valid = check_registration(*options.registration, where);
        !valid.ok()) {
      return failed(std::move(valid), false);
    }
  }
  bool asked = false;  // refused for what the options ask, not what it is
  vkc::Result<std::unique_ptr<State>> opened =
      State::open(arrival.session, std::move(options), where, &asked);
  if (!opened.ok()) {
    return failed(opened.status(),
                  opened.status().domain() == Code::InvalidArgument && !asked);
  }
  std::unique_ptr<State> state = std::move(opened).value();
  // A recording that cannot be made costs the session nothing: it reads
  // unrecorded, and the first pass returns why.
  std::unique_ptr<stream::StreamWriter> recording;
  vkc::Status unrecorded;
  if (!record_path.empty()) {
    std::error_code ignored;
    if (std::filesystem::exists(
            std::filesystem::symlink_status(record_path, ignored))) {
      unrecorded = vkc::Status::invalid_argument(
          where + ": the recording: something is already at " + record_path);
    } else {
      vkc::Result<std::unique_ptr<stream::StreamWriter>> made =
          stream::StreamWriter::create(record_path, arrival.session);
      if (made.ok()) {
        recording = std::move(made).value();
      } else {
        unrecorded = made.status().with_context(where + ": the recording");
      }
    }
    if (!unrecorded.ok()) {
      warn(unrecorded.message() + "; the session goes on unrecorded");
    }
  }
  state->link =
      std::make_unique<Link>(handle, std::move(arrival.slot->token),
                             arrival.session, std::move(recording), name);
  arrival.handle.reset();
  arrival.slot.reset();
  if (!unrecorded.ok()) state->link->unrecorded(std::move(unrecorded));
  {
    std::lock_guard<std::mutex> lock(d.mutex);
    ++d.stats.admitted;
  }
  return std::unique_ptr<RgbdStreamSensor>(
      new RgbdStreamSensor(std::move(state)));
} catch (const std::bad_alloc&) {
  arrival.handle.reset();
  arrival.slot.reset();
  return vkc::Status::out_of_memory("RgbdStreamSensor::Door::admit");
}

RgbdStreamSensor::Door::Stats RgbdStreamSensor::Door::stats() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  Stats out = impl_->stats;
  out.held = impl_->holding();
  return out;
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

const std::vector<RgbdStreamSensor::Skipped>& RgbdStreamSensor::skipped_color()
    const noexcept {
  return state_->skipped_color;
}

const RgbdStreamSensor::LinkStats& RgbdStreamSensor::link_stats()
    const noexcept {
  static const LinkStats kNone;
  return state_->link ? state_->link->stats() : kNone;
}

const std::array<std::uint8_t, 16>& RgbdStreamSensor::session_id()
    const noexcept {
  return state_->session;
}

void RgbdStreamSensor::say(gnet::ViewerNote note) noexcept {
  if (state_->link) state_->link->handle()->say(std::move(note));
}

std::shared_ptr<SessionHandle> RgbdStreamSensor::handle() const noexcept {
  if (!state_->link) return nullptr;
  return state_->link->handle();
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
    s.stats = {};
    s.skipped.clear();
    s.skipped_color.clear();
  }
  return {};
}

void RgbdStreamSensor::stop() noexcept {
  State& s = *state_;
  s.running = false;
  if (s.link) s.link->let_go();
  s.rewind();
}

void RgbdStreamSensor::State::rewind() noexcept {
  file_ended = false;
  at_end = false;
  if (reader) position = reader->begin();
  picture_lost = false;
  taken = false;
  taken_sequence = 0;
  taken_time = 0;
  taken_set = 0;
  depth_set.reset();
  depth = {};
  gate = sensor::KeyGate{};
  in_flight.clear();
  picture_set.reset();
  color = {};
  waiting.clear();
  pictures.clear();
  ready.clear();
#if VR_EXAMPLE_LINK_VIDEO
  // A refusal stands, so a stream refused is refused again.
  if (decoder) (void)decoder->reset();
#endif
}

FrameResult RgbdStreamSensor::poll() try {
  State& s = *state_;
  if (!s.running || s.at_end) return no_frame();
  const Clock::time_point deadline = Clock::now() + kPollWait;
  for (;;) {
    if (!s.ready.empty()) return s.hand_out();
    if (s.file_ended) {
      s.finish();
      return no_frame();
    }
    stream::Record record;
    if (s.link) {
      if (s.link->ended()) {
        const vkc::Status ended = s.end_file();
        if (!ended.ok()) return ended;
        continue;
      }
      const vkc::Result<bool> more =
          s.link->next(deadline, &record, &s.live_payload);
      if (!more.ok()) return more.status();  // the recording's, once
      if (!more.value()) {
        if (s.link->ended()) continue;
        return no_frame();
      }
      s.arrival_ns = static_cast<std::uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::system_clock::now().time_since_epoch())
              .count());
    } else {
      const stream::Position before = s.position;
      const vkc::Result<bool> more = s.reader->next(&s.position, &record);
      if (!more.ok()) {
        // An error that leaves the reader where it was, such as a read past
        // the end of a file shortened since it opened, would come again at
        // every poll: the file ends there.
        if (s.position.offset <= before.offset) {
          (void)s.end_file();
          if (s.ready.empty()) s.finish();
          return more.status();
        }
        // Damage, or a record of a later writer, which the reader is now
        // past: what it held is lost, and the next frame's sequence says so.
        const Code code = more.status().domain();
        if (code == Code::InvalidArgument || code == Code::Unsupported) {
          continue;
        }
        return more.status();
      }
      if (!more.value()) {
        const vkc::Status ended = s.end_file();
        if (!ended.ok()) return ended;
        continue;
      }
    }
    vkc::Status taken;
    if (record.info.track == s.track) {
      ++s.stats.received;
      FrameResult frame = s.take(record);
      if (frame.ok() && frame.value()) {
        s.waiting.push_back({record.info.set, std::move(*frame.value())});
      } else {
        ++s.stats.failed;
        if (!frame.ok()) taken = frame.status();
      }
    } else if (s.picture_track && record.info.track == *s.picture_track) {
      taken = s.take_color(record);
      // Live, the writer makes a key only for a reader that joins.
      if (s.link && std::exchange(s.picture_lost, false) && taken.ok()) {
        s.link->fail("a picture failed to decode");
      }
    } else {
      continue;
    }
    s.join();
    if (!taken.ok()) return taken;
  }
} catch (const std::bad_alloc&) {
  return vkc::Status::out_of_memory("RgbdStreamSensor::poll");
}

vkc::Status RgbdStreamSensor::State::read_payload(
    const stream::Record& record, std::vector<std::uint8_t>* out) {
  if (link) {
    out->swap(live_payload);
    live_payload.clear();
    return {};
  }
  return reader->read(record, out);
}

FrameResult RgbdStreamSensor::State::take(const stream::Record& record) {
  const stream::RecordInfo& at = record.info;
  depth.find(at.seq, link != nullptr);
  if (at.set != stream::kNoSet && (!depth_set || at.set > *depth_set)) {
    depth_set = at.set;
  }
  // A record out of order would break the contract's sequence or times.
  if (taken &&
      (at.seq <= taken_sequence ||
       (at.pts_ns != 0 && taken_time != 0 && at.pts_ns <= taken_time))) {
    return no_frame();
  }
  // Reading colour, a frame is joined to its picture by set, so the sets of
  // the records taken increase too.
  if (picture_track &&
      (at.set == stream::kNoSet || (taken && at.set <= taken_set))) {
    return no_frame();
  }
  // The frame holds its payload, which its depth and plane point into.
  auto payload = std::make_shared<std::vector<std::uint8_t>>();
  const vkc::Status read = read_payload(record, payload.get());
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
  if (link) frame.arrival_ns = arrival_ns;
  if (frame.depth_confidence != nullptr &&
      frame.min_confidence < options.min_confidence) {
    frame.min_confidence = options.min_confidence;
  }
  exact_rigid(frame.color_to_world);
  exact_rigid(frame.depth_to_color);
  if (options.registration) {
    frame.color_to_world =
        options.registration->tracking_to_world * frame.color_to_world;
  }
  taken = true;
  taken_sequence = at.seq;
  taken_set = at.set;
  if (at.pts_ns != 0) taken_time = at.pts_ns;
  return some_frame(std::move(frame));
}

vkc::Status RgbdStreamSensor::State::take_color(const stream::Record& record) {
#if VR_EXAMPLE_LINK_VIDEO
  const stream::RecordInfo& at = record.info;
  color.find(at.seq, link != nullptr);
  // A unit not decoded leaves the ones after it predicted from a picture the
  // decoder never had: the gate waits for the next key.
  std::vector<std::uint8_t> payload;
  const vkc::Status read = read_payload(record, &payload);
  if (!read.ok()) {
    lose_picture();
    return read.domain() == Code::InvalidArgument ? vkc::Status{} : read;
  }
  const vkc::Result<wire::ColorRecord> decoded =
      wire::decode_color(info, payload.data(), payload.size());
  if (!decoded.ok()) {
    lose_picture();
    const bool refused = decoded.status().domain() == Code::InvalidArgument;
    return refused ? vkc::Status{} : decoded.status();
  }
  const wire::ColorRecord& r = decoded.value();
  // Every unit the gate admits is decoded, those of frames let go after it
  // too: the frames after them are predicted from them.
  const std::optional<int> type =
      hevc::first_slice_type(r.picture, r.picture_bytes);
  switch (gate.admit(at.seq, type && hevc::is_irap(*type))) {
    case sensor::KeyGate::Admission::Skip:
      return {};
    case sensor::KeyGate::Admission::Restart:
      // The pictures held back for display were decoded before the loss:
      // out with them, then a reset, so the decoder takes the key as the
      // stream's first and skips the leading pictures of a CRA.
      VKC_TRY(end_stream());
      in_flight.clear();
      if (vkc::Status reset = decoder->reset(); !reset.ok()) {
        return decoder_failed(std::move(reset));
      }
      break;
    case sensor::KeyGate::Admission::Take:
      break;
  }
  const std::int64_t unit = next_unit++;
  in_flight.emplace(
      unit, ColorFacts{at.set, at.pts_ns, r.camera, r.codes, r.exposure});
  if (vkc::Status sent = decoder->send(r.picture, r.picture_bytes, unit);
      !sent.ok()) {
    in_flight.erase(unit);
    return send_failed(std::move(sent));
  }
  color.take(at.seq, at.pts_ns, &skipped_color);
  VKC_TRY(receive_pictures());
  // A unit sent this long ago with no picture is one the decoder skipped.
  while (!in_flight.empty() &&
         in_flight.begin()->first + static_cast<std::int64_t>(kMaxHeld) <
             next_unit) {
    in_flight.erase(in_flight.begin());
  }
  return {};
#else
  (void)record;
  return {};
#endif
}

vkc::Status RgbdStreamSensor::State::receive_pictures() {
#if VR_EXAMPLE_LINK_VIDEO
  for (;;) {
    vkc::Result<std::optional<sensor::DecodedPicture>> received =
        decoder->receive();
    if (!received.ok()) {
      if (received.status().domain() != Code::IoError) {
        return decoder_failed(received.status());
      }
      // A picture that failed to decode: the ones in flight with it are
      // lost, and the gate waits for the next key.
      in_flight.clear();
      lose_picture();
      return {};
    }
    if (!received.value()) return {};
    sensor::DecodedPicture& decoded = *received.value();
    const auto found = in_flight.find(decoded.pts);
    if (found == in_flight.end()) continue;  // its unit let go before
    const ColorFacts facts = found->second;
    in_flight.erase(found);
    if (facts.set == stream::kNoSet) continue;  // a picture of no frame
    // Pictures come out in display order, the frames' order: a unit sent
    // before it for an earlier frame and still in flight never shows.
    for (auto it = in_flight.begin(); it != in_flight.end();) {
      it = it->second.set < facts.set ? in_flight.erase(it) : std::next(it);
    }
    if (picture_set && facts.set <= *picture_set) continue;  // out of order
    picture_set = facts.set;
    pictures.push_back({facts, std::move(decoded)});
  }
#else
  return {};
#endif
}

vkc::Status RgbdStreamSensor::State::send_failed(vkc::Status why) {
  const bool lost = why.domain() == Code::IoError;
  if (lost) lose_picture();
  // The decoder can have moved pictures to ready before failing, which must
  // be taken before anything more is sent; after a refusal, they come out
  // and then the refusal.
  VKC_TRY(receive_pictures());
  return lost ? vkc::Status{} : decoder_failed(std::move(why));
}

vkc::Status RgbdStreamSensor::State::end_stream() {
#if VR_EXAMPLE_LINK_VIDEO
  if (!decoder || in_flight.empty()) return {};
  if (vkc::Status ended = decoder->send(nullptr, 0, 0); !ended.ok()) {
    return send_failed(std::move(ended));
  }
  return receive_pictures();
#else
  return {};
#endif
}

vkc::Status RgbdStreamSensor::State::decoder_failed(vkc::Status why) {
  in_flight.clear();
  file_ended = true;
  if (link) link->end(SessionHandle::End::Decoder);  // live, the stream too
  return std::move(why).with_context("RgbdStreamSensor: decoding sensor " +
                                     info.id + "'s pictures");
}

void RgbdStreamSensor::State::join() {
  if (!picture_track) {
    while (!waiting.empty()) {
      ready.push_back(std::move(waiting.front().frame));
      waiting.pop_front();
    }
    return;
  }
  // The depth frames and the pictures each come in set order.
  const auto fail = [&] {
    waiting.pop_front();
    ++stats.failed;
  };
  for (;;) {
    if (!waiting.empty() && !pictures.empty()) {
      Waiting& d = waiting.front();
      Picture& p = pictures.front();
      if (d.set < p.facts.set) {
        fail();  // its picture will not come: a later frame's has
        continue;
      }
      if (d.set > p.facts.set) {
        pictures.pop_front();  // its depth record will not come
        continue;
      }
      bool whole = d.frame.timestamp_ns == p.facts.pts;
#if VR_EXAMPLE_LINK_VIDEO
      whole = whole && as_recorded(p);
#endif
      if (!whole) {
        pictures.pop_front();
        fail();
        continue;
      }
      sensor::RgbdFrame& frame = d.frame;
      frame.color = p.decoded.yuv;
      frame.color_camera = p.facts.camera;
      frame.color_encoding = *p.decoded.encoding;
      frame.color_exposure = p.facts.exposure;
      ready.push_back(std::move(frame));
      waiting.pop_front();
      pictures.pop_front();
      continue;
    }
    if (!waiting.empty()) {
      // Its picture may come yet, unless the file has ended or it has
      // waited past any picture's delay.
      if (file_ended || waiting.size() > kMaxHeld) {
        fail();
        continue;
      }
      return;
    }
    if (!pictures.empty()) {
      // Its depth record may come yet, unless one of its set or later has.
      const std::uint64_t set = pictures.front().facts.set;
      if (file_ended || (depth_set && *depth_set >= set) ||
          pictures.size() > kMaxHeld) {
        pictures.pop_front();
        continue;
      }
    }
    return;
  }
}

FrameResult RgbdStreamSensor::State::hand_out() {
  sensor::RgbdFrame frame = std::move(ready.front());
  ready.pop_front();
  depth.take(frame.sequence, frame.timestamp_ns, &skipped);
  ++stats.delivered;
  return some_frame(std::move(frame));
}

vkc::Status RgbdStreamSensor::State::end_file() {
  file_ended = true;
  vkc::Status failure = end_stream();
  in_flight.clear();
  join();
  return failure;
}

void RgbdStreamSensor::State::finish() {
  at_end = true;
  // A finished file's footer counts each track's records; a link has none.
  const auto written = [&](std::uint16_t t) -> std::uint64_t {
    if (!reader || !reader->recovery().finished ||
        t >= reader->stats().size()) {
      return 0;
    }
    return reader->stats()[t].records;
  };
  depth.end(written(track), &skipped);
  if (picture_track) color.end(written(*picture_track), &skipped_color);
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
