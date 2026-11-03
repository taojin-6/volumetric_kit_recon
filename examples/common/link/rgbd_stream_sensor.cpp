// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "link/rgbd_stream_sensor.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <iterator>
#include <map>
#include <memory>
#include <new>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "volumetric_kit/core/base/check.hpp"
#include "volumetric_kit/core/base/log.hpp"
#include "volumetric_kit/core/base/unique_fd.hpp"
#include "volumetric_kit/core/net/dial.hpp"
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

namespace stream = volumetric_kit::grid::stream;
namespace gnet = volumetric_kit::grid::networking;
namespace net = volumetric_kit::core::net;
namespace wire = volumetric_kit::sensor::wire;
namespace hevc = volumetric_kit::sensor::hevc;
using Code = vkc::Status::Code;
using FrameResult = vkc::Result<std::optional<sensor::RgbdFrame>>;
using Clock = std::chrono::steady_clock;

// A live stream ends at its third failure within kFailureWindow.
constexpr std::size_t kMaxFailures = 3;
constexpr std::chrono::seconds kFailureWindow{10};
// How long listen's wait for the first session reads at a time.
constexpr std::chrono::milliseconds kSessionSlice{10};
// How long a failed accept waits before the next.
constexpr std::chrono::milliseconds kAcceptRetry{10};

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

// One session's records over a live link, across its writer's connections:
// the listener, the connection, the session the first one brought, the
// records taken on each track, the recording and the failure cap.
class Link {
 public:
  Link(std::unique_ptr<net::Listener> listener, std::string name)
      : listener_(std::move(listener)), name_(std::move(name)) {}

  ~Link() { end(); }
  Link(const Link&) = delete;
  Link& operator=(const Link&) = delete;

  // Takes connections until one brings a session by `deadline`; it is the
  // session from then on.
  vkc::Status await_session(Clock::time_point deadline) {
    while (!session_) {
      if (!client_) {
        VKC_TRY(accept(deadline));
        if (!client_) {
          if (Clock::now() < deadline) continue;
          return vkc::Status::io_error(name_ + ": no session within the wait");
        }
      }
      // In slices: a read returns at a record, not at the session, which a
      // writer that has no frame yet sends alone.
      stream::Record record;
      std::vector<std::uint8_t> payload;
      const vkc::Result<bool> read = client_->next(
          std::min(left(deadline), kSessionSlice), &record, &payload);
      if (!read.ok()) {
        refuse(read.status().message());
        continue;
      }
      if (client_->session() != nullptr) {
        session_ = *client_->session();
        VKC_ASSIGN(session_bytes_, session_bytes(*session_));
        last_.assign(session_->tracks.size(), std::nullopt);
        join();
        if (read.value()) pending_ = Pending{record, std::move(payload)};
      } else if (Clock::now() >= deadline) {
        return vkc::Status::io_error(name_ + ": no session within the wait");
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

  // The session's next record by `deadline`, with its payload: true with
  // one; false when none came by then, or once the stream has ended. A
  // connection's failure is never returned; the recording's write error
  // is, once.
  vkc::Result<bool> next(Clock::time_point deadline, stream::Record* record,
                         std::vector<std::uint8_t>* payload) {
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
      if (!client_) {
        VKC_TRY(accept(deadline));
        if (!client_) return false;
      }
      const vkc::Result<bool> read =
          client_->next(left(deadline), record, payload);
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
      if (!read.value() && Clock::now() >= deadline) return false;
    }
  }

  // The connection failed, or its stream did: let it go and take the
  // writer's next; the third within kFailureWindow ends the stream.
  void fail(const std::string& why) {
    warn("RgbdStreamSensor: " + name_ + ": the writer's connection failed (" +
         why + "); waiting for its next");
    let_go();
    ++stats_.failures;
    const Clock::time_point now = Clock::now();
    failures_.push_back(now);
    while (now - failures_.front() >= kFailureWindow) failures_.pop_front();
    if (failures_.size() >= kMaxFailures) {
      warn("RgbdStreamSensor: " + name_ + ": " + std::to_string(kMaxFailures) +
           " failures within " + std::to_string(kFailureWindow.count()) +
           " s: the stream ends");
      end();
    }
  }

  // The connection goes, not as a failure: the next is taken as the
  // writer redials.
  void let_go() noexcept {
    client_.reset();
    pending_.reset();
    accepted_ = false;
    stats_.connected = false;
  }

  // No more connections, and the recording finished.
  void end() noexcept {
    ended_ = true;
    let_go();
    listener_.reset();
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

 private:
  struct Pending {
    stream::Record record;
    std::vector<std::uint8_t> payload;
  };

  // The next connection by `deadline`, if one comes, to read.
  vkc::Status accept(Clock::time_point deadline) {
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

  // The connection brought the session: its records count from now on.
  void join() {
    accepted_ = true;
    stats_.connected = true;
    ++stats_.connections;
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
        recording_.reset();  // a file that may end inside the record
        error_ = std::move(written).with_context(
            "RgbdStreamSensor: recording " + name_ + "'s session");
      }
    }
    return true;
  }

  std::unique_ptr<net::Listener> listener_;
  std::string name_;  // "host:port", what each message names
  std::unique_ptr<gnet::StreamClient> client_;
  bool accepted_ = false;  // client_ brought the session
  std::optional<stream::Session> session_;
  std::vector<std::uint8_t> session_bytes_;  // its VRST chunk
  std::optional<Pending> pending_;  // a record that came with the session
  std::vector<std::optional<std::uint64_t>> last_;  // per track
  std::unique_ptr<stream::StreamWriter> recording_;
  std::deque<Clock::time_point> failures_;  // within the window
  bool ended_ = false;
  vkc::Status error_;  // the recording's, returned once
  RgbdStreamSensor::LinkStats stats_;
};

}  // namespace

struct RgbdStreamSensor::State {
  // Where the records come from: a file's reader, or a live link.
  std::unique_ptr<stream::StreamReader> reader;
  std::unique_ptr<Link> link;
  std::vector<std::uint8_t> live_payload;  // the live record's, read with it
  std::uint64_t arrival_ns = 0;            // when the live record came
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
  // refusals named after `where`.
  static vkc::Result<std::unique_ptr<State>> open(
      const stream::Session& session, Options options,
      const std::string& where);
};

vkc::Result<std::unique_ptr<RgbdStreamSensor::State>>
RgbdStreamSensor::State::open(const stream::Session& session, Options options,
                              const std::string& where) {
  const auto bad = [&](const std::string& why) {
    return vkc::Status::invalid_argument(where + ": " + why);
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
                 ", not the stream's " + hex(session.id));
    }
    if (reg.world_epoch != 0) {
      return bad("the registration holds for world epoch " +
                 std::to_string(reg.world_epoch) +
                 ", and a session is one world, epoch 0");
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
        return decoder.status().with_context(at + ": opening the decoder");
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
  vkc::Result<std::unique_ptr<net::Listener>> listening =
      net::Listener::create(host, port);
  if (!listening.ok()) return listening.status().with_context(kApi);
  auto link = std::make_unique<Link>(std::move(listening).value(), name);
  if (vkc::Status came = link->await_session(after(wait)); !came.ok()) {
    return came.with_context(kApi);
  }
  VKC_ASSIGN(std::unique_ptr<State> state,
             State::open(link->session(), std::move(options), where));
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
  if (link) link->end();  // live, the stream ends with it
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
