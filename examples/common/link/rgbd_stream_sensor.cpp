// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "link/rgbd_stream_sensor.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <iterator>
#include <map>
#include <memory>
#include <new>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "volumetric_kit/core/base/check.hpp"
#include "volumetric_kit/grid/stream/session.hpp"
#include "volumetric_kit/grid/stream/stream_reader.hpp"
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
namespace wire = volumetric_kit::sensor::wire;
namespace hevc = volumetric_kit::sensor::hevc;
using Code = vkc::Status::Code;
using FrameResult = vkc::Result<std::optional<sensor::RgbdFrame>>;

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

  // `seq` counts at `pts`: a span for the records it jumps over.
  void take(std::uint64_t seq, std::uint64_t pts,
            std::vector<RgbdStreamSensor::Skipped>* spans) {
    const std::uint64_t expected = any ? last + 1 : 0;
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
    const std::uint64_t expected = any ? last + 1 : 0;
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

}  // namespace

struct RgbdStreamSensor::State {
  std::unique_ptr<stream::StreamReader> reader;
  std::uint16_t track = 0;                     // the depth track's index
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
  std::int64_t next_unit = 0;
  std::map<std::int64_t, ColorFacts> in_flight;
  std::optional<std::uint64_t> picture_set;  // the last picture's
  Run color;

  // The join: depth frames and pictures in set order, and the frames whole.
  std::deque<Waiting> waiting;
  std::deque<Picture> pictures;
  std::deque<sensor::RgbdFrame> ready;

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
                 ", not the file's " + hex(session.id));
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

const std::vector<RgbdStreamSensor::Skipped>& RgbdStreamSensor::skipped_color()
    const noexcept {
  return state_->skipped_color;
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
  s.rewind();
}

void RgbdStreamSensor::State::rewind() noexcept {
  file_ended = false;
  at_end = false;
  position = reader->begin();
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
  for (;;) {
    if (!s.ready.empty()) return s.hand_out();
    if (s.file_ended) {
      s.finish();
      return no_frame();
    }
    const stream::Position before = s.position;
    stream::Record record;
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
      // Damage, or a record of a later writer, which the reader is now past:
      // what it held is lost, and the next frame's sequence says so.
      const Code code = more.status().domain();
      if (code == Code::InvalidArgument || code == Code::Unsupported) continue;
      return more.status();
    }
    if (!more.value()) {
      const vkc::Status ended = s.end_file();
      if (!ended.ok()) return ended;
      continue;
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
    } else {
      continue;
    }
    s.join();
    if (!taken.ok()) return taken;
  }
} catch (const std::bad_alloc&) {
  return vkc::Status::out_of_memory("RgbdStreamSensor::poll");
}

FrameResult RgbdStreamSensor::State::take(const stream::Record& record) {
  const stream::RecordInfo& at = record.info;
  depth.found = std::max(depth.found, at.seq + 1);
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
  taken = true;
  taken_sequence = at.seq;
  taken_set = at.set;
  if (at.pts_ns != 0) taken_time = at.pts_ns;
  return some_frame(std::move(frame));
}

vkc::Status RgbdStreamSensor::State::take_color(const stream::Record& record) {
#if VR_EXAMPLE_LINK_VIDEO
  const stream::RecordInfo& at = record.info;
  color.found = std::max(color.found, at.seq + 1);
  // A unit not decoded leaves the ones after it predicted from a picture the
  // decoder never had: the gate waits for the next key.
  std::vector<std::uint8_t> payload;
  const vkc::Status read = reader->read(record, &payload);
  if (!read.ok()) {
    gate.lose();
    return read.domain() == Code::InvalidArgument ? vkc::Status{} : read;
  }
  const vkc::Result<wire::ColorRecord> decoded =
      wire::decode_color(info, payload.data(), payload.size());
  if (!decoded.ok()) {
    gate.lose();
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
      gate.lose();
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
  if (lost) gate.lose();
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
  // A finished file's footer counts each track's records.
  const auto written = [&](std::uint16_t t) -> std::uint64_t {
    if (!reader->recovery().finished || t >= reader->stats().size()) return 0;
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
