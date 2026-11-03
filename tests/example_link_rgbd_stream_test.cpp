// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// A session's sensor read back as an IRgbdSensor
// (examples/common/link/rgbd_stream_sensor.hpp), on sessions written here
// through grid's StreamWriter and sensor's wire form, each sensor's picture
// track (records the reader skips) before its depth track, a frame's picture
// record before its depth record. Host only.
// - Round trips: an Orbbec-like sensor (fixed, the device's clock, a lens on
//   each camera, no plane, 640 x 576) and a tracked one (the host's clock, a
//   plane, 256 x 192, its own pose, depth camera and noise each record), both
//   on pattern depth carrying each frame's index. info() is the encoded
//   sensor less its colour camera; every frame is its record's, bit for bit,
//   its depth and plane read in place, held past the sensor; rgbd_breach is
//   empty; a restart reads it again.
// - A registration places a tracked sensor's poses, and is refused for a
//   fixed sensor, another sensor, session or epoch, or one that does not
//   validate. Two sensors need Options::sensor. The floor raises records
//   with a plane only.
// - Damage: a payload byte flipped (its CRC), a header byte flipped and a
//   record cut from the file each leave a gap and a skipped span, nothing
//   else, the first and last record of a finished file included; a killed
//   writer's torn tail leaves none. A file shortened after it opened errs
//   twice, then ends, the rest a span.
// - Records out of order and records the decode refuses are counted failed;
//   a seq that repeats or goes back leaves no gap. The contract holds
//   throughout.
// - Opening refuses no depth track, a later wire version, short params, and a
//   missing file.

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "link/rgbd_stream_sensor.hpp"
#include "volumetric_kit/core/base/file_io.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/testing/scratch_dir.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/grid/stream/chunks.hpp"
#include "volumetric_kit/grid/stream/session.hpp"
#include "volumetric_kit/grid/stream/stream_reader.hpp"
#include "volumetric_kit/grid/stream/stream_writer.hpp"
#include "volumetric_kit/sensor/camera/tracked_registration.hpp"
#include "volumetric_kit/sensor/rgbd_frame.hpp"
#include "volumetric_kit/sensor/rgbd_sensor.hpp"
#include "volumetric_kit/sensor/testing/rgbd_contract.hpp"
#include "volumetric_kit/sensor/testing/rgbd_pattern.hpp"
#include "volumetric_kit/sensor/wire/rgbd_wire.hpp"

namespace {

namespace vkc = volumetric_kit::core;
namespace sensor = volumetric_kit::sensor;
namespace camera = volumetric_kit::sensor::camera;
namespace stream = volumetric_kit::grid::stream;
namespace testing = volumetric_kit::sensor::testing;
namespace wire = volumetric_kit::sensor::wire;
using Code = vkc::Status::Code;
using vr_example::link::RgbdStreamSensor;
using Id = std::array<std::uint8_t, 16>;

constexpr std::uint16_t kDeviceClock = 1;
constexpr Id kSessionId = {0x8f, 0x3c, 0x0a, 0x7e, 0x5b, 0x2d, 0x4e, 0x6f,
                           0x9a, 0x1b, 0x0c, 0x2d, 0x3e, 0x4f, 0x5a, 0x6b};

camera::CameraModel pinhole(std::uint32_t width, std::uint32_t height) {
  camera::CameraModel m;
  m.size = {width, height};
  m.intrinsics = {0.8 * width, 0.8 * width + 0.125, 0.5 * width - 0.25,
                  0.5 * height + 0.375};
  return m;
}

// A rigid transform: a turn of `angle` about +Y, then a translation.
camera::Mat4d pose(double angle, double tx) {
  camera::Mat4d m(1.0);
  m[0][0] = std::cos(angle);
  m[0][2] = -std::sin(angle);
  m[2][0] = std::sin(angle);
  m[2][2] = std::cos(angle);
  m[3][0] = tx;
  m[3][1] = -0.5 * tx;
  m[3][2] = 2.0 + tx;
  return m;
}

// An Orbbec Femto Mega's: its NFOV depth and 4K colour, each with a lens.
sensor::SensorInfo femto_info() {
  sensor::SensorInfo info;
  info.id = "CL8F25300AB";
  info.model = "Orbbec Femto Mega";
  camera::CameraModel depth = pinhole(640, 576);
  depth.distortion = {0.0731,  -0.0213, 2.5e-4, -1.25e-3,
                      0.00419, 0.512,   -0.105, 1.0e-5};
  info.depth = depth;
  camera::CameraModel color = pinhole(3840, 2160);
  color.distortion = {0.0794, -0.105, 2.5e-4, -3.75e-4, 0.0431, 0, 0, 1e-5};
  info.color = color;
  info.depth_to_color = pose(0.01, -0.032);
  info.role = sensor::SyncRole::Primary;
  info.clock = sensor::ClockDomain::Device;
  info.pose = sensor::PoseSource::Fixed;
  info.fps = 25;
  info.color_controls = camera::ColorControls{167, 60};
  return info;
}

// An iPad's: ARKit's depth and colour, no lens, posed by its own tracking,
// written at 10 Hz.
sensor::SensorInfo ipad_info() {
  sensor::SensorInfo info;
  info.id = "ipad";
  info.model = "iPad Pro";
  info.depth = pinhole(256, 192);
  info.color = pinhole(1920, 1440);
  info.role = sensor::SyncRole::FreeRun;
  info.clock = sensor::ClockDomain::Host;
  info.pose = sensor::PoseSource::Tracked;
  info.fps = 10;
  return info;
}

// The frames a driver of `info` hands its writer: pattern depth carrying
// each frame's index, with a plane at `floor` when it is above 0. A tracked
// sensor's frames each have a pose, a refined depth camera and a noise of
// their own; a fixed sensor's share its calibration's.
struct Take {
  sensor::SensorInfo info;
  std::uint16_t clock = 0;  // its tracks' clock
  std::vector<testing::PatternDepth> depth;
  std::vector<sensor::RgbdFrame> frames;
};

Take make_take(const sensor::SensorInfo& info, std::size_t count,
               std::uint8_t floor) {
  Take take;
  take.info = info;
  const bool tracked = info.pose == sensor::PoseSource::Tracked;
  take.clock = info.clock == sensor::ClockDomain::Host ? stream::kHostClock
                                                       : kDeviceClock;
  const std::uint64_t first =
      tracked ? std::uint64_t{1'762'000'000'000'000'000} : 5'000'000;
  const std::uint64_t period = 1'000'000'000 / info.fps;
  const std::uint32_t width = info.depth->size.width;
  const std::uint32_t height = info.depth->size.height;
  for (std::size_t i = 0; i < count; ++i) {
    take.depth.push_back(
        testing::pattern_depth(width, height, static_cast<std::uint32_t>(i)));
  }
  for (std::size_t i = 0; i < count; ++i) {
    const double k = static_cast<double>(i);
    sensor::RgbdFrame f;
    f.depth = take.depth[i].depth.data();
    if (floor > 0) {
      f.depth_confidence = take.depth[i].confidence.data();
      f.min_confidence = floor;
    }
    f.depth_camera = *info.depth;
    f.color_to_world = pose(0.3, 1.25);
    f.depth_noise = {0.0f, 0.95e-3f, 0.0f};
    if (tracked) {
      f.depth_camera.intrinsics.fx += 0.25 * k;
      f.color_to_world = pose(0.1 * k, 0.05 * k);
      f.depth_noise = {0.001f, 0.01f * static_cast<float>(1 + i), 0.0f};
    }
    f.depth_to_color = info.depth_to_color;
    f.metres_per_unit = 0.001f;
    f.min_depth = 0.25f;
    f.max_depth = 5.46f;
    f.timestamp_ns = first + i * period;
    f.sequence = i;
    take.frames.push_back(f);
  }
  return take;
}

// The picture track and the depth track of `take`'s sensor.
std::vector<stream::Track> tracks_of(const Take& take) {
  stream::Track picture;
  picture.kind = stream::TrackKind::RgbdColor;
  picture.clock = take.clock;
  picture.rate = {take.info.fps, 1};
  picture.width = take.info.color->size.width;
  picture.height = take.info.color->size.height;
  picture.sensor = take.info.id;
  picture.params = wire::encode_color_params(wire::ColorCodec::HevcMain8);
  stream::Track depth;
  depth.kind = stream::TrackKind::RgbdDepth;
  depth.clock = take.clock;
  depth.rate = {take.info.fps, 1};
  depth.width = take.info.depth->size.width;
  depth.height = take.info.depth->size.height;
  depth.sensor = take.info.id;
  const vkc::Result<std::string> params = wire::encode_sensor(take.info);
  if (params.ok()) depth.params = params.value();
  return {picture, depth};
}

stream::Session session_of(const std::vector<const Take*>& takes) {
  stream::Session session;
  session.id = kSessionId;
  session.created_ns = 1'762'000'000'000'000'000;
  session.writer = "recon_example_link_rgbd_stream_test";
  for (const Take* take : takes) {
    for (stream::Track& track : tracks_of(*take)) {
      session.tracks.push_back(std::move(track));
    }
  }
  return session;
}

// Frame `i` of `take` as its picture record (opaque bytes) on track
// `picture`, then its depth record on track `picture + 1`, in set `i`.
vkc::Status append_frame(stream::StreamWriter& writer, std::uint16_t picture,
                         const Take& take, std::size_t i) {
  const sensor::RgbdFrame& f = take.frames[i];
  stream::RecordInfo info;
  info.track = picture;
  info.key = true;
  info.pts_ns = f.timestamp_ns;
  info.seq = i;
  info.set = i;
  const std::array<std::uint8_t, 8> access_unit = {0,    0,    0,    1,
                                                   0x26, 0x01, 0xaf, 0x09};
  VKC_TRY(writer.append(info, access_unit.data(), access_unit.size()));
  VKC_ASSIGN(const wire::DepthRecord d, wire::encode_depth(take.info, f));
  info.track = static_cast<std::uint16_t>(picture + 1);
  return writer.append(
      info, stream::RecordPayload::of(d.prefix.data(), d.prefix.size(), d.depth,
                                      d.depth_bytes, d.confidence,
                                      d.confidence_bytes));
}

// A finished session of `takes`, a frame of each in turn.
vkc::Status write_session(const std::string& path,
                          const std::vector<const Take*>& takes) {
  VKC_ASSIGN(auto writer,
             stream::StreamWriter::create(path, session_of(takes)));
  for (std::size_t i = 0; i < takes.front()->frames.size(); ++i) {
    for (std::size_t t = 0; t < takes.size(); ++t) {
      VKC_TRY(append_frame(*writer, static_cast<std::uint16_t>(2 * t),
                           *takes[t], i));
    }
  }
  return writer->finish();
}

// Every frame `s` hands out from its start to its end.
vkc::Result<std::vector<sensor::RgbdFrame>> read_all(RgbdStreamSensor& s) {
  VKC_TRY(s.start());
  std::vector<sensor::RgbdFrame> frames;
  while (!s.exhausted()) {
    VKC_ASSIGN(std::optional<sensor::RgbdFrame> frame, s.poll());
    if (frame) frames.push_back(std::move(*frame));
  }
  return frames;
}

bool same_bytes(const void* a, const void* b, std::size_t size) {
  return a != nullptr && b != nullptr && std::memcmp(a, b, size) == 0;
}

bool same_camera(const camera::CameraModel& a, const camera::CameraModel& b) {
  static_assert(sizeof(camera::CameraModel) == 104, "no padding to compare");
  return same_bytes(&a, &b, sizeof a);
}

bool same_mat(const camera::Mat4d& a, const camera::Mat4d& b) {
  return same_bytes(&a, &b, sizeof a);
}

// `read` is `written` less its colour camera, bit for bit.
bool same_info_less_color(const sensor::SensorInfo& read,
                          const sensor::SensorInfo& written) {
  return read.id == written.id && read.model == written.model && !read.color &&
         read.depth && same_camera(*read.depth, *written.depth) &&
         same_mat(read.depth_to_color, written.depth_to_color) &&
         read.role == written.role && read.clock == written.clock &&
         read.pose == written.pose && read.fps == written.fps &&
         read.color_controls == written.color_controls;
}

// `read` is the depth half of `written`, bit for bit, but for its pose,
// which a registration may place (`color_to_world`), and its floor.
bool same_depth_half(const sensor::RgbdFrame& read,
                     const sensor::RgbdFrame& written,
                     const camera::Mat4d& color_to_world,
                     std::uint8_t min_confidence) {
  const std::size_t samples = std::size_t{written.depth_camera.size.width} *
                              written.depth_camera.size.height;
  const bool plane = written.depth_confidence != nullptr;
  return read.pixels != nullptr && read.depth != written.depth &&
         same_bytes(read.depth, written.depth, 2 * samples) &&
         (plane ? same_bytes(read.depth_confidence, written.depth_confidence,
                             samples)
                : read.depth_confidence == nullptr) &&
         read.min_confidence == min_confidence &&
         same_camera(read.depth_camera, written.depth_camera) &&
         same_mat(read.color_to_world, color_to_world) &&
         same_mat(read.depth_to_color, written.depth_to_color) &&
         read.metres_per_unit == written.metres_per_unit &&
         read.min_depth == written.min_depth &&
         read.max_depth == written.max_depth &&
         same_bytes(&read.depth_noise, &written.depth_noise,
                    sizeof read.depth_noise) &&
         read.timestamp_ns == written.timestamp_ns &&
         read.sequence == written.sequence && read.arrival_ns == 0 &&
         !read.has_color() && read.depth_device.buffer == nullptr;
}

bool carries_index(const sensor::RgbdFrame& f) {
  return testing::depth_index(f.depth, f.depth_camera.size.width,
                              f.depth_camera.size.height) ==
         std::optional<std::uint32_t>(static_cast<std::uint32_t>(f.sequence));
}

bool refused(const vkc::Result<std::unique_ptr<RgbdStreamSensor>>& opened,
             Code code, const std::string& says) {
  return !opened.ok() && opened.status().domain() == code &&
         opened.status().message().rfind("RgbdStreamSensor::open_file: ", 0) ==
             0 &&
         opened.status().message().find(says) != std::string::npos;
}

camera::TrackedRegistration registration_of(const std::string& sensor_id) {
  camera::TrackedRegistration reg;
  reg.sensor = sensor_id;
  reg.session = kSessionId;
  reg.world.sensor = "CL8F25300AB";
  reg.tracking_to_world = pose(-0.7, 0.4);
  reg.rms_px = 0.61;
  reg.views = 6;
  reg.edge_m = 0.1471;
  return reg;
}

// The fixed sensor's frames come back as written, the sensor less its colour
// camera; the frames outlive it; a restart reads them again.
int test_fixed_round_trip() {
  const vkc::test::Scratch scratch("recon_link_rgbd_fixed_");
  const std::string path = (scratch.path / "femto.vrst").string();
  const Take take = make_take(femto_info(), 5, 0);
  CHECK(write_session(path, {&take}).ok());

  vkc::Result<std::unique_ptr<RgbdStreamSensor>> opened =
      RgbdStreamSensor::open_file(path, {});
  CHECK(opened.ok());
  std::unique_ptr<RgbdStreamSensor> s = std::move(opened).value();
  CHECK(same_info_less_color(s->info(), take.info));
  CHECK(s->set_queue_depth(0).domain() == Code::InvalidArgument);
  CHECK(s->set_queue_depth(4).ok());
  CHECK(s->set_frame_notify([] {}).domain() == Code::Unsupported);
  {
    const auto before = s->poll();  // not started: nothing
    CHECK(before.ok() && !before.value() && !s->exhausted());
  }

  vkc::Result<std::vector<sensor::RgbdFrame>> read = read_all(*s);
  CHECK(read.ok() && read.value().size() == take.frames.size());
  const std::vector<sensor::RgbdFrame> frames = std::move(read).value();
  CHECK(s->set_queue_depth(2).domain() == Code::InvalidArgument);
  for (std::size_t i = 0; i < frames.size(); ++i) {
    CHECK(same_depth_half(frames[i], take.frames[i],
                          take.frames[i].color_to_world, 0));
    CHECK(carries_index(frames[i]));
  }
  CHECK(testing::rgbd_breach(s->info(), frames, s->stats()).empty());
  const sensor::SensorStats stats = s->stats();
  CHECK(stats.received == 5 && stats.delivered == 5 && stats.failed == 0 &&
        stats.dropped == 0);
  CHECK(s->skipped().empty());
  {
    const auto after = s->poll();  // the end: nothing more
    CHECK(after.ok() && !after.value() && s->exhausted());
  }

  // A restart reads the file again, counted afresh.
  s->stop();
  CHECK(!s->exhausted());
  const vkc::Result<std::vector<sensor::RgbdFrame>> again = read_all(*s);
  CHECK(again.ok() && again.value().size() == 5 && s->stats().received == 5);
  CHECK(again.value()[4].sequence == 4 && carries_index(again.value()[4]));

  // The frames hold their payloads past the sensor.
  s.reset();
  for (const sensor::RgbdFrame& f : frames) CHECK(carries_index(f));
  return 0;
}

// The tracked sensor's frames come back as written, each with its own pose,
// depth camera, noise and plane, or placed by a registration.
int test_tracked_round_trip() {
  const vkc::test::Scratch scratch("recon_link_rgbd_tracked_");
  const std::string path = (scratch.path / "ipad.vrst").string();
  const Take take = make_take(ipad_info(), 6, 2);
  CHECK(write_session(path, {&take}).ok());

  {
    vkc::Result<std::unique_ptr<RgbdStreamSensor>> opened =
        RgbdStreamSensor::open_file(path, {});
    CHECK(opened.ok());
    RgbdStreamSensor& s = *opened.value();
    CHECK(same_info_less_color(s.info(), take.info));
    const vkc::Result<std::vector<sensor::RgbdFrame>> read = read_all(s);
    CHECK(read.ok() && read.value().size() == 6);
    for (std::size_t i = 0; i < 6; ++i) {
      const sensor::RgbdFrame& f = read.value()[i];
      CHECK(
          same_depth_half(f, take.frames[i], take.frames[i].color_to_world, 2));
      CHECK(carries_index(f));
    }
    CHECK(testing::rgbd_breach(s.info(), read.value(), s.stats()).empty());
  }

  // A registration places each pose in the array's world.
  RgbdStreamSensor::Options options;
  options.registration = registration_of("ipad");
  vkc::Result<std::unique_ptr<RgbdStreamSensor>> opened =
      RgbdStreamSensor::open_file(path, options);
  CHECK(opened.ok());
  RgbdStreamSensor& s = *opened.value();
  const vkc::Result<std::vector<sensor::RgbdFrame>> read = read_all(s);
  CHECK(read.ok() && read.value().size() == 6);
  for (std::size_t i = 0; i < 6; ++i) {
    const camera::Mat4d placed =
        options.registration->tracking_to_world * take.frames[i].color_to_world;
    CHECK(!same_mat(placed, take.frames[i].color_to_world));
    CHECK(same_depth_half(read.value()[i], take.frames[i], placed, 2));
  }
  CHECK(testing::rgbd_breach(s.info(), read.value(), s.stats()).empty());
  return 0;
}

// A registration not for the sensor is refused.
int test_registration_refusals() {
  const vkc::test::Scratch scratch("recon_link_rgbd_registration_");
  const std::string fixed = (scratch.path / "femto.vrst").string();
  const std::string tracked = (scratch.path / "ipad.vrst").string();
  const Take femto = make_take(femto_info(), 2, 0);
  const Take ipad = make_take(ipad_info(), 2, 2);
  CHECK(write_session(fixed, {&femto}).ok());
  CHECK(write_session(tracked, {&ipad}).ok());

  RgbdStreamSensor::Options options;
  options.registration = registration_of(femto.info.id);
  CHECK(refused(RgbdStreamSensor::open_file(fixed, options),
                Code::InvalidArgument, "which is fixed"));

  options.registration = registration_of("another");
  CHECK(refused(RgbdStreamSensor::open_file(tracked, options),
                Code::InvalidArgument, "not ipad's"));
  options.registration = registration_of("ipad");
  options.registration->session[15] ^= 1;
  CHECK(refused(RgbdStreamSensor::open_file(tracked, options),
                Code::InvalidArgument,
                "another session's: 8f3c0a7e5b2d4e6f9a1b0c2d3e4f5a6a, not the "
                "stream's 8f3c0a7e5b2d4e6f9a1b0c2d3e4f5a6b"));
  options.registration = registration_of("ipad");
  options.registration->world_epoch = 1;
  CHECK(refused(RgbdStreamSensor::open_file(tracked, options),
                Code::InvalidArgument, "world epoch 1"));
  options.registration = registration_of("ipad");
  options.registration->views = 0;
  CHECK(refused(RgbdStreamSensor::open_file(tracked, options),
                Code::InvalidArgument, "the registration"));
  options.registration = registration_of("ipad");
  CHECK(RgbdStreamSensor::open_file(tracked, options).ok());
  return 0;
}

// Two sensors in one session: each read by its id, neither without one.
int test_two_sensors() {
  const vkc::test::Scratch scratch("recon_link_rgbd_two_");
  const std::string path = (scratch.path / "two.vrst").string();
  const Take femto = make_take(femto_info(), 3, 0);
  const Take ipad = make_take(ipad_info(), 3, 2);
  CHECK(write_session(path, {&femto, &ipad}).ok());

  CHECK(refused(RgbdStreamSensor::open_file(path, {}), Code::InvalidArgument,
                "name one (Options::sensor)"));
  RgbdStreamSensor::Options options;
  options.sensor = "nobody";
  CHECK(refused(RgbdStreamSensor::open_file(path, options),
                Code::InvalidArgument, "no depth track of sensor nobody"));
  for (const Take* take : {&femto, &ipad}) {
    options.sensor = take->info.id;
    vkc::Result<std::unique_ptr<RgbdStreamSensor>> opened =
        RgbdStreamSensor::open_file(path, options);
    CHECK(opened.ok());
    RgbdStreamSensor& s = *opened.value();
    CHECK(same_info_less_color(s.info(), take->info));
    const vkc::Result<std::vector<sensor::RgbdFrame>> read = read_all(s);
    CHECK(read.ok() && read.value().size() == 3);
    for (std::size_t i = 0; i < 3; ++i) {
      CHECK(same_depth_half(read.value()[i], take->frames[i],
                            take->frames[i].color_to_world,
                            take->frames[i].min_confidence));
    }
    CHECK(testing::rgbd_breach(s.info(), read.value(), s.stats()).empty());
  }
  return 0;
}

// Options::min_confidence raises a record's floor, never lowers it, and gives
// none to a record without a plane.
int test_floor() {
  const vkc::test::Scratch scratch("recon_link_rgbd_floor_");
  struct Case {
    sensor::SensorInfo info;
    std::uint8_t written;
    std::uint8_t handed_out;
  };
  const Case cases[] = {
      {ipad_info(), 1, 2}, {ipad_info(), 3, 3}, {femto_info(), 0, 0}};
  int n = 0;
  for (const Case& c : cases) {
    const std::string path =
        (scratch.path / ("floor" + std::to_string(n++) + ".vrst")).string();
    const Take take = make_take(c.info, 2, c.written);
    CHECK(write_session(path, {&take}).ok());
    RgbdStreamSensor::Options options;
    options.min_confidence = 2;
    vkc::Result<std::unique_ptr<RgbdStreamSensor>> opened =
        RgbdStreamSensor::open_file(path, options);
    CHECK(opened.ok());
    RgbdStreamSensor& s = *opened.value();
    const vkc::Result<std::vector<sensor::RgbdFrame>> read = read_all(s);
    CHECK(read.ok() && read.value().size() == 2);
    for (std::size_t i = 0; i < 2; ++i) {
      CHECK(same_depth_half(read.value()[i], take.frames[i],
                            take.frames[i].color_to_world, c.handed_out));
    }
    CHECK(testing::rgbd_breach(s.info(), read.value(), s.stats()).empty());
  }
  return 0;
}

// A depth record's chunk in the file at `path`, by its seq.
struct Chunk {
  std::uint64_t offset = 0;
  std::uint64_t size = 0;  // the payload's
};

std::vector<Chunk> depth_chunks(const std::string& path) {
  std::vector<Chunk> chunks;
  const auto opened = stream::StreamReader::open(path);
  if (!opened.ok()) return chunks;
  const stream::StreamReader& reader = *opened.value();
  stream::Position pos = reader.begin();
  stream::Record record;
  for (;;) {
    const vkc::Result<bool> more = reader.next(&pos, &record);
    if (!more.ok() || !more.value()) break;
    if (record.info.track == 1) chunks.push_back({record.offset, record.size});
  }
  return chunks;
}

// The frames read from `path`, which must be those of `take` but for the
// one of seq `lost`, a span of it skipped, with its neighbours' times (0 for
// none: the first or the last), and `failed` counted.
int check_one_lost(const std::string& path, const Take& take,
                   std::uint64_t lost, std::uint64_t received,
                   std::uint64_t failed) {
  vkc::Result<std::unique_ptr<RgbdStreamSensor>> opened =
      RgbdStreamSensor::open_file(path, {});
  CHECK(opened.ok());
  RgbdStreamSensor& s = *opened.value();
  const vkc::Result<std::vector<sensor::RgbdFrame>> read = read_all(s);
  CHECK(read.ok() && read.value().size() == take.frames.size() - 1);
  std::size_t at = 0;
  for (std::size_t i = 0; i < take.frames.size(); ++i) {
    if (i == lost) continue;
    const sensor::RgbdFrame& f = read.value()[at++];
    CHECK(same_depth_half(f, take.frames[i], take.frames[i].color_to_world, 2));
    CHECK(carries_index(f));
  }
  const sensor::SensorStats stats = s.stats();
  CHECK(stats.received == received && stats.failed == failed &&
        stats.delivered == take.frames.size() - 1);
  CHECK(s.skipped().size() == 1);
  const RgbdStreamSensor::Skipped& span = s.skipped()[0];
  const std::uint64_t after = lost > 0 ? take.frames[lost - 1].timestamp_ns : 0;
  const std::uint64_t before =
      lost + 1 < take.frames.size() ? take.frames[lost + 1].timestamp_ns : 0;
  CHECK(span.first == lost && span.count == 1 && span.after_ns == after &&
        span.before_ns == before);
  CHECK(testing::rgbd_breach(s.info(), read.value(), stats).empty());
  return 0;
}

// Damage costs the records it hits, each leaving a gap and a span: in the
// middle, first or last of a finished file. A killed writer's file says
// nothing of what follows its last record.
int test_damage() {
  const vkc::test::Scratch scratch("recon_link_rgbd_damage_");
  const std::string path = (scratch.path / "whole.vrst").string();
  const Take take = make_take(ipad_info(), 8, 2);
  CHECK(write_session(path, {&take}).ok());
  const vkc::Result<std::string> whole = vkc::read_file(path);
  CHECK(whole.ok());
  const std::vector<Chunk> chunks = depth_chunks(path);
  CHECK(chunks.size() == 8);
  constexpr std::uint64_t kRecordHeader = 56;
  // The file with one byte of `whole` flipped by `mask`.
  int n = 0;
  const auto flipped = [&](std::uint64_t at, char mask) {
    std::string bytes = whole.value();
    bytes[at] ^= mask;
    const std::string damaged =
        (scratch.path / ("damaged" + std::to_string(n++) + ".vrst")).string();
    return vkc::write_file(damaged, bytes, vkc::WriteMode::CreateNew).ok()
               ? damaged
               : std::string();
  };
  const auto in_rows = [&](std::size_t record) {
    return chunks[record].offset + kRecordHeader + wire::kDepthPrefixBytes + 10;
  };
  const auto in_header = [&](std::size_t record) {
    return chunks[record].offset + 24;  // its pts
  };

  // A flipped byte in a record's rows fails its CRC: found, and failed.
  CHECK(check_one_lost(flipped(in_rows(3), 0x40), take, 3, 8, 1) == 0);
  CHECK(check_one_lost(flipped(in_rows(7), 0x40), take, 7, 8, 1) == 0);
  // A flipped byte in a record's header fails its CRC: never found.
  CHECK(check_one_lost(flipped(in_header(2), 0x01), take, 2, 7, 0) == 0);
  CHECK(check_one_lost(flipped(in_header(0), 0x01), take, 0, 7, 0) == 0);
  CHECK(check_one_lost(flipped(in_header(7), 0x01), take, 7, 7, 0) == 0);
  // Record 5 cut from the file: never found.
  {
    std::string bytes = whole.value();
    const std::uint64_t chunk = kRecordHeader + ((chunks[5].size + 7) & ~7ull);
    bytes.erase(chunks[5].offset, chunk);
    const std::string cut = (scratch.path / "cut.vrst").string();
    CHECK(vkc::write_file(cut, bytes, vkc::WriteMode::CreateNew).ok());
    CHECK(check_one_lost(cut, take, 5, 7, 0) == 0);
  }
  // A killed writer's file, its last record torn and no footer: the frames
  // before it, and no span.
  {
    const std::string torn = (scratch.path / "torn.vrst").string();
    CHECK(vkc::write_file(torn, whole.value().substr(0, in_rows(7)),
                          vkc::WriteMode::CreateNew)
              .ok());
    vkc::Result<std::unique_ptr<RgbdStreamSensor>> opened =
        RgbdStreamSensor::open_file(torn, {});
    CHECK(opened.ok());
    RgbdStreamSensor& s = *opened.value();
    const vkc::Result<std::vector<sensor::RgbdFrame>> read = read_all(s);
    CHECK(read.ok() && read.value().size() == 7);
    CHECK(read.value().back().sequence == 6 && s.stats().received == 7 &&
          s.skipped().empty());
    CHECK(testing::rgbd_breach(s.info(), read.value(), s.stats()).empty());
  }
  return 0;
}

// A file shortened after it opened: the record the cut runs through is an
// error and a loss, the read past the cut an error that ends the file, and
// the footer read at open counts the rest lost.
int test_shortened() {
  const vkc::test::Scratch scratch("recon_link_rgbd_shortened_");
  const std::string path = (scratch.path / "shortened.vrst").string();
  const Take take = make_take(ipad_info(), 6, 2);
  CHECK(write_session(path, {&take}).ok());
  const std::vector<Chunk> chunks = depth_chunks(path);
  CHECK(chunks.size() == 6);

  vkc::Result<std::unique_ptr<RgbdStreamSensor>> opened =
      RgbdStreamSensor::open_file(path, {});
  CHECK(opened.ok());
  RgbdStreamSensor& s = *opened.value();
  CHECK(s.start().ok());
  std::vector<sensor::RgbdFrame> frames;
  {
    vkc::Result<std::optional<sensor::RgbdFrame>> first = s.poll();
    CHECK(first.ok() && first.value());
    frames.push_back(std::move(*first.value()));
  }
  std::error_code error;
  std::filesystem::resize_file(path, chunks[2].offset + 100, error);
  CHECK(!error);
  std::vector<Code> errors;
  for (int polls = 0; polls < 20 && !s.exhausted(); ++polls) {
    vkc::Result<std::optional<sensor::RgbdFrame>> polled = s.poll();
    if (!polled.ok()) {
      errors.push_back(polled.status().domain());
      continue;
    }
    if (polled.value()) frames.push_back(std::move(*polled.value()));
  }
  CHECK(s.exhausted());
  CHECK(errors.size() == 2 && errors[0] == Code::IoError &&
        errors[1] == Code::IoError);
  CHECK(frames.size() == 2 && frames[1].sequence == 1);
  const sensor::SensorStats stats = s.stats();
  CHECK(stats.received == 3 && stats.delivered == 2 && stats.failed == 1);
  CHECK(s.skipped().size() == 1 && s.skipped()[0].first == 2 &&
        s.skipped()[0].count == 4 &&
        s.skipped()[0].after_ns == take.frames[1].timestamp_ns &&
        s.skipped()[0].before_ns == 0);
  CHECK(testing::rgbd_breach(s.info(), frames, stats).empty());
  {
    const auto after = s.poll();  // ended: nothing more, and no error
    CHECK(after.ok() && !after.value());
  }
  return 0;
}

// Records the frames before them would break the contract with, and records
// the decode refuses, give no frame and are counted failed.
int test_refused_records() {
  const vkc::test::Scratch scratch("recon_link_rgbd_refused_");
  const std::string path = (scratch.path / "refused.vrst").string();
  const Take take = make_take(ipad_info(), 6, 2);
  {
    vkc::Result<std::unique_ptr<stream::StreamWriter>> created =
        stream::StreamWriter::create(path, session_of({&take}));
    CHECK(created.ok());
    stream::StreamWriter& writer = *created.value();
    CHECK(append_frame(writer, 0, take, 0).ok());
    CHECK(append_frame(writer, 0, take, 1).ok());
    // Seq 2 at frame 1's time: its frame would not come after frame 1.
    Take same_time = make_take(ipad_info(), 3, 2);
    same_time.frames[2].timestamp_ns = take.frames[1].timestamp_ns;
    CHECK(append_frame(writer, 0, same_time, 2).ok());
    // Seq 3, its prefix alone: the decode refuses its size.
    const vkc::Result<wire::DepthRecord> d =
        wire::encode_depth(take.info, take.frames[3]);
    CHECK(d.ok());
    stream::RecordInfo info;
    info.track = 1;
    info.key = true;
    info.pts_ns = take.frames[3].timestamp_ns;
    info.seq = 3;
    info.set = 3;
    CHECK(writer.append(info, d.value().prefix.data(), d.value().prefix.size())
              .ok());
    CHECK(append_frame(writer, 0, take, 4).ok());
    CHECK(append_frame(writer, 0, take, 5).ok());
    CHECK(writer.finish().ok());
  }

  vkc::Result<std::unique_ptr<RgbdStreamSensor>> opened =
      RgbdStreamSensor::open_file(path, {});
  CHECK(opened.ok());
  RgbdStreamSensor& s = *opened.value();
  const vkc::Result<std::vector<sensor::RgbdFrame>> read = read_all(s);
  CHECK(read.ok() && read.value().size() == 4);
  const std::size_t kept[] = {0, 1, 4, 5};
  for (std::size_t i = 0; i < 4; ++i) {
    CHECK(same_depth_half(read.value()[i], take.frames[kept[i]],
                          take.frames[kept[i]].color_to_world, 2));
  }
  const sensor::SensorStats stats = s.stats();
  CHECK(stats.received == 6 && stats.delivered == 4 && stats.failed == 2);
  CHECK(s.skipped().size() == 1 && s.skipped()[0].first == 2 &&
        s.skipped()[0].count == 2 &&
        s.skipped()[0].after_ns == take.frames[1].timestamp_ns &&
        s.skipped()[0].before_ns == take.frames[4].timestamp_ns);
  CHECK(testing::rgbd_breach(s.info(), read.value(), stats).empty());
  return 0;
}

// A depth record to frame: its seq, and the frame of the take it holds.
struct Framed {
  std::uint64_t seq = 0;
  std::size_t frame = 0;
};

// The depth records of `take` framed as `records` lists them, after the
// session chunk: one encoder a record, so seq need not increase, as
// StreamWriter's must. No picture records, and no footer.
std::string framed(const Take& take, const std::vector<Framed>& records) {
  const stream::Session session = session_of({&take});
  vkc::Result<stream::ChunkEncoder> first =
      stream::ChunkEncoder::create(session);
  if (!first.ok()) return {};
  std::string out(first.value().session_chunk().begin(),
                  first.value().session_chunk().end());
  for (const auto& [seq, i] : records) {
    vkc::Result<stream::ChunkEncoder> encoder =
        stream::ChunkEncoder::create(session);
    const vkc::Result<wire::DepthRecord> d =
        wire::encode_depth(take.info, take.frames[i]);
    if (!encoder.ok() || !d.ok()) return {};
    const stream::RecordPayload payload = stream::RecordPayload::of(
        d.value().prefix.data(), d.value().prefix.size(), d.value().depth,
        d.value().depth_bytes, d.value().confidence,
        d.value().confidence_bytes);
    stream::RecordInfo info;
    info.track = 1;
    info.key = true;
    info.pts_ns = take.frames[i].timestamp_ns;
    info.seq = seq;
    info.set = seq;
    const vkc::Result<stream::ChunkEncoder::RecordFraming> framing =
        encoder.value().frame(info, payload);
    if (!framing.ok()) return {};
    out.append(framing.value().header.begin(), framing.value().header.end());
    out.append(reinterpret_cast<const char*>(d.value().prefix.data()),
               d.value().prefix.size());
    out.append(reinterpret_cast<const char*>(d.value().depth),
               d.value().depth_bytes);
    if (d.value().confidence != nullptr) {
      out.append(reinterpret_cast<const char*>(d.value().confidence),
                 d.value().confidence_bytes);
    }
    out.append(framing.value().padding, '\0');
  }
  return out;
}

// Records whose seq repeats or goes back, at times that still increase, give
// no frame and no gap: counted failed, nothing skipped.
int test_sequence_order() {
  const vkc::test::Scratch scratch("recon_link_rgbd_order_");
  const std::string path = (scratch.path / "order.vrst").string();
  const Take take = make_take(ipad_info(), 6, 2);
  const std::string bytes =
      framed(take, {{0, 0}, {1, 1}, {2, 2}, {2, 3}, {1, 4}, {3, 5}});
  CHECK(!bytes.empty());
  CHECK(vkc::write_file(path, bytes, vkc::WriteMode::CreateNew).ok());

  vkc::Result<std::unique_ptr<RgbdStreamSensor>> opened =
      RgbdStreamSensor::open_file(path, {});
  CHECK(opened.ok());
  RgbdStreamSensor& s = *opened.value();
  const vkc::Result<std::vector<sensor::RgbdFrame>> read = read_all(s);
  CHECK(read.ok() && read.value().size() == 4);
  const std::size_t kept[] = {0, 1, 2, 5};  // the frames, by seq 0 to 3
  for (std::size_t i = 0; i < 4; ++i) {
    const sensor::RgbdFrame& f = read.value()[i];
    CHECK(
        f.sequence == i &&
        f.timestamp_ns == take.frames[kept[i]].timestamp_ns &&
        testing::depth_index(f.depth, f.depth_camera.size.width,
                             f.depth_camera.size.height) ==
            std::optional<std::uint32_t>(static_cast<std::uint32_t>(kept[i])));
  }
  const sensor::SensorStats stats = s.stats();
  CHECK(stats.received == 6 && stats.delivered == 4 && stats.failed == 2);
  CHECK(s.skipped().empty());
  CHECK(testing::rgbd_breach(s.info(), read.value(), stats).empty());
  return 0;
}

// Opening refuses what is not a sensor this build reads.
int test_open_refusals() {
  const vkc::test::Scratch scratch("recon_link_rgbd_open_");
  const std::string missing = (scratch.path / "missing.vrst").string();
  CHECK(refused(RgbdStreamSensor::open_file(missing, {}), Code::NotFound, ""));

  const Take take = make_take(ipad_info(), 1, 2);
  const auto write = [&](const std::string& name, stream::Session session) {
    const std::string path = (scratch.path / name).string();
    vkc::Result<std::unique_ptr<stream::StreamWriter>> writer =
        stream::StreamWriter::create(path, session);
    if (writer.ok()) (void)writer.value()->finish();
    return path;
  };
  stream::Session pictures = session_of({&take});
  pictures.tracks.pop_back();  // its picture track alone
  CHECK(
      refused(RgbdStreamSensor::open_file(write("pictures.vrst", pictures), {}),
              Code::InvalidArgument, "no RgbdDepth track"));
  stream::Session later = session_of({&take});
  later.tracks[1].params[0] = 2;  // version 2
  CHECK(refused(RgbdStreamSensor::open_file(write("later.vrst", later), {}),
                Code::Unsupported, "track 1"));
  stream::Session short_params = session_of({&take});
  short_params.tracks[1].params.resize(100);
  CHECK(refused(
      RgbdStreamSensor::open_file(write("short.vrst", short_params), {}),
      Code::InvalidArgument, "track 1"));
  return 0;
}

}  // namespace

int main() {
  if (test_fixed_round_trip() != 0) return 1;
  if (test_tracked_round_trip() != 0) return 1;
  if (test_registration_refusals() != 0) return 1;
  if (test_two_sensors() != 0) return 1;
  if (test_floor() != 0) return 1;
  if (test_damage() != 0) return 1;
  if (test_shortened() != 0) return 1;
  if (test_refused_records() != 0) return 1;
  if (test_sequence_order() != 0) return 1;
  if (test_open_refusals() != 0) return 1;
  std::printf("rgbd stream sensor tests passed\n");
  return 0;
}
