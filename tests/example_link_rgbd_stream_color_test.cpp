// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// A session's sensor read back with its pictures
// (examples/common/link/rgbd_stream_sensor.hpp), on sessions written here
// through grid's StreamWriter and sensor's wire form: each colour record an
// access unit of sensor's committed H.265 clips (tools/make_hevc_fixtures.sh:
// 8 x 2 patches that shift one a frame, so a picture's luma names its frame),
// each depth record pattern depth carrying its frame's index. The pictures
// decode on this machine's hardware (VideoToolbox, NVDEC) onto the device,
// and are read back to the host. On NVIDIA the test holds the GPU's primary
// CUDA context for its cases, so no decoder makes it again.
// - Every frame goes out with its own picture: its luma names its depth's
//   index, its colour camera and exposure are its colour record's, and
//   info() names the colour camera. rgbd_breach is empty, and a restart
//   decodes it all again. Without a device the picture track is not read.
// - B-frames: pictures that come out in another order than their units went
//   in join their own depth records.
// - The decoder's refusal: the pictures it hands out before it keep their
//   frames, one poll returns it, and the file ends there.
// - Open GOPs, and a loss before a CRA: the pictures held for display come
//   out before the decoder restarts, and the CRA's leading pictures are
//   skipped.
// - A colour record lost on the way, or failing its CRC, costs the frames up
//   to the next key: counted failed, a gap and a span each, and a span of
//   the picture track. Decoding restarts at the key.
// - A depth record lost or refused lets its picture go, and the frames
//   after it keep theirs, as every access unit is decoded.
// - A record whose colour codes are not its picture's description (a BT.601
//   matrix over a BT.709 stream, full range over limited, primaries no
//   encoding names, BT.709 over an unlabelled stream decoded as BT.601)
//   gives no frame; the frames after it keep their pictures.
// - Opening with a device refuses a picture track it cannot read, which
//   opening without one leaves unread, and which pictures_optional lets go,
//   reading the depth and saying why (pictures_unread).
// Where no device path opens, the test skips; VS_TEST_HEVC_BACKEND, which CI
// sets on the legs that promise one (NVDEC on the CUDA leg, VideoToolbox on
// macOS), makes it fail instead. In a build without sensor's decoders, a
// device is refused as Unsupported.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "link/rgbd_stream_sensor.hpp"
#include "volumetric_kit/core/base/file_io.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/testing/gpu_test.hpp"
#include "volumetric_kit/core/testing/scratch_dir.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/grid/stream/chunks.hpp"
#include "volumetric_kit/grid/stream/session.hpp"
#include "volumetric_kit/grid/stream/stream_reader.hpp"
#include "volumetric_kit/grid/stream/stream_writer.hpp"
#include "volumetric_kit/sensor/frame/device_requirements.hpp"
#include "volumetric_kit/sensor/rgbd_frame.hpp"
#include "volumetric_kit/sensor/rgbd_sensor.hpp"
#include "volumetric_kit/sensor/testing/rgbd_contract.hpp"
#include "volumetric_kit/sensor/testing/rgbd_pattern.hpp"
#include "volumetric_kit/sensor/video/color_codes.hpp"
#include "volumetric_kit/sensor/video/hevc_syntax.hpp"
#include "volumetric_kit/sensor/wire/rgbd_wire.hpp"

#if VR_TEST_LINK_VIDEO
#include "volumetric_kit/sensor/prep/picture_readback.hpp"
#include "volumetric_kit/sensor/testing/hevc_clip.hpp"
#include "volumetric_kit/sensor/video/hevc_decoder.hpp"
#if !defined(__APPLE__)
#include "volumetric_kit/sensor/video/cuda_interop.hpp"
#endif
#endif

namespace {

namespace vkc = volumetric_kit::core;
namespace sensor = volumetric_kit::sensor;
namespace camera = volumetric_kit::sensor::camera;
namespace stream = volumetric_kit::grid::stream;
namespace testing = volumetric_kit::sensor::testing;
namespace wire = volumetric_kit::sensor::wire;
namespace hevc = volumetric_kit::sensor::hevc;
using Code = vkc::Status::Code;
using Gpu = vkc::test::GpuContext;
using Units = std::vector<std::vector<std::uint8_t>>;
using vr_example::link::RgbdStreamSensor;

constexpr std::uint64_t kFirstNs = 1'762'000'000'000'000'000;

camera::CameraModel pinhole(std::uint32_t width, std::uint32_t height,
                            double refined) {
  camera::CameraModel m;
  m.size = {width, height};
  m.intrinsics = {0.8 * width + refined, 0.8 * width + 0.125,
                  0.5 * width - 0.25, 0.5 * height + 0.375};
  return m;
}

// A phone's: ARKit's depth beside a picture of the clips' size, posed by its
// own tracking, written at 10 Hz.
sensor::SensorInfo phone_info() {
  sensor::SensorInfo info;
  info.id = "ipad";
  info.model = "iPad Pro";
  info.depth = pinhole(256, 192, 0.0);
  info.color = pinhole(256, 144, 0.0);
  info.role = sensor::SyncRole::FreeRun;
  info.clock = sensor::ClockDomain::Host;
  info.pose = sensor::PoseSource::Tracked;
  info.fps = 10;
  return info;
}

bool refused(const vkc::Result<std::unique_ptr<RgbdStreamSensor>>& opened,
             Code code, const std::string& says) {
  if (opened.ok()) return false;
  const std::string& message = opened.status().message();
  if (opened.status().domain() != code ||
      message.find(says) == std::string::npos) {
    std::fprintf(stderr, "refused as: %s\n", message.c_str());
    return false;
  }
  return message.rfind("RgbdStreamSensor::open_file: ", 0) == 0;
}

RgbdStreamSensor::Options on_device(const Gpu& gpu) {
  RgbdStreamSensor::Options options;
  options.device = &gpu.device;
  options.allocator = &gpu.allocator;
  return options;
}

// The picture track (0) and the depth track (1) of `info`'s sensor.
stream::Session session_of(const sensor::SensorInfo& info) {
  stream::Session session;
  session.id = {0x8f, 0x3c, 0x0a, 0x7e, 0x5b, 0x2d, 0x4e, 0x6f,
                0x9a, 0x1b, 0x0c, 0x2d, 0x3e, 0x4f, 0x5a, 0x6b};
  session.created_ns = kFirstNs;
  session.writer = "recon_example_link_rgbd_stream_color_test";
  stream::Track color;
  color.kind = stream::TrackKind::RgbdColor;
  color.clock = stream::kHostClock;
  color.rate = {info.fps, 1};
  color.width = 256;
  color.height = 144;
  color.sensor = info.id;
  color.params = wire::encode_color_params(wire::ColorCodec::HevcMain8);
  stream::Track depth = color;
  depth.kind = stream::TrackKind::RgbdDepth;
  depth.width = 256;
  depth.height = 192;
  const vkc::Result<std::string> params = wire::encode_sensor(info);
  if (params.ok()) depth.params = params.value();
  session.tracks = {color, depth};
  return session;
}

// A session of `info`'s sensor whose tracks `change` alters, with no
// records.
template <typename Change>
std::string session_file(const vkc::test::Scratch& scratch,
                         const std::string& name, sensor::SensorInfo info,
                         Change change) {
  stream::Session session = session_of(info);
  change(session);
  const std::string path = (scratch.path / name).string();
  vkc::Result<std::unique_ptr<stream::StreamWriter>> writer =
      stream::StreamWriter::create(path, session);
  if (writer.ok()) (void)writer.value()->finish();
  return path;
}

#if VR_TEST_LINK_VIDEO

constexpr std::uint64_t kPeriodNs = 100'000'000;  // 10 Hz
// What a frame's colour is to wire::encode_color, which asks only that it
// has one: the pictures are the clip's.
constexpr std::uint8_t kSomeColor = 0;

// What the patch clips state: BT.709 limited range.
sensor::ColorCodes bt709() {
  sensor::ColorCodes codes;
  codes.matrix = 1;
  codes.transfer = 1;
  codes.primaries = 1;
  return codes;
}

// A session to write: its frames, in capture order, and its picture track's
// access units in decode order, each with the frame it shows and the codes
// its record states.
struct Take {
  sensor::SensorInfo info = phone_info();
  std::vector<testing::PatternDepth> depth;
  std::vector<sensor::RgbdFrame> frames;
  Units units;
  std::vector<std::size_t> shows;
  std::vector<sensor::ColorCodes> codes;
};

// Each frame with pattern depth of its index and a plane at floor 2, and a
// colour camera, a pose and an exposure of its own.
Take make_take(Units units, std::vector<std::size_t> shows,
               const sensor::ColorCodes& codes) {
  Take take;
  const std::size_t count = units.size();
  for (std::size_t i = 0; i < count; ++i) {
    take.depth.push_back(
        testing::pattern_depth(256, 192, static_cast<std::uint32_t>(i)));
  }
  for (std::size_t i = 0; i < count; ++i) {
    const double k = static_cast<double>(i);
    sensor::RgbdFrame f;
    f.depth = take.depth[i].depth.data();
    f.depth_confidence = take.depth[i].confidence.data();
    f.min_confidence = 2;
    f.depth_camera = *take.info.depth;
    f.metres_per_unit = 0.001f;
    f.min_depth = 0.25f;
    f.max_depth = 5.0f;
    f.depth_noise = {0.001f, 0.01f, 0.0f};
    f.color.host = &kSomeColor;
    f.color.width = 256;
    f.color.height = 144;
    f.color_camera = pinhole(256, 144, 0.5 * k);
    f.color_exposure = {1.0f / 120 + 1e-4f * static_cast<float>(i),
                        100.0f + static_cast<float>(i),
                        -0.5f + 0.125f * static_cast<float>(i)};
    f.color_to_world = camera::Mat4d(1.0);
    f.color_to_world[3][0] = 0.05 * k;
    f.timestamp_ns = kFirstNs + i * kPeriodNs;
    f.sequence = i;
    take.frames.push_back(f);
  }
  take.units = std::move(units);
  take.shows = std::move(shows);
  take.codes.assign(count, codes);
  return take;
}

std::vector<std::size_t> in_order(std::size_t count) {
  std::vector<std::size_t> shows(count);
  for (std::size_t i = 0; i < count; ++i) shows[i] = i;
  return shows;
}

// Records the writer leaves out, as lost on the way: colour records by
// their unit, depth records by their frame.
struct Lost {
  std::vector<std::size_t> units;
  std::vector<std::size_t> frames;
};

bool among(const std::vector<std::size_t>& list, std::size_t i) {
  return std::find(list.begin(), list.end(), i) != list.end();
}

// A finished session of `take`: for each unit in decode order, its colour
// record (its seq the unit's, its set and pts the frame it shows), then the
// depth record of the frame of that number, but for those `lost`.
vkc::Status write_session(const std::string& path, const Take& take,
                          const Lost& lost = {}) {
  VKC_ASSIGN(auto writer,
             stream::StreamWriter::create(path, session_of(take.info)));
  for (std::size_t k = 0; k < take.units.size(); ++k) {
    if (!among(lost.units, k)) {
      const sensor::RgbdFrame& shown = take.frames[take.shows[k]];
      VKC_ASSIGN(const auto prefix,
                 wire::encode_color(take.info, shown, take.codes[k]));
      const std::vector<std::uint8_t>& unit = take.units[k];
      const std::optional<int> type =
          hevc::first_slice_type(unit.data(), unit.size());
      stream::RecordInfo info;
      info.track = 0;
      info.key = type && hevc::is_irap(*type);
      info.pts_ns = shown.timestamp_ns;
      info.seq = k;
      info.set = take.shows[k];
      VKC_TRY(writer->append(
          info, stream::RecordPayload::of(prefix.data(), prefix.size(),
                                          unit.data(), unit.size())));
    }
    if (k < take.frames.size() && !among(lost.frames, k)) {
      const sensor::RgbdFrame& f = take.frames[k];
      VKC_ASSIGN(const wire::DepthRecord d, wire::encode_depth(take.info, f));
      stream::RecordInfo info;
      info.track = 1;
      info.key = true;
      info.pts_ns = f.timestamp_ns;
      info.seq = k;
      info.set = k;
      VKC_TRY(writer->append(
          info, stream::RecordPayload::of(d.prefix.data(), d.prefix.size(),
                                          d.depth, d.depth_bytes, d.confidence,
                                          d.confidence_bytes)));
    }
  }
  return writer->finish();
}

// A copy of the file at `path` at `to`, a byte of the payload of `track`'s
// record `seq` flipped, `at` bytes in: its CRC fails.
vkc::Status damage(const std::string& path, const std::string& to,
                   std::uint16_t track, std::uint64_t seq, std::uint64_t at) {
  VKC_ASSIGN(std::string bytes, vkc::read_file(path));
  VKC_ASSIGN(const auto reader, stream::StreamReader::open(path));
  stream::Position pos = reader->begin();
  stream::Record record;
  for (;;) {
    VKC_ASSIGN(const bool more, reader->next(&pos, &record));
    if (!more) return vkc::Status::not_found("no such record");
    if (record.info.track == track && record.info.seq == seq) break;
  }
  constexpr std::uint64_t kRecordHeader = 56;
  bytes[record.offset + kRecordHeader + at] ^= 0x40;
  return vkc::write_file(to, bytes, vkc::WriteMode::CreateNew);
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
  return std::memcmp(a, b, size) == 0;
}

bool same_camera(const camera::CameraModel& a, const camera::CameraModel& b) {
  static_assert(sizeof(camera::CameraModel) == 104, "no padding to compare");
  return same_bytes(&a, &b, sizeof a);
}

// `spans` are `want`, field for field.
bool same_spans(const std::vector<RgbdStreamSensor::Skipped>& spans,
                const std::vector<RgbdStreamSensor::Skipped>& want) {
  if (spans.size() != want.size()) return false;
  for (std::size_t i = 0; i < want.size(); ++i) {
    const RgbdStreamSensor::Skipped& a = spans[i];
    const RgbdStreamSensor::Skipped& b = want[i];
    if (a.first != b.first || a.count != b.count || a.after_ns != b.after_ns ||
        a.before_ns != b.before_ns) {
      std::fprintf(stderr, "span %zu: {%llu, %llu} not {%llu, %llu}\n", i,
                   static_cast<unsigned long long>(a.first),
                   static_cast<unsigned long long>(a.count),
                   static_cast<unsigned long long>(b.first),
                   static_cast<unsigned long long>(b.count));
      return false;
    }
  }
  return true;
}

std::uint64_t time_of(std::size_t frame) {
  return kFirstNs + frame * kPeriodNs;
}

constexpr const char* kPatches = VR_HEVC_DATA "/patches_256x144.h265";
constexpr const char* kUnlabelled = VR_HEVC_DATA "/unlabelled_256x144.h265";
// The patch frames with B-frames, then two of 4:0:0 grey.
constexpr const char* kBFrames = VR_HEVC_DATA "/refused_256x144.h265";
// 16 patch frames in open GOPs: a CRA every 6, the two B-frames shown before
// each sent after it as its leading (RASL) pictures, predicted across it.
constexpr const char* kOpenGop = VR_HEVC_DATA "/open_gop_256x144.h265";

Units units_of(const char* clip) { return testing::access_units_of_file(clip); }

// The patch clip twice over: 16 frames, a key every 4.
Units patches16() {
  Units units = units_of(kPatches);
  const Units again = units;
  units.insert(units.end(), again.begin(), again.end());
  return units;
}

// What the decoder makes of `units`, each sent with its index for its pts
// and its pictures taken, then the end, up to the first failure: the units
// whose pictures it hands out, in that order, those ready at a failed send
// included; how many units it took; and the failure.
struct Decoding {
  std::vector<std::size_t> shown;
  std::size_t taken = 0;
  vkc::Status failure;
};

Decoding decode_units(const Gpu& gpu, const Units& units) {
  Decoding out;
  sensor::HevcDecoder::Options options;
  options.device = &gpu.device;
  options.allocator = &gpu.allocator;
  auto decoder = sensor::HevcDecoder::create(options);
  if (!decoder) {
    out.failure = decoder.status();
    return out;
  }
  const auto drain = [&]() -> vkc::Status {
    for (;;) {
      VKC_ASSIGN(const std::optional<sensor::DecodedPicture> picture,
                 decoder->receive());
      if (!picture) return {};
      out.shown.push_back(static_cast<std::size_t>(picture->pts));
    }
  };
  for (std::size_t i = 0; i <= units.size(); ++i) {
    const bool end = i == units.size();
    const vkc::Status sent =
        decoder->send(end ? nullptr : units[i].data(),
                      end ? 0 : units[i].size(), static_cast<std::int64_t>(i));
    const vkc::Status drained = drain();
    out.failure = sent.ok() ? drained : sent;
    if (!sent.ok()) break;
    if (!end) ++out.taken;
    if (!drained.ok()) break;
  }
  return out;
}

// The frame each of `units` shows: its picture's place in display order.
// Empty unless the decoder hands out every unit's picture.
std::vector<std::size_t> display_order(const Gpu& gpu, const Units& units) {
  const Decoding decoding = decode_units(gpu, units);
  if (!decoding.failure.ok() || decoding.shown.size() != units.size()) {
    return {};
  }
  std::vector<std::size_t> shows(units.size(), units.size());
  for (std::size_t place = 0; place < units.size(); ++place) {
    const std::size_t k = decoding.shown[place];
    if (k >= units.size() || shows[k] != units.size()) return {};
    shows[k] = place;
  }
  return shows;
}

// The frame a patch clip's picture shows, modulo 8: patch (0, 0) is
// 40 + 24 (frame mod 8) in luma, read from its middle. Empty for a picture
// that shows no patch value there.
std::optional<std::size_t> luma_index(sensor::PictureReadback& readback,
                                      const sensor::RgbdFrame& frame) {
  sensor::YuvImage picture = frame.color;
  const vkc::Result<sensor::PictureView> view =
      readback.read(picture, sensor::PictureReadback::Planes::Luma);
  if (!view.ok() || view->width != 256 || view->height != 144) return {};
  double sum = 0.0;
  for (std::uint32_t y = 28; y < 44; ++y) {
    for (std::uint32_t x = 8; x < 24; ++x)
      sum += view->luma[y * view->stride + x];
  }
  const double patch = (sum / 256.0 - 40.0) / 24.0;
  const double nearest = std::round(patch);
  if (nearest < 0.0 || nearest > 7.0 || std::abs(patch - nearest) > 0.25) {
    return {};
  }
  return static_cast<std::size_t>(nearest);
}

// `f` is frame `f.sequence` of `take`, whole: its depth and its own
// picture, read back, with its colour record's camera and exposure, decoded
// with the weights `kr` and `kb`.
int check_frame(sensor::PictureReadback& readback, const sensor::RgbdFrame& f,
                const Take& take, float kr = 0.2126f, float kb = 0.0722f) {
  const auto i = static_cast<std::size_t>(f.sequence);
  CHECK(i < take.frames.size());
  const sensor::RgbdFrame& written = take.frames[i];
  CHECK(testing::depth_index(f.depth, f.depth_camera.size.width,
                             f.depth_camera.size.height) ==
        std::optional<std::uint32_t>(static_cast<std::uint32_t>(i)));
  CHECK(f.timestamp_ns == written.timestamp_ns);
  CHECK(f.color.image[0] != nullptr && f.color.image[1] != nullptr &&
        f.color.host == nullptr);
  CHECK(f.color.width == 256 && f.color.height == 144);
  CHECK(f.color.kr == kr && f.color.kb == kb && !f.color.full_range);
  CHECK(same_camera(f.color_camera, written.color_camera));
  CHECK(same_bytes(&f.color_exposure, &written.color_exposure,
                   sizeof f.color_exposure));
  CHECK(f.color_encoding ==
        (sensor::ColorEncoding{sensor::ColorEncoding::Transfer::Bt709,
                               sensor::ColorEncoding::Primaries::Bt709}));
  const std::optional<std::size_t> shows = luma_index(readback, f);
  if (shows != std::optional<std::size_t>(i % 8)) {
    std::fprintf(stderr, "frame %zu shows picture %d\n", i,
                 shows ? static_cast<int>(*shows) : -1);
    return 1;
  }
  return 0;
}

// The frames `s` handed out are those of `take` numbered `kept`, each whole.
int check_kept(sensor::PictureReadback& readback,
               const std::vector<sensor::RgbdFrame>& frames, const Take& take,
               const std::vector<std::size_t>& kept) {
  CHECK(frames.size() == kept.size());
  for (std::size_t i = 0; i < kept.size(); ++i) {
    CHECK(frames[i].sequence == kept[i]);
    if (check_frame(readback, frames[i], take) != 0) return 1;
  }
  return 0;
}

std::vector<std::size_t> all_but(std::size_t count,
                                 const std::vector<std::size_t>& gone) {
  std::vector<std::size_t> kept;
  for (std::size_t i = 0; i < count; ++i) {
    if (!among(gone, i)) kept.push_back(i);
  }
  return kept;
}

// The spans of `count` depth records once the file has ended, the frames
// `kept` (in order) handed out.
std::vector<RgbdStreamSensor::Skipped> spans_around(
    const std::vector<std::size_t>& kept, std::size_t count) {
  std::vector<RgbdStreamSensor::Skipped> spans;
  std::size_t next = 0;
  std::uint64_t after = 0;
  for (const std::size_t k : kept) {
    if (k > next) spans.push_back({next, k - next, after, time_of(k)});
    next = k + 1;
    after = time_of(k);
  }
  if (count > next) spans.push_back({next, count - next, after, 0});
  return spans;
}

// Every frame with its own picture, the colour camera named, the contract
// kept; again after a restart; and with no device, the depth alone.
int test_round_trip(const Gpu& gpu, sensor::PictureReadback& readback) {
  const vkc::test::Scratch scratch("recon_link_rgbd_color_trip_");
  const std::string path = (scratch.path / "phone.vrst").string();
  const Take take = make_take(patches16(), in_order(16), bt709());
  CHECK(write_session(path, take).ok());

  vkc::Result<std::unique_ptr<RgbdStreamSensor>> opened =
      RgbdStreamSensor::open_file(path, on_device(gpu));
  CHECK(opened.ok());
  RgbdStreamSensor& s = *opened.value();
  CHECK(s.info().color && same_camera(*s.info().color, *take.info.color));
  for (int pass = 0; pass < 2; ++pass) {
    vkc::Result<std::vector<sensor::RgbdFrame>> read = read_all(s);
    CHECK(read.ok());
    if (check_kept(readback, read.value(), take, in_order(16)) != 0) return 1;
    CHECK(testing::rgbd_breach(s.info(), read.value(), s.stats()).empty());
    const sensor::SensorStats stats = s.stats();
    CHECK(stats.received == 16 && stats.delivered == 16 && stats.failed == 0);
    CHECK(s.skipped().empty() && s.skipped_color().empty());
    s.stop();  // and read it all again, decoded afresh
  }

  vkc::Result<std::unique_ptr<RgbdStreamSensor>> depth_alone =
      RgbdStreamSensor::open_file(path, {});
  CHECK(depth_alone.ok());
  RgbdStreamSensor& d = *depth_alone.value();
  CHECK(!d.info().color);
  const vkc::Result<std::vector<sensor::RgbdFrame>> read = read_all(d);
  CHECK(read.ok() && read.value().size() == 16);
  for (const sensor::RgbdFrame& f : read.value()) CHECK(!f.has_color());
  CHECK(testing::rgbd_breach(d.info(), read.value(), d.stats()).empty());
  CHECK(d.skipped_color().empty());
  return 0;
}

// B-frames: the units go in decode order and their pictures come out in
// display order, each joined to its own frame. Unit 3 lost on the way, a
// picture nothing refers to (TRAIL_N): the decoder could go on, but the
// numbers show a loss, so the gate waits for a key, and none comes. The
// frames units 0 to 2 show keep their pictures; the rest go.
int test_b_frames(const Gpu& gpu, sensor::PictureReadback& readback) {
  const vkc::test::Scratch scratch("recon_link_rgbd_color_b_");
  Units units = units_of(kBFrames);
  CHECK(units.size() == 10);
  units.resize(8);  // the patch frames; the grey ones after are 4:0:0
  std::vector<std::size_t> shows = display_order(gpu, units);
  CHECK(shows.size() == 8 && !std::is_sorted(shows.begin(), shows.end()));
  const std::optional<int> unreferenced =
      hevc::first_slice_type(units[3].data(), units[3].size());
  CHECK(unreferenced == std::optional<int>(0));  // TRAIL_N
  const Take take = make_take(units, std::move(shows), bt709());
  for (const bool lose : {false, true}) {
    const std::string path =
        (scratch.path / (lose ? "lost.vrst" : "whole.vrst")).string();
    CHECK(write_session(
              path, take,
              {lose ? std::vector<std::size_t>{3} : std::vector<std::size_t>{},
               {}})
              .ok());
    vkc::Result<std::unique_ptr<RgbdStreamSensor>> opened =
        RgbdStreamSensor::open_file(path, on_device(gpu));
    CHECK(opened.ok());
    RgbdStreamSensor& s = *opened.value();
    const vkc::Result<std::vector<sensor::RgbdFrame>> read = read_all(s);
    CHECK(read.ok());
    CHECK(testing::rgbd_breach(s.info(), read.value(), s.stats()).empty());
    CHECK(s.stats().received == 8);
    if (!lose) {
      if (check_kept(readback, read.value(), take, in_order(8)) != 0) return 1;
      CHECK(s.stats().failed == 0 && s.skipped().empty() &&
            s.skipped_color().empty());
      continue;
    }
    std::vector<std::size_t> kept(take.shows.begin(), take.shows.begin() + 3);
    std::sort(kept.begin(), kept.end());
    if (check_kept(readback, read.value(), take, kept) != 0) return 1;
    CHECK(s.stats().failed == 5);
    CHECK(same_spans(s.skipped_color(), {{3, 5, time_of(take.shows[2]), 0}}));
  }
  return 0;
}

// The whole refused clip: the decoder refuses the stream at unit 8, the
// first grey one, with pictures still held for display. The pictures it
// hands out before the refusal keep their frames (VideoToolbox's: all 8),
// one poll returns the refusal, and the file ends there.
int test_refused(const Gpu& gpu, sensor::PictureReadback& readback) {
  const vkc::test::Scratch scratch("recon_link_rgbd_color_refused_");
  const std::string path = (scratch.path / "refused.vrst").string();
  const Units units = units_of(kBFrames);
  CHECK(units.size() == 10);
  std::vector<std::size_t> shows =
      display_order(gpu, Units(units.begin(), units.begin() + 8));
  CHECK(shows.size() == 8);
  shows.push_back(8);
  shows.push_back(9);
  const Decoding decoding = decode_units(gpu, units);
  CHECK(decoding.failure.domain() == Code::Unsupported && decoding.taken == 8);
  std::vector<std::size_t> kept;
  for (const std::size_t unit : decoding.shown) kept.push_back(shows[unit]);
  std::sort(kept.begin(), kept.end());
  CHECK(!kept.empty());
  const Take take = make_take(units, shows, bt709());
  CHECK(write_session(path, take).ok());

  vkc::Result<std::unique_ptr<RgbdStreamSensor>> opened =
      RgbdStreamSensor::open_file(path, on_device(gpu));
  CHECK(opened.ok());
  RgbdStreamSensor& s = *opened.value();
  CHECK(s.start().ok());
  std::vector<sensor::RgbdFrame> frames;
  std::vector<vkc::Status> errors;
  for (int polls = 0; !s.exhausted(); ++polls) {
    CHECK(polls < 100);
    vkc::Result<std::optional<sensor::RgbdFrame>> polled = s.poll();
    if (!polled.ok()) {
      errors.push_back(polled.status());
    } else if (polled.value()) {
      frames.push_back(std::move(*polled.value()));
    }
  }
  CHECK(errors.size() == 1 && errors[0].domain() == Code::Unsupported &&
        errors[0].message().find("decoding sensor ipad's pictures") !=
            std::string::npos);
  if (check_kept(readback, frames, take, kept) != 0) return 1;
  const sensor::SensorStats stats = s.stats();
  CHECK(stats.received == 8 && stats.delivered == kept.size() &&
        stats.failed == 8 - kept.size());
  CHECK(same_spans(s.skipped(), spans_around(kept, 10)));
  CHECK(same_spans(s.skipped_color(), {{8, 2, time_of(shows[7]), 0}}));
  CHECK(testing::rgbd_breach(s.info(), frames, stats).empty());
  return 0;
}

// Open GOPs: every frame keeps its picture. Then the unit before the last
// CRA lost on the way: its frame goes; the pictures held back for display,
// decoded before the loss, come out before the decoder restarts at the CRA;
// and the CRA's leading pictures, predicted from the lost one, are skipped,
// their frames gone, with no picture of another frame's.
int test_open_gop(const Gpu& gpu, sensor::PictureReadback& readback) {
  const vkc::test::Scratch scratch("recon_link_rgbd_color_gop_");
  const Units units = units_of(kOpenGop);
  std::vector<std::size_t> shows = display_order(gpu, units);
  CHECK(units.size() == 16 && shows.size() == 16);
  const auto type_of = [&](std::size_t k) {
    return hevc::first_slice_type(units[k].data(), units[k].size())
        .value_or(-1);
  };
  std::size_t cra = 0;
  for (std::size_t k = 0; k < units.size(); ++k) {
    if (type_of(k) == hevc::kCraNut) cra = k;
  }
  CHECK(cra >= 2);
  const std::size_t lost = cra - 1;
  // Units before the lost one show frames after its: held back at the loss.
  CHECK(std::any_of(shows.begin(),
                    shows.begin() + static_cast<std::ptrdiff_t>(lost),
                    [&](std::size_t f) { return f > shows[lost]; }));
  std::vector<std::size_t> gone = {shows[lost]};
  for (std::size_t k = cra + 1; k < units.size() && hevc::is_rasl(type_of(k));
       ++k) {
    gone.push_back(shows[k]);
  }
  CHECK(gone.size() == 3);
  const Take take = make_take(units, shows, bt709());
  for (const bool lose : {false, true}) {
    const std::string path =
        (scratch.path / (lose ? "lost.vrst" : "whole.vrst")).string();
    CHECK(write_session(path, take,
                        {lose ? std::vector<std::size_t>{lost}
                              : std::vector<std::size_t>{},
                         {}})
              .ok());
    vkc::Result<std::unique_ptr<RgbdStreamSensor>> opened =
        RgbdStreamSensor::open_file(path, on_device(gpu));
    CHECK(opened.ok());
    RgbdStreamSensor& s = *opened.value();
    const vkc::Result<std::vector<sensor::RgbdFrame>> read = read_all(s);
    CHECK(read.ok());
    CHECK(testing::rgbd_breach(s.info(), read.value(), s.stats()).empty());
    CHECK(s.stats().received == 16);
    const std::vector<std::size_t> kept =
        lose ? all_but(16, gone) : in_order(16);
    if (check_kept(readback, read.value(), take, kept) != 0) return 1;
    CHECK(same_spans(s.skipped(), spans_around(kept, 16)));
    if (!lose) {
      CHECK(s.skipped_color().empty());
      continue;
    }
    CHECK(same_spans(s.skipped_color(), {{lost, 1, time_of(shows[lost - 1]),
                                          time_of(shows[cra])}}));
  }
  return 0;
}

// A colour record lost on the way (5) or failing its CRC (13): the frames
// up to the next key (8; none after 13) go, each counted, a gap and a span,
// and the picture track's spans name the units not decoded. Every frame kept
// shows its own picture, never a neighbour's.
int test_lost_picture(const Gpu& gpu, sensor::PictureReadback& readback) {
  const vkc::test::Scratch scratch("recon_link_rgbd_color_lost_");
  const std::string lost = (scratch.path / "lost.vrst").string();
  const std::string path = (scratch.path / "damaged.vrst").string();
  const Take take = make_take(patches16(), in_order(16), bt709());
  CHECK(take.units[13].size() > 20);  // the byte flipped is in its unit
  CHECK(write_session(lost, take, {{5}, {}}).ok());
  CHECK(damage(lost, path, 0, 13, wire::kColorPrefixBytes + 20).ok());

  vkc::Result<std::unique_ptr<RgbdStreamSensor>> opened =
      RgbdStreamSensor::open_file(path, on_device(gpu));
  CHECK(opened.ok());
  RgbdStreamSensor& s = *opened.value();
  const vkc::Result<std::vector<sensor::RgbdFrame>> read = read_all(s);
  CHECK(read.ok());
  if (check_kept(readback, read.value(), take,
                 all_but(16, {5, 6, 7, 13, 14, 15})) != 0) {
    return 1;
  }
  const sensor::SensorStats stats = s.stats();
  CHECK(stats.received == 16 && stats.delivered == 10 && stats.failed == 6);
  CHECK(same_spans(s.skipped(),
                   {{5, 3, time_of(4), time_of(8)}, {13, 3, time_of(12), 0}}));
  CHECK(same_spans(s.skipped_color(),
                   {{5, 3, time_of(4), time_of(8)}, {13, 3, time_of(12), 0}}));
  CHECK(testing::rgbd_breach(s.info(), read.value(), stats).empty());
  return 0;
}

// A depth record lost on the way (2) or failing its CRC (6): its frame
// goes, its picture let go, and the next frame, predicted from that picture,
// keeps its own.
int test_lost_depth(const Gpu& gpu, sensor::PictureReadback& readback) {
  const vkc::test::Scratch scratch("recon_link_rgbd_color_depth_");
  const std::string lost = (scratch.path / "lost.vrst").string();
  const std::string path = (scratch.path / "damaged.vrst").string();
  const Take take = make_take(patches16(), in_order(16), bt709());
  CHECK(write_session(lost, take, {{}, {2}}).ok());
  CHECK(damage(lost, path, 1, 6, wire::kDepthPrefixBytes + 20).ok());

  vkc::Result<std::unique_ptr<RgbdStreamSensor>> opened =
      RgbdStreamSensor::open_file(path, on_device(gpu));
  CHECK(opened.ok());
  RgbdStreamSensor& s = *opened.value();
  const vkc::Result<std::vector<sensor::RgbdFrame>> read = read_all(s);
  CHECK(read.ok());
  if (check_kept(readback, read.value(), take, all_but(16, {2, 6})) != 0) {
    return 1;
  }
  const sensor::SensorStats stats = s.stats();
  CHECK(stats.received == 15 && stats.delivered == 14 && stats.failed == 1);
  CHECK(same_spans(s.skipped(), {{2, 1, time_of(1), time_of(3)},
                                 {6, 1, time_of(5), time_of(7)}}));
  CHECK(s.skipped_color().empty());
  CHECK(testing::rgbd_breach(s.info(), read.value(), stats).empty());
  return 0;
}

// Records whose codes are not what their pictures decode as give no frame:
// the frame goes, counted, and the frames after it keep their pictures,
// predicted from its. An unlabelled stream decodes as BT.601 at 144 rows:
// records that state BT.709 give none, records that leave it unspecified
// give every frame, in BT.601.
int test_description(const Gpu& gpu, sensor::PictureReadback& readback) {
  const vkc::test::Scratch scratch("recon_link_rgbd_color_codes_");
  {
    const std::string path = (scratch.path / "codes.vrst").string();
    Take take = make_take(patches16(), in_order(16), bt709());
    take.codes[1].matrix = 6;         // BT.601 over the stream's BT.709
    take.codes[9].full_range = true;  // over its limited range
    take.codes[10].primaries = 6;     // SD primaries: no encoding names them
    CHECK(write_session(path, take).ok());
    vkc::Result<std::unique_ptr<RgbdStreamSensor>> opened =
        RgbdStreamSensor::open_file(path, on_device(gpu));
    CHECK(opened.ok());
    RgbdStreamSensor& s = *opened.value();
    const vkc::Result<std::vector<sensor::RgbdFrame>> read = read_all(s);
    CHECK(read.ok());
    if (check_kept(readback, read.value(), take, all_but(16, {1, 9, 10})) !=
        0) {
      return 1;
    }
    const sensor::SensorStats stats = s.stats();
    CHECK(stats.received == 16 && stats.delivered == 13 && stats.failed == 3);
    CHECK(same_spans(s.skipped(), {{1, 1, time_of(0), time_of(2)},
                                   {9, 2, time_of(8), time_of(11)}}));
    CHECK(s.skipped_color().empty());
    CHECK(testing::rgbd_breach(s.info(), read.value(), stats).empty());
  }
  int n = 0;
  for (const bool stated : {true, false}) {
    const std::string path =
        (scratch.path / ("unlabelled" + std::to_string(n++) + ".vrst"))
            .string();
    const Take take = make_take(units_of(kUnlabelled), in_order(8),
                                stated ? bt709() : sensor::ColorCodes{});
    CHECK(write_session(path, take).ok());
    vkc::Result<std::unique_ptr<RgbdStreamSensor>> opened =
        RgbdStreamSensor::open_file(path, on_device(gpu));
    CHECK(opened.ok());
    RgbdStreamSensor& s = *opened.value();
    const vkc::Result<std::vector<sensor::RgbdFrame>> read = read_all(s);
    CHECK(read.ok());
    const sensor::SensorStats stats = s.stats();
    CHECK(testing::rgbd_breach(s.info(), read.value(), stats).empty());
    if (stated) {
      CHECK(read.value().empty() && stats.received == 8 && stats.failed == 8);
      CHECK(same_spans(s.skipped(), {{0, 8, 0, 0}}));
      continue;
    }
    CHECK(read.value().size() == 8 && stats.failed == 0);
    for (const sensor::RgbdFrame& f : read.value()) {
      if (check_frame(readback, f, take, 0.299f, 0.114f) != 0) return 1;
    }
  }
  return 0;
}

// With a device, a picture track it cannot read is refused; without one it
// is not read, and the depth opens; under pictures_optional it is let go,
// the depth opens, and pictures_unread says why.
int test_open_refusals(const Gpu& gpu) {
  const vkc::test::Scratch scratch("recon_link_rgbd_color_open_");
  struct Case {
    std::string name;
    sensor::SensorInfo info;
    void (*change)(stream::Session&);
    Code code;
    std::string says;
  };
  sensor::SensorInfo no_color = phone_info();
  no_color.color.reset();
  const Case cases[] = {
      {"clock.vrst", phone_info(),
       [](stream::Session& s) { s.tracks[0].clock = 1; }, Code::InvalidArgument,
       "colour on clock 1, its depth on clock 0"},
      {"version.vrst", phone_info(),
       [](stream::Session& s) { s.tracks[0].params[0] = 2; }, Code::Unsupported,
       "track 0"},
      {"codec.vrst", phone_info(),
       [](stream::Session& s) { s.tracks[0].params[2] = 2; }, Code::Unsupported,
       "track 0"},
      {"two.vrst", phone_info(),
       [](stream::Session& s) { s.tracks.push_back(s.tracks[0]); },
       Code::InvalidArgument, "several picture tracks of sensor ipad"},
      {"no_camera.vrst", no_color, [](stream::Session&) {},
       Code::InvalidArgument, "whose params name no colour camera"},
  };
  for (const Case& c : cases) {
    const std::string path = session_file(scratch, c.name, c.info, c.change);
    CHECK(refused(RgbdStreamSensor::open_file(path, on_device(gpu)), c.code,
                  c.says));
    vkc::Result<std::unique_ptr<RgbdStreamSensor>> depth_alone =
        RgbdStreamSensor::open_file(path, {});
    CHECK(depth_alone.ok() && !depth_alone.value()->info().color &&
          depth_alone.value()->pictures_unread().empty());
    RgbdStreamSensor::Options optional = on_device(gpu);
    optional.pictures_optional = true;
    vkc::Result<std::unique_ptr<RgbdStreamSensor>> let_go =
        RgbdStreamSensor::open_file(path, optional);
    CHECK(let_go.ok() && !let_go.value()->info().color &&
          let_go.value()->pictures_unread().find(c.says) != std::string::npos);
  }
  // A session with no picture track reads as depth alone, with a device too.
  const std::string depth_only = session_file(
      scratch, "depth_only.vrst", phone_info(),
      [](stream::Session& s) { s.tracks.erase(s.tracks.begin()); });
  vkc::Result<std::unique_ptr<RgbdStreamSensor>> opened =
      RgbdStreamSensor::open_file(depth_only, on_device(gpu));
  CHECK(opened.ok() && !opened.value()->info().color &&
        opened.value()->pictures_unread().empty());
  // One it reads is read under pictures_optional too, nothing said.
  const std::string readable = session_file(
      scratch, "readable.vrst", phone_info(), [](stream::Session&) {});
  RgbdStreamSensor::Options optional = on_device(gpu);
  optional.pictures_optional = true;
  vkc::Result<std::unique_ptr<RgbdStreamSensor>> read =
      RgbdStreamSensor::open_file(readable, optional);
  CHECK(read.ok() && read.value()->info().color &&
        read.value()->pictures_unread().empty());
  return 0;
}

int gpu_main(Gpu& gpu) {
#if !defined(__APPLE__)
  // The GPU's primary CUDA context, held for the run, the probe's decoder
  // included, as sensor's decoder tests hold it: each decoder retains it,
  // and one let go by its last holder is made again by the next, which took
  // most of the test's time. Where CUDA does not reach the device, the
  // probe below says whether that skips or fails.
  vkc::Result<std::unique_ptr<sensor::CudaContext>> held =
      sensor::CudaContext::open(gpu.device);
  if (!held.ok() && held.status().domain() != Code::Unsupported) {
    std::fprintf(stderr, "FAIL: %s\n", held.status().message().c_str());
    return 1;
  }
#endif
  {
    sensor::HevcDecoder::Options options;
    options.device = &gpu.device;
    options.allocator = &gpu.allocator;
    auto probe = sensor::HevcDecoder::create(options);
    if (!probe) {
      if (probe.status().domain() != Code::Unsupported) {
        std::fprintf(stderr, "FAIL: %s\n", probe.status().message().c_str());
        return 1;
      }
      const bool promised = testing::hevc_decoder_promised();
      std::fprintf(stderr, "no device path for the decoder (%s); %s\n",
                   probe.status().message().c_str(),
                   promised ? "failing" : "skipping");
      return promised ? 1 : vkc::test::kSkipExitCode;
    }
  }
  vkc::Result<std::unique_ptr<sensor::PictureReadback>> readback =
      sensor::PictureReadback::create(gpu.device, gpu.allocator);
  CHECK(readback.ok());
  sensor::PictureReadback& rb = *readback.value();
  if (test_round_trip(gpu, rb) != 0) return 1;
  if (test_b_frames(gpu, rb) != 0) return 1;
  if (test_refused(gpu, rb) != 0) return 1;
  if (test_open_gop(gpu, rb) != 0) return 1;
  if (test_lost_picture(gpu, rb) != 0) return 1;
  if (test_lost_depth(gpu, rb) != 0) return 1;
  if (test_description(gpu, rb) != 0) return 1;
  if (test_open_refusals(gpu) != 0) return 1;
  std::printf("rgbd stream sensor colour tests passed\n");
  return 0;
}

#else  // VR_TEST_LINK_VIDEO

// Without sensor's decoders, a device to decode on is refused; the depth
// opens without one.
int gpu_main(Gpu& gpu) {
  const vkc::test::Scratch scratch("recon_link_rgbd_color_none_");
  const std::string path = session_file(scratch, "phone.vrst", phone_info(),
                                        [](stream::Session&) {});
  CHECK(refused(RgbdStreamSensor::open_file(path, on_device(gpu)),
                Code::Unsupported, "sensor_video"));
  CHECK(RgbdStreamSensor::open_file(path, {}).ok());
  // Under pictures_optional, read as depth alone, saying why.
  RgbdStreamSensor::Options optional = on_device(gpu);
  optional.pictures_optional = true;
  vkc::Result<std::unique_ptr<RgbdStreamSensor>> let_go =
      RgbdStreamSensor::open_file(path, optional);
  CHECK(let_go.ok() && !let_go.value()->info().color &&
        let_go.value()->pictures_unread().find("sensor_video") !=
            std::string::npos);
  std::printf("rgbd stream sensor colour tests passed (no decoders)\n");
  return 0;
}

#endif  // VR_TEST_LINK_VIDEO

}  // namespace

int main() {
  // Unbuffered, so a crash in a driver still shows which test it was in.
  std::setvbuf(stdout, nullptr, _IONBF, 0);
#if VR_TEST_LINK_VIDEO
  if (units_of(kPatches).size() != 8) {
    std::fprintf(stderr, "FAIL: cannot split %s into 8 access units\n",
                 kPatches);
    return 1;
  }
#endif
  return vkc::test::run_on_gpu(sensor::device_requirements(), gpu_main);
}
