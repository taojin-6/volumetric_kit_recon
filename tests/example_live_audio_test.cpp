// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// A live run's mic (examples/common/live_audio.hpp) with no device: a fake
// mic of synthetic tone chunks, 20 ms each on the host's clock, fed into a
// codec stream after each frame as a live run feeds it after each set
// (AudioFeed), the frames a sphere's at 25 a second, the mic holding audio
// 100 ms past the frame just sent. The file holds both tracks: the session
// live_session gives, every frame, and an Audio record a chunk from the first
// frame's pts on, each a key at its chunk's capture time with its sequence,
// unbroken, and its samples; the two chunks before the first frame dropped,
// and the chunks captured after the last frame there too, as the last drain
// takes them. The feed drains once a frame and once more at the end, then
// stops the mic, and reports its counters. A mic whose drain fails ends the
// audio there, warned once, and the frames go on. With no mic the feed does
// nothing, and the file is byte for byte the one the session before audio
// writes. With output.serve, a session with the mic's track is refused as
// the stream is made. open_audio opens nothing for an empty source.audio, and
// a build without the microphone driver refuses a named one. The device part
// skips with no device.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "fuse_sender.hpp"
#include "live_audio.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/config/config.hpp"
#include "volumetric_kit/core/testing/gpu_test.hpp"
#include "volumetric_kit/core/testing/scratch_dir.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/grid/codec/encoder.hpp"
#include "volumetric_kit/grid/codec_stream/codec_stream.hpp"
#include "volumetric_kit/grid/stream/session.hpp"
#include "volumetric_kit/grid/stream/stream_reader.hpp"
#include "volumetric_kit/grid/testing/codec_fixture.hpp"
#include "volumetric_kit/grid/testing/stream_files.hpp"
#include "volumetric_kit/grid/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/sensor/audio_sensor.hpp"
#include "volumetric_kit/sensor/rgbd_sensor.hpp"

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;
namespace codec_stream = volumetric_kit::grid::codec_stream;
namespace sensor = volumetric_kit::sensor;
namespace stream = volumetric_kit::grid::stream;
namespace vol = volumetric_kit::grid::volume;
namespace vrst = vgrid_test::vrst;

namespace {

constexpr std::uint32_t kRate = 48000;
constexpr std::uint32_t kChunkFrames = 960;     // 20 ms
constexpr std::uint64_t kChunkNs = 20'000'000;  // a chunk's span
constexpr std::uint64_t kFrameNs = 40'000'000;  // 25 frames a second
constexpr std::uint64_t kLagNs = 100'000'000;   // audio held past a frame
constexpr std::uint64_t kTailNs = 200'000'000;  // captured after the last
constexpr std::uint64_t kEarlyChunks = 2;       // before the first frame
constexpr std::size_t kFrames = 8;
constexpr std::uint64_t kStartNs = 1'760'000'000'123'456'789;  // host clock
constexpr const char* kProgram = "recon_example_live_audio";

// A frame's pts.
std::uint64_t frame_pts(std::size_t frame) {
  return kStartNs + frame * kFrameNs;
}

// The samples of the chunk numbered `seq`: a 440 Hz tone on every channel,
// its phase running on from the chunk before.
std::vector<std::int16_t> tone(std::uint64_t seq, std::uint16_t channels) {
  std::vector<std::int16_t> samples;
  for (std::uint64_t f = 0; f < kChunkFrames; ++f) {
    const double t = static_cast<double>(seq * kChunkFrames + f) /
                     static_cast<double>(kRate);
    const auto s = static_cast<std::int16_t>(
        std::lround(12000.0 * std::sin(2.0 * 3.14159265358979 * 440.0 * t)));
    for (std::uint16_t c = 0; c < channels; ++c) samples.push_back(s);
  }
  return samples;
}

// Bytes as a record holds samples: signed 16-bit, little-endian.
vrst::Bytes le_bytes(const std::vector<std::int16_t>& samples) {
  vrst::Bytes b;
  for (const std::int16_t s : samples) {
    vrst::put(b, static_cast<std::uint16_t>(s), 2);
  }
  return b;
}

// A mic the test captures for: each chunk 20 ms of tone, stamped on the
// host's clock from the first chunk's time on and numbered from 0, held
// until drained. Its drains fail from the `fail_from`-th on (none at 0).
class FakeMic final : public sensor::IAudioSensor {
 public:
  FakeMic(std::uint64_t first_ns, std::uint64_t fail_from)
      : next_ns_(first_ns), fail_from_(fail_from) {
    info_.id = "fake-mic";
    info_.model = "a fake mic";
    info_.clock = sensor::ClockDomain::Host;
    info_.sample_rate = kRate;
    info_.channels = 2;
    info_.chunk_frames = kChunkFrames;
  }

  const sensor::AudioInfo& info() const noexcept override { return info_; }
  vkc::Status start() override {
    ++starts;
    running = true;
    return {};
  }
  void stop() noexcept override {
    ++stops;
    running = false;
    held_.clear();
  }
  vkc::Status drain(std::vector<sensor::AudioChunk>* out) override {
    ++drains;
    if (fail_from_ != 0 && drains >= fail_from_) {
      return vkc::Status::io_error("fake-mic: nothing for 1 s");
    }
    for (sensor::AudioChunk& chunk : held_) {
      out->push_back(std::move(chunk));
      ++stats_.delivered;
    }
    held_.clear();
    return {};
  }
  sensor::SensorStats stats() const noexcept override { return stats_; }

  // Capture, while running, every chunk whose last sample is before
  // `until_ns`.
  void capture(std::uint64_t until_ns) {
    while (running && next_ns_ + kChunkNs <= until_ns) {
      sensor::AudioChunk chunk;
      chunk.timestamp_ns = next_ns_;
      chunk.sequence = captured;
      chunk.samples = tone(captured, info_.channels);
      held_.push_back(std::move(chunk));
      ++captured;
      ++stats_.received;
      next_ns_ += kChunkNs;
    }
  }

  int starts = 0;
  int stops = 0;
  std::uint64_t drains = 0;
  std::uint64_t captured = 0;
  bool running = false;

 private:
  sensor::AudioInfo info_;
  sensor::SensorStats stats_;
  std::vector<sensor::AudioChunk> held_;
  std::uint64_t next_ns_;
  std::uint64_t fail_from_;
};

// A run's scenario: 25 fps.
volumetric_kit::core::config::Config scenario() {
  volumetric_kit::core::config::Config opt;
  opt.source.fps = 25;
  return opt;
}

// The file at `path` coded from `session`: kFrames frames of `grid`, the
// feed of `mic` (or none) started first, sent after each frame once the mic
// captured up to kLagNs past it, and finished after kTailNs more, before the
// stream.
int run(vkc::test::GpuContext& gpu, vol::VoxelBlockGrid& grid,
        const stream::Session& session, FakeMic* mic, const std::string& path,
        std::string* report) {
  codec_stream::CodecOutputs outputs;
  outputs.stream = path;
  outputs.decode = false;
  vkc::Result<codec_stream::CodecStream> made =
      codec_stream::CodecStream::create(gpu.device, gpu.allocator, {}, session,
                                        outputs);
  CHECK(made.ok());
  codec_stream::CodecStream& coded = made.value();
  vr_example::AudioFeed feed(mic, &coded, vr_example::kAudioTrack, kProgram);
  CHECK(feed.start().ok());
  for (std::size_t frame = 0; frame < kFrames; ++frame) {
    CHECK(coded.code(grid, frame, frame_pts(frame)).ok());
    if (mic != nullptr) mic->capture(frame_pts(frame) + kLagNs);
    CHECK(feed.send().ok());
  }
  if (mic != nullptr) mic->capture(frame_pts(kFrames - 1) + kTailNs);
  CHECK(feed.finish().ok());
  CHECK(feed.send().ok());  // finished: nothing more
  *report = feed.report();
  CHECK(coded.finish().ok());
  return 0;
}

// The chunks a mic numbered `first` to `last` (inclusive) as the file's Audio
// records, in file order: each a key on the track at its chunk's capture
// time, its seq the chunk's, its payload the chunk's tone; each after a
// geometry record at or past its pts unless past the last frame's; and every
// frame there.
int check_file(const std::string& path, const stream::Session& session,
               std::uint64_t first_ns, std::uint64_t first,
               std::uint64_t last) {
  const std::unique_ptr<stream::StreamReader> reader = vrst::open(path);
  CHECK(reader != nullptr);
  CHECK(vrst::same(reader->session(), session));
  const vrst::ReadBack back = vrst::read_all(*reader);
  CHECK(back.ended && back.next_errors.empty() && back.read_errors.empty());
  std::size_t frames = 0;
  std::uint64_t seq = first;
  std::uint64_t last_frame_pts = 0;
  for (std::size_t i = 0; i < back.records.size(); ++i) {
    const stream::RecordInfo& info = back.records[i].info;
    if (info.track == 0) {
      CHECK(info.pts_ns == frame_pts(frames));
      last_frame_pts = info.pts_ns;
      ++frames;
      continue;
    }
    CHECK(info.track == vr_example::kAudioTrack);
    CHECK(info.key && info.set == stream::kNoSet);
    CHECK(info.seq == seq);
    CHECK(info.pts_ns == first_ns + seq * kChunkNs);
    CHECK(back.payloads[i] == le_bytes(tone(seq, 2)));
    // Near the sets': after the frame it plays with, never before the first.
    CHECK(frames > 0 && info.pts_ns >= frame_pts(0));
    CHECK(info.pts_ns <= last_frame_pts || frames == kFrames);
    ++seq;
  }
  CHECK(frames == kFrames);
  CHECK(seq == last + 1);
  return 0;
}

// A mic beside the frames: both tracks in the file, the audio unbroken from
// the first frame on, the last drain's chunks included; the mic drained once
// a frame and once at the end, then stopped; the stream's counts and the
// feed's report.
int recorded(vkc::test::GpuContext& gpu, vol::VoxelBlockGrid& grid,
             const vkc::test::Scratch& scratch) {
  const std::uint64_t first_ns = kStartNs - kEarlyChunks * kChunkNs;
  FakeMic mic(first_ns, 0);
  const stream::Session session =
      vr_example::live_session(kProgram, scenario(), true, &mic.info());
  CHECK(session.tracks.size() == 2);
  const std::string path = (scratch.path / "recorded.vrst").string();
  std::string report;
  CHECK(run(gpu, grid, session, &mic, path, &report) == 0);
  CHECK(mic.starts == 1 && mic.stops == 1 && !mic.running);
  CHECK(mic.drains == kFrames + 1);
  // Up to 200 ms past the last frame, from 40 ms before the first.
  const std::uint64_t captured =
      (frame_pts(kFrames - 1) + kTailNs - first_ns) / kChunkNs;
  CHECK(mic.captured == captured);
  CHECK(check_file(path, session, first_ns, kEarlyChunks, captured - 1) == 0);
  CHECK(report == "  audio fake-mic: " + std::to_string(captured) +
                      " received, " + std::to_string(captured) +
                      " delivered, 0 dropped, 0 failed\n");
  return 0;
}

// A mic whose fourth drain fails: the audio ends there, the feed warns and
// drains no more, finishing stops the mic, and every frame is coded; the file
// holds the chunks the first three drains took.
int lost(vkc::test::GpuContext& gpu, vol::VoxelBlockGrid& grid,
         const vkc::test::Scratch& scratch) {
  FakeMic mic(kStartNs, 4);
  const stream::Session session =
      vr_example::live_session(kProgram, scenario(), true, &mic.info());
  const std::string path = (scratch.path / "lost.vrst").string();
  std::string report;
  CHECK(run(gpu, grid, session, &mic, path, &report) == 0);
  CHECK(mic.drains == 4 && mic.stops == 1);
  // The third drain took what was captured up to frame 2's pts + 100 ms.
  const std::uint64_t taken = (frame_pts(2) + kLagNs - kStartNs) / kChunkNs;
  CHECK(check_file(path, session, kStartNs, 0, taken - 1) == 0);
  CHECK(report.size() > 14 &&
        report.compare(report.size() - 14, 14, "; ended early\n") == 0);
  return 0;
}

// No mic: the feed does nothing, and the file is the one the session before
// audio writes, byte for byte.
int no_mic(vkc::test::GpuContext& gpu, vol::VoxelBlockGrid& grid,
           const vkc::test::Scratch& scratch) {
  stream::Session before = vgrid_example::codec_session(kProgram, {25, 1});
  before.tracks[0].clock = stream::kHostClock;
  const stream::Session session =
      vr_example::live_session(kProgram, scenario(), true, nullptr);
  const std::string with_feed = (scratch.path / "no_mic.vrst").string();
  const std::string plain = (scratch.path / "before.vrst").string();
  std::string report = "unset";
  CHECK(run(gpu, grid, session, nullptr, with_feed, &report) == 0);
  CHECK(report.empty());
  codec_stream::CodecOutputs outputs;
  outputs.stream = plain;
  outputs.decode = false;
  vkc::Result<codec_stream::CodecStream> made =
      codec_stream::CodecStream::create(gpu.device, gpu.allocator, {}, before,
                                        outputs);
  CHECK(made.ok());
  for (std::size_t frame = 0; frame < kFrames; ++frame) {
    CHECK(made->code(grid, frame, frame_pts(frame)).ok());
  }
  CHECK(made->finish().ok());
  CHECK(vkc::test::read_bytes(with_feed) == vkc::test::read_bytes(plain));
  return 0;
}

// A mic with output.serve: the stream refuses the session as it is made,
// which a run does before its cameras open, until grid serves an Audio track.
int served(vkc::test::GpuContext& gpu) {
  FakeMic mic(kStartNs, 0);
  codec_stream::CodecOutputs outputs;
  outputs.decode = false;
  outputs.serve.emplace();
  outputs.serve->host = "127.0.0.1";
  outputs.serve->port = 0;
  const vkc::Result<codec_stream::CodecStream> made =
      codec_stream::CodecStream::create(
          gpu.device, gpu.allocator, {},
          vr_example::live_session(kProgram, scenario(), true, &mic.info()),
          outputs);
  CHECK(made.status().domain() == vkc::Status::Code::Unsupported);
  return 0;
}

int gpu_main(vkc::test::GpuContext& gpu) {
  vkc::test::Scratch scratch("recon_example_live_audio_");
  const codec_fixture::Sphere sphere{
      volumetric_kit::grid::Vec3f(0.01f, -0.02f, 0.03f), 0.09f};
  vkc::Result<vol::VoxelBlockGrid> grid =
      codec_fixture::sphere_grid(gpu, sphere);
  CHECK(grid.ok());
  CHECK(recorded(gpu, *grid, scratch) == 0);
  CHECK(lost(gpu, *grid, scratch) == 0);
  CHECK(no_mic(gpu, *grid, scratch) == 0);
  CHECK(served(gpu) == 0);
  std::puts("recon_example_live_audio: OK");
  return 0;
}

// open_audio: no mic for an empty source.audio; without the microphone
// driver, a named one refused as Unsupported. A build with the driver is not
// asked to open one: that would open a device.
int opened() {
  volumetric_kit::core::config::Config opt;
  vkc::Result<std::unique_ptr<sensor::IAudioSensor>> none =
      vr_example::open_audio(opt);
  CHECK(none.ok() && none.value() == nullptr);
#ifndef VR_EXAMPLE_MICROPHONE
  opt.source.audio = "default";
  const vkc::Result<std::unique_ptr<sensor::IAudioSensor>> refused =
      vr_example::open_audio(opt);
  CHECK(refused.status().domain() == vkc::Status::Code::Unsupported);
#endif
  return 0;
}

}  // namespace

int main() {
  CHECK(opened() == 0);
  return vkc::test::run_on_gpu(vr::device_requirements(), gpu_main);
}
