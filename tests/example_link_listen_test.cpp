// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// A session's sensor read live (examples/common/link/rgbd_stream_sensor.hpp,
// RgbdStreamSensor::listen) from a writer that dials 127.0.0.1 as an iPad's
// does (tests/link_dialler.hpp), on a thread of the test, with the depth
// alone: pattern depth carrying each frame's index. Host only.
// - Loopback: the frames equal open_file's of the session written as a file,
//   arrival times added, and the recording reads back as the same frames,
//   finished. rgbd_breach is empty.
// - A reconnect: the writer drops, another session's writer is refused and
//   not counted, and the writer's next connection sends two records again
//   (skipped) and then frames after a loss (a gap and a span). The recording
//   is one valid file of every record taken, which reads back as the live
//   frames with the same span.
// - Three failures within 10 s end the stream: exhausted, the listener
//   closed. The window itself (link/failure_cap.hpp): a failure 10 s or more
//   before the last no longer counts.
// - A start counts its spans from the first record found after it: a writer
//   already at frame 3, and again after a stop (which lets the connection
//   go, not as a failure) at frame 8, leaves none.
// - A poll with nothing to take, the writer silent or none connected, comes
//   back empty within its bound.
// - A record the recording refuses leaves it finished, with the records
//   before it, and poll returns the refusal once.
// - Refusals: a wildcard, empty or named host, port 0, a wait of 0, no
//   session within the wait, and a recording already there, before
//   listening. A session open_file would refuse is a connection refused:
//   the right writer after it is taken, and alone it is named when the wait
//   ends.
// - session_check's pass (link/session_check.hpp, SessionReport::ok).

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "link/failure_cap.hpp"
#include "link/rgbd_stream_sensor.hpp"
#include "link/session_check.hpp"
#include "link_dialler.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/net/dial.hpp"
#include "volumetric_kit/core/testing/net_loopback.hpp"
#include "volumetric_kit/core/testing/scratch_dir.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/grid/stream/stream_reader.hpp"
#include "volumetric_kit/grid/stream/stream_writer.hpp"
#include "volumetric_kit/sensor/testing/rgbd_contract.hpp"
#include "volumetric_kit/sensor/testing/rgbd_pattern.hpp"

namespace {

namespace vkc = volumetric_kit::core;
namespace sensor = volumetric_kit::sensor;
namespace stream = volumetric_kit::grid::stream;
namespace testing = volumetric_kit::sensor::testing;
using Code = vkc::Status::Code;
using vr_example::link::RgbdStreamSensor;
using vr_test::Dialler;
using vr_test::PatternFrames;
using vr_test::Record;
using Frames = std::vector<sensor::RgbdFrame>;

constexpr auto kWait = std::chrono::seconds(5);

// The session of `p` as a file at `path`, its frames' records in order.
vkc::Status write_file(const std::string& path, const PatternFrames& p) {
  VKC_ASSIGN(auto writer,
             stream::StreamWriter::create(path, vr_test::session_of(p.info)));
  for (const sensor::RgbdFrame& f : p.frames) {
    Record r;
    VKC_TRY(vr_test::depth_record(p.info, f, 0, f.sequence, &r));
    VKC_TRY(writer->append(r.info, r.payload));
  }
  return writer->finish();
}

vkc::Status publish(Dialler& d, const PatternFrames& p, std::size_t frame) {
  Record r;
  VKC_TRY(vr_test::depth_record(p.info, p.frames[frame], 0, frame, &r));
  vkc::Status published = d.publish(r);
  if (!published.ok()) {
    std::fprintf(stderr, "publish: %s\n", published.message().c_str());
  }
  return published;
}

// Frames polled from `s` until it has handed out `count` or is exhausted,
// for up to kWait.
Frames poll_frames(RgbdStreamSensor& s, std::size_t count,
                   std::atomic<std::size_t>* taken = nullptr) {
  Frames frames;
  const auto end = vr_test::Clock::now() + kWait;
  while (frames.size() < count && !s.exhausted() &&
         vr_test::Clock::now() < end) {
    vkc::Result<std::optional<sensor::RgbdFrame>> frame = s.poll();
    if (!frame.ok()) {
      std::fprintf(stderr, "poll: %s\n", frame.status().message().c_str());
      break;
    }
    if (*frame) {
      frames.push_back(std::move(**frame));
      if (taken != nullptr) taken->store(frames.size());
    }
  }
  return frames;
}

// One poll of `s` with nothing to take: empty, and back within ten times
// kPollWait, the bound a lane that polls beside other work relies on.
bool empty_poll(RgbdStreamSensor& s) {
  const auto start = vr_test::Clock::now();
  const vkc::Result<std::optional<sensor::RgbdFrame>> frame = s.poll();
  const auto took = std::chrono::duration_cast<std::chrono::milliseconds>(
      vr_test::Clock::now() - start);
  if (frame.ok() && !frame.value() && took < 10 * RgbdStreamSensor::kPollWait) {
    return true;
  }
  std::fprintf(stderr, "a poll with nothing to take: %s after %lld ms\n",
               !frame.ok()     ? frame.status().message().c_str()
               : frame.value() ? "a frame"
                               : "empty",
               static_cast<long long>(took.count()));
  return false;
}

// Every frame of the file at `path`.
Frames file_frames(const std::string& path, RgbdStreamSensor** keep,
                   std::unique_ptr<RgbdStreamSensor>* owner) {
  vkc::Result<std::unique_ptr<RgbdStreamSensor>> opened =
      RgbdStreamSensor::open_file(path, {});
  if (!opened.ok()) {
    std::fprintf(stderr, "%s\n", opened.status().message().c_str());
    return {};
  }
  *owner = std::move(opened).value();
  *keep = owner->get();
  if (!(*owner)->start().ok()) return {};
  return poll_frames(**owner, ~std::size_t{0});
}

// `a` and `b` are the same frame: depth and plane bytes, cameras, pose,
// units, range, noise, floor, time and number.
bool same_frame(const sensor::RgbdFrame& a, const sensor::RgbdFrame& b) {
  const std::size_t samples =
      std::size_t{a.depth_camera.size.width} * a.depth_camera.size.height;
  return a.depth != nullptr && b.depth != nullptr &&
         std::memcmp(&a.depth_camera, &b.depth_camera, sizeof a.depth_camera) ==
             0 &&
         std::memcmp(a.depth, b.depth, samples * 2) == 0 &&
         (a.depth_confidence == nullptr) == (b.depth_confidence == nullptr) &&
         (a.depth_confidence == nullptr ||
          std::memcmp(a.depth_confidence, b.depth_confidence, samples) == 0) &&
         a.color_to_world == b.color_to_world &&
         a.metres_per_unit == b.metres_per_unit && a.min_depth == b.min_depth &&
         a.max_depth == b.max_depth && a.depth_noise.a == b.depth_noise.a &&
         a.depth_noise.b == b.depth_noise.b &&
         a.depth_noise.c == b.depth_noise.c &&
         a.min_confidence == b.min_confidence &&
         a.timestamp_ns == b.timestamp_ns && a.sequence == b.sequence;
}

bool same_frames(const Frames& a, const Frames& b) {
  if (a.size() != b.size()) {
    std::fprintf(stderr, "%zu frames, not %zu\n", a.size(), b.size());
    return false;
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (!same_frame(a[i], b[i])) {
      std::fprintf(stderr, "frame %zu differs\n", i);
      return false;
    }
  }
  return true;
}

// The frames' sequences, in order.
std::vector<std::uint64_t> sequences(const Frames& frames) {
  std::vector<std::uint64_t> out;
  for (const sensor::RgbdFrame& f : frames) out.push_back(f.sequence);
  return out;
}

// Each frame's depth carries its own number.
bool own_depth(const Frames& frames) {
  for (const sensor::RgbdFrame& f : frames) {
    if (testing::depth_index(f.depth, 256, 192) !=
        std::optional<std::uint32_t>(static_cast<std::uint32_t>(f.sequence))) {
      return false;
    }
  }
  return true;
}

// The recording at `path` is finished and holds `records` records.
bool finished_with(const std::string& path, std::uint64_t records) {
  vkc::Result<std::unique_ptr<stream::StreamReader>> reader =
      stream::StreamReader::open(path);
  if (!reader.ok()) {
    std::fprintf(stderr, "%s\n", reader.status().message().c_str());
    return false;
  }
  const auto& r = *reader.value();
  return r.recovery().finished && r.stats().size() == 1 &&
         r.stats()[0].records == records;
}

vkc::Result<std::unique_ptr<RgbdStreamSensor>> listen(
    std::uint16_t port, std::string record_path = {},
    std::chrono::milliseconds wait = kWait) {
  return RgbdStreamSensor::listen("127.0.0.1", port, {}, std::move(record_path),
                                  wait);
}

// 8 frames over loopback equal the file's, and the recording is the file's
// frames again, finished.
int test_loopback(const vkc::test::Scratch& scratch) {
  const PatternFrames p = vr_test::pattern_frames(vr_test::phone_info(), 8);
  const std::string source = (scratch.path / "source.vrst").string();
  CHECK(write_file(source, p).ok());
  const std::string recording = (scratch.path / "loopback.vrst").string();
  const std::uint16_t port = vkc::test::closed_port();
  std::atomic<bool> done{false};
  bool sent = false;
  std::thread writer([&] {
    auto d = Dialler::dial(vr_test::session_of(p.info), port);
    if (!d.ok()) return;
    bool all = true;
    for (std::size_t i = 0; i < p.frames.size(); ++i) {
      all = all && publish(**d, p, i).ok();
    }
    sent = all && vr_test::wait_for([&] { return done.load(); });
  });
  vkc::Result<std::unique_ptr<RgbdStreamSensor>> live = listen(port, recording);
  if (!live.ok()) std::fprintf(stderr, "%s\n", live.status().message().c_str());
  CHECK(live.ok());
  RgbdStreamSensor& s = *live.value();
  CHECK(s.session_id() == vr_test::session_of(p.info).id);  // the dialler's
  CHECK(s.start().ok());
  const Frames frames = poll_frames(s, 8);
  const bool quick = empty_poll(s);  // the writer connected and silent
  done = true;
  writer.join();
  CHECK(sent && quick);
  CHECK(frames.size() == 8);
  for (const sensor::RgbdFrame& f : frames) CHECK(f.arrival_ns != 0);
  CHECK(own_depth(frames));
  CHECK(testing::rgbd_breach(s.info(), frames, s.stats()).empty());
  const sensor::SensorStats stats = s.stats();
  CHECK(stats.received == 8 && stats.delivered == 8 && stats.failed == 0);
  CHECK(s.skipped().empty());
  const RgbdStreamSensor::LinkStats link = s.link_stats();
  CHECK(link.connections == 1 && link.refused == 0 && link.failures == 0 &&
        link.repeated == 0 && link.recorded == 8 && link.connected);
  CHECK(!s.exhausted());

  std::unique_ptr<RgbdStreamSensor> file_owner;
  RgbdStreamSensor* file = nullptr;
  const Frames from_file = file_frames(source, &file, &file_owner);
  CHECK(same_frames(frames, from_file));
  CHECK(from_file[0].arrival_ns == 0);
  live.value().reset();  // finishes the recording
  CHECK(finished_with(recording, 8));
  std::unique_ptr<RgbdStreamSensor> recorded_owner;
  RgbdStreamSensor* recorded = nullptr;
  CHECK(same_frames(file_frames(recording, &recorded, &recorded_owner),
                    from_file));
  return 0;
}

// Frames 0-3, a drop, another session refused, then frames 2 and 3 again
// and 6-9: one failure, one refusal, two repeats, a span of 4 and 5, and one
// valid recording of the 8 records taken.
int test_reconnect(const vkc::test::Scratch& scratch) {
  const PatternFrames p = vr_test::pattern_frames(vr_test::phone_info(), 10);
  const std::string recording = (scratch.path / "reconnect.vrst").string();
  const std::uint16_t port = vkc::test::closed_port();
  std::atomic<std::size_t> taken{0};
  std::atomic<bool> done{false};
  bool sent = false;
  bool foreign_let_go = false;
  std::thread writer([&] {
    {
      auto d = Dialler::dial(vr_test::session_of(p.info), port);
      if (!d.ok()) return;
      for (std::size_t i = 0; i < 4; ++i) {
        if (!publish(**d, p, i).ok()) return;
      }
      if (!vr_test::wait_for([&] { return taken.load() >= 4; })) return;
    }  // the writer's connection drops
    {
      // Another session of the same sensor dials: refused.
      auto d = Dialler::dial(vr_test::session_of(p.info, 0x6c), port);
      if (!d.ok()) return;
      for (std::size_t i = 4; i < 6; ++i) {
        if (!publish(**d, p, i).ok()) return;
      }
      foreign_let_go = (*d)->let_go();
    }
    auto d = Dialler::dial(vr_test::session_of(p.info), port);
    if (!d.ok()) return;
    for (const std::size_t i : {2, 3, 6, 7, 8, 9}) {
      if (!publish(**d, p, i).ok()) return;
    }
    sent = vr_test::wait_for([&] { return done.load(); });
  });
  vkc::Result<std::unique_ptr<RgbdStreamSensor>> live = listen(port, recording);
  CHECK(live.ok());
  RgbdStreamSensor& s = *live.value();
  CHECK(s.start().ok());
  const Frames frames = poll_frames(s, 8, &taken);
  done = true;
  writer.join();
  CHECK(sent && foreign_let_go);
  CHECK((sequences(frames) ==
         std::vector<std::uint64_t>{0, 1, 2, 3, 6, 7, 8, 9}));
  CHECK(own_depth(frames));
  CHECK(testing::rgbd_breach(s.info(), frames, s.stats()).empty());
  const sensor::SensorStats stats = s.stats();
  CHECK(stats.received == 8 && stats.delivered == 8 && stats.failed == 0);
  CHECK(s.skipped().size() == 1);
  const RgbdStreamSensor::Skipped gap = s.skipped()[0];
  CHECK(gap.first == 4 && gap.count == 2 &&
        gap.after_ns == p.frames[3].timestamp_ns &&
        gap.before_ns == p.frames[6].timestamp_ns);
  const RgbdStreamSensor::LinkStats link = s.link_stats();
  CHECK(link.connections == 2 && link.refused == 1 && link.failures == 1 &&
        link.repeated == 2 && link.recorded == 8 && link.connected);

  live.value().reset();
  CHECK(finished_with(recording, 8));
  std::unique_ptr<RgbdStreamSensor> owner;
  RgbdStreamSensor* file = nullptr;
  CHECK(same_frames(file_frames(recording, &file, &owner), frames));
  CHECK(file->skipped().size() == 1 && file->skipped()[0].first == 4 &&
        file->skipped()[0].count == 2);
  return 0;
}

// Three connections that each send a frame and drop end the stream: the
// frames come out, the sensor is exhausted, and nothing listens any more.
int test_failure_cap() {
  const PatternFrames p = vr_test::pattern_frames(vr_test::phone_info(), 3);
  const std::uint16_t port = vkc::test::closed_port();
  std::atomic<std::size_t> taken{0};
  bool sent = false;
  std::thread writer([&] {
    for (std::size_t i = 0; i < 3; ++i) {
      auto d = Dialler::dial(vr_test::session_of(p.info), port);
      if (!d.ok() || !publish(**d, p, i).ok()) return;
      if (!vr_test::wait_for([&] { return taken.load() > i; })) return;
    }
    sent = true;
  });
  vkc::Result<std::unique_ptr<RgbdStreamSensor>> live = listen(port);
  CHECK(live.ok());
  RgbdStreamSensor& s = *live.value();
  CHECK(s.start().ok());
  const Frames frames = poll_frames(s, ~std::size_t{0}, &taken);
  writer.join();
  CHECK(sent);
  CHECK((sequences(frames) == std::vector<std::uint64_t>{0, 1, 2}));
  CHECK(s.exhausted());
  CHECK(testing::rgbd_breach(s.info(), frames, s.stats()).empty());
  const RgbdStreamSensor::LinkStats link = s.link_stats();
  CHECK(link.connections == 3 && link.failures == 3 && !link.connected);
  // The listener is closed: a dial is refused at once.
  CHECK(!volumetric_kit::core::net::dial("127.0.0.1", port,
                                         std::chrono::milliseconds(500))
             .ok());
  return 0;
}

// The cap's window, at the times given: three failures end a stream only
// within 10 s, so drops spread over a take never add up.
int test_failure_window() {
  using std::chrono::milliseconds;
  using std::chrono::seconds;
  const vr_example::link::FailureCap::Clock::time_point t0{};
  vr_example::link::FailureCap cap;  // kLinkFailures within kLinkFailureWindow
  CHECK(!cap.fail(t0));
  CHECK(!cap.fail(t0 + seconds(1)));
  CHECK(!cap.fail(t0 + seconds(10)));          // the first is 10 s before: out
  CHECK(cap.fail(t0 + milliseconds(10'500)));  // 1, 10 and 10.5 s: within
  return 0;
}

// A record the recording refuses: frame 1, a drop, then frame 2's record at
// frame 0's time, a key below the last key's. The file is finished with
// frame 1's record as it stood, and poll returns the refusal once.
int test_recording_refused(const vkc::test::Scratch& scratch) {
  const PatternFrames p = vr_test::pattern_frames(vr_test::phone_info(), 3);
  const std::string recording = (scratch.path / "refused.vrst").string();
  const std::uint16_t port = vkc::test::closed_port();
  std::atomic<std::size_t> taken{0};
  std::atomic<bool> done{false};
  bool sent = false;
  std::thread writer([&] {
    {
      auto d = Dialler::dial(vr_test::session_of(p.info), port);
      if (!d.ok() || !publish(**d, p, 1).ok()) return;
      if (!vr_test::wait_for([&] { return taken.load() >= 1; })) return;
    }
    auto d = Dialler::dial(vr_test::session_of(p.info), port);
    if (!d.ok()) return;
    sensor::RgbdFrame early = p.frames[2];
    early.timestamp_ns = p.frames[0].timestamp_ns;
    Record r;
    if (!vr_test::depth_record(p.info, early, 0, 2, &r).ok() ||
        !(*d)->publish(r).ok()) {
      return;
    }
    sent = vr_test::wait_for([&] { return done.load(); });
  });
  vkc::Result<std::unique_ptr<RgbdStreamSensor>> live = listen(port, recording);
  bool refusal = false;
  std::uint64_t recorded = 0;
  if (live.ok() && live.value()->start().ok() &&
      poll_frames(*live.value(), 1, &taken).size() == 1) {
    const auto end = vr_test::Clock::now() + kWait;
    while (!refusal && vr_test::Clock::now() < end) {
      const vkc::Result<std::optional<sensor::RgbdFrame>> polled =
          live.value()->poll();
      refusal =
          !polled.ok() && polled.status().domain() == Code::InvalidArgument &&
          polled.status().message().find("recording") != std::string::npos;
    }
    recorded = live.value()->link_stats().recorded;
  }
  done = true;
  writer.join();
  CHECK(live.ok() && sent && refusal && recorded == 1);
  CHECK(finished_with(recording, 1));  // finished with the sensor still open
  return 0;
}

// A writer already at frame 3 when the reader listens, and at frame 8 when
// it starts again: each start counts its spans from the first record found
// after it, so none is reported, and the stop lets the connection go.
int test_start_mid_session() {
  const PatternFrames p = vr_test::pattern_frames(vr_test::phone_info(), 10);
  const std::uint16_t port = vkc::test::closed_port();
  std::atomic<std::size_t> taken{0};
  std::atomic<bool> restarted{false};
  std::atomic<bool> done{false};
  bool sent = false;
  std::thread writer([&] {
    {
      auto d = Dialler::dial(vr_test::session_of(p.info), port);
      if (!d.ok()) return;
      for (std::size_t i = 3; i < 6; ++i) {
        if (!publish(**d, p, i).ok()) return;
      }
      if (!(*d)->let_go()) return;  // the reader's stop
    }
    if (!vr_test::wait_for([&] { return restarted.load(); })) return;
    auto d = Dialler::dial(vr_test::session_of(p.info), port);
    if (!d.ok()) return;
    for (std::size_t i = 8; i < 10; ++i) {
      if (!publish(**d, p, i).ok()) return;
    }
    sent = vr_test::wait_for([&] { return done.load(); });
  });
  vkc::Result<std::unique_ptr<RgbdStreamSensor>> live = listen(port);
  CHECK(live.ok());
  RgbdStreamSensor& s = *live.value();
  CHECK(s.start().ok());
  const Frames first = poll_frames(s, 3, &taken);
  CHECK((sequences(first) == std::vector<std::uint64_t>{3, 4, 5}));
  CHECK(s.skipped().empty());
  s.stop();
  CHECK(!s.link_stats().connected && s.link_stats().failures == 0);
  CHECK(s.start().ok());
  const bool quick = empty_poll(s);  // no writer connected
  restarted = true;
  const Frames second = poll_frames(s, 2);
  done = true;
  writer.join();
  CHECK(sent && quick);
  CHECK((sequences(second) == std::vector<std::uint64_t>{8, 9}));
  CHECK(s.skipped().empty());
  CHECK(testing::rgbd_breach(s.info(), second, s.stats()).empty());
  CHECK(s.link_stats().connections == 2 && s.link_stats().failures == 0);
  return 0;
}

bool refused(const vkc::Result<std::unique_ptr<RgbdStreamSensor>>& opened,
             Code code, const std::string& says) {
  if (opened.ok()) return false;
  const std::string& message = opened.status().message();
  if (opened.status().domain() != code ||
      message.find(says) == std::string::npos ||
      message.rfind("RgbdStreamSensor::listen: ", 0) != 0) {
    std::fprintf(stderr, "refused as: %s\n", message.c_str());
    return false;
  }
  return true;
}

// The listen's refusals: before listening, and of the sessions that come.
int test_refusals(const vkc::test::Scratch& scratch) {
  const std::uint16_t port = vkc::test::closed_port();
  CHECK(refused(RgbdStreamSensor::listen("0.0.0.0", port, {}),
                Code::InvalidArgument, "wildcard"));
  CHECK(refused(RgbdStreamSensor::listen("", port, {}), Code::InvalidArgument,
                "empty"));
  CHECK(refused(RgbdStreamSensor::listen("localhost", port, {}),
                Code::InvalidArgument, "numeric"));
  CHECK(refused(RgbdStreamSensor::listen("127.0.0.1", 0, {}),
                Code::InvalidArgument, "port 0"));
  CHECK(refused(listen(port, {}, std::chrono::milliseconds(0)),
                Code::InvalidArgument, "positive"));
  CHECK(refused(listen(port, {}, std::chrono::milliseconds(30)), Code::IoError,
                "no session within the wait"));

  const PatternFrames p = vr_test::pattern_frames(vr_test::phone_info(), 1);
  // Something at the recording path, before any writer dials.
  const std::string there = (scratch.path / "there.vrst").string();
  CHECK(write_file(there, p).ok());
  CHECK(refused(listen(port, there), Code::InvalidArgument, "the recording"));

  // A session open_file would refuse dials first: that connection is
  // refused, and the right writer, which dials after it, is taken.
  stream::Session no_depth = vr_test::session_of(p.info);
  no_depth.tracks[0].kind = stream::TrackKind::Geometry;
  no_depth.tracks[0].params.clear();
  std::atomic<bool> done{false};
  bool let_go = false;
  bool sent = false;
  std::thread writers([&] {
    {
      auto d = Dialler::dial(no_depth, port);
      if (!d.ok() || !(let_go = (*d)->let_go())) return;
    }
    auto d = Dialler::dial(vr_test::session_of(p.info), port);
    if (!d.ok() || !publish(**d, p, 0).ok()) return;
    sent = vr_test::wait_for([&] { return done.load(); });
  });
  vkc::Result<std::unique_ptr<RgbdStreamSensor>> live = listen(port);
  if (!live.ok()) std::fprintf(stderr, "%s\n", live.status().message().c_str());
  bool taken = false;
  if (live.ok()) {
    RgbdStreamSensor& s = *live.value();
    const RgbdStreamSensor::LinkStats link = s.link_stats();
    taken = link.refused == 1 && link.connections == 1 && s.start().ok() &&
            sequences(poll_frames(s, 1)) == std::vector<std::uint64_t>{0};
  }
  done = true;
  writers.join();
  CHECK(live.ok() && taken && let_go && sent);
  live.value().reset();

  // Alone, it is refused, and the wait ends naming why.
  bool alone_let_go = false;
  std::thread alone([&] {
    auto d = Dialler::dial(no_depth, port);
    if (d.ok()) alone_let_go = (*d)->let_go();
  });
  const bool named = refused(listen(port, {}, std::chrono::milliseconds(300)),
                             Code::IoError, "no RgbdDepth track");
  alone.join();
  CHECK(named && alone_let_go);
  return 0;
}

// session_check's pass: a frame came, the contract held, and every frame
// whose depth names an index showed its own picture.
int test_report_ok() {
  vr_example::link::SessionReport r;
  CHECK(!r.ok());  // no frame
  r.frames = 2;
  r.own_picture = 1;
  r.no_depth_index = 1;  // a real depth, which nothing is checked against
  CHECK(r.ok());
  r.no_picture_index = 1;  // a picture blended, damaged or not the pattern's
  CHECK(!r.ok());
  r.no_picture_index = 0;
  r.wrong = 1;
  CHECK(!r.ok());
  r.wrong = 0;
  r.breach = "a breach";
  CHECK(!r.ok());
  return 0;
}

}  // namespace

int main() {
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  const vkc::test::Scratch scratch("recon_link_listen_");
  if (test_loopback(scratch) != 0) return 1;
  if (test_reconnect(scratch) != 0) return 1;
  if (test_failure_cap() != 0) return 1;
  if (test_failure_window() != 0) return 1;
  if (test_start_mid_session() != 0) return 1;
  if (test_recording_refused(scratch) != 0) return 1;
  if (test_refusals(scratch) != 0) return 1;
  if (test_report_ok() != 0) return 1;
  std::printf("rgbd stream sensor listen tests passed\n");
  return 0;
}
