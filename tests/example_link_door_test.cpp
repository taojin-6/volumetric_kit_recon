// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Writers' sessions on one address (examples/common/link/
// rgbd_stream_sensor.hpp, RgbdStreamSensor::Door), each dialled as an iPad's
// writer dials (tests/link_dialler.hpp), depth alone: pattern depth carrying
// each frame's index. Host only, on 127.0.0.1.
// - open returns within 100 ms with no writer; an address it cannot bind is
//   an IoError, and a later open binds it once it is free; its refusals.
// - Two sessions at once, each polled on a thread of its own, each get only
//   their own frames and notes.
// - A reconnect goes to its session with no new arrival: two connections,
//   the records sent again skipped, and one recording that reads back.
// - A newer connection from the same peer takes the place of the one read
//   at once: one superseded, no failure.
// - One from another peer (a second address of this host; skipped without
//   one) only once the one read has brought nothing for kSupersedeSilence.
// - The cap refuses a new session while a held one's reconnect goes through,
//   and the refused one arrives once the first sensor is destroyed.
// - A held session's id with other bytes is refused, the held one untouched.
// - A refusal is logged once an id a spell: until that id arrives, or after
//   a quiet.
// - A session admit refuses for what it is leaves its id remembered as
//   refused; one refused for what the options ask does not.
// - A stray connection, silent, does not delay a real one, and goes at the
//   handshake limit, closed.
// - Nine strays let the oldest go, and three from one peer that peer's.
// - The failure cap ends only that session; its id cools down, refused for
//   that, and then arrives as a new session.
// - connected and ended, seen from another thread, flip as the link reads;
//   listen's handle too.
// - A say once the sensor is gone, and the door, is harmless; an arrival
//   never admitted ends with the door.
// - admit's refusals: an arrival not this door's, nor another door's, nor
//   one admitted. A recording it cannot make leaves the session unrecorded,
//   its error polled once. A file has no handle.
// - The screen refuses a new session it gives a reason, before the cap and
//   counted as screened, never a held one's reconnect; door.csv holds a row
//   for each refusal, the door's and admit's, and each connection
//   superseded, with its session and peer; it is made at its first row, so
//   a door that refuses nothing writes none, and never over a file already
//   there.

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "link/rgbd_stream_sensor.hpp"
#include "link_dialler.hpp"
#include "volumetric_kit/core/base/log.hpp"
#include "volumetric_kit/core/base/result.hpp"
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
namespace gnet = volumetric_kit::grid::networking;
using Code = vkc::Status::Code;
using Tone = gnet::ViewerNote::Tone;
using vr_example::link::DoorOptions;
using vr_example::link::RgbdStreamSensor;
using vr_example::link::SessionHandle;
using vr_test::Clock;
using vr_test::Dialler;
using vr_test::PatternFrames;
using vr_test::Record;
using Door = RgbdStreamSensor::Door;
using End = SessionHandle::End;
using Frames = std::vector<sensor::RgbdFrame>;
using std::chrono::milliseconds;

constexpr auto kWait = std::chrono::seconds(5);

// The warnings logged while it lives, from any thread; the default sink back
// once it goes, before what it holds does.
class Warnings {
 public:
  Warnings() {
    vkc::set_log_handler(
        [this](vkc::LogLevel level, std::string_view, std::string_view line) {
          if (level < vkc::LogLevel::Warning) return;
          std::lock_guard<std::mutex> lock(mutex_);
          lines_.emplace_back(line);
        });
  }
  ~Warnings() { vkc::set_log_handler({}); }
  Warnings(const Warnings&) = delete;
  Warnings& operator=(const Warnings&) = delete;

  // How many hold `text`.
  std::size_t count(const std::string& text) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<std::size_t>(
        std::count_if(lines_.begin(), lines_.end(), [&](const std::string& w) {
          return w.find(text) != std::string::npos;
        }));
  }

 private:
  mutable std::mutex mutex_;
  std::vector<std::string> lines_;
};

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
  const auto end = Clock::now() + kWait;
  while (frames.size() < count && !s.exhausted() && Clock::now() < end) {
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

// Polls a sensor on a thread of its own, as a lane's poll thread does, until
// stopped: its frames kept, their count read from any thread.
class Poller {
 public:
  explicit Poller(RgbdStreamSensor& s) : thread_([this, &s] { run(s); }) {}
  ~Poller() { (void)stop(); }
  Poller(const Poller&) = delete;
  Poller& operator=(const Poller&) = delete;

  // Stops and joins; the frames are the caller's from then on.
  Frames stop() {
    stop_ = true;
    if (thread_.joinable()) thread_.join();
    return std::move(frames_);
  }
  std::size_t count() const { return count_.load(); }

 private:
  void run(RgbdStreamSensor& s) {
    while (!stop_.load()) {
      vkc::Result<std::optional<sensor::RgbdFrame>> frame = s.poll();
      if (frame.ok() && *frame) {
        frames_.push_back(std::move(**frame));
        count_ = frames_.size();
      } else if (s.exhausted()) {
        std::this_thread::sleep_for(milliseconds(1));
      }
    }
  }

  std::atomic<bool> stop_{false};
  std::atomic<std::size_t> count_{0};
  Frames frames_;       // the thread's until it is joined
  std::thread thread_;  // last: started once the rest is
};

// The frames' sequences, in order.
std::vector<std::uint64_t> sequences(const Frames& frames) {
  std::vector<std::uint64_t> out;
  for (const sensor::RgbdFrame& f : frames) out.push_back(f.sequence);
  return out;
}

std::vector<std::uint64_t> from_to(std::uint64_t first, std::uint64_t last) {
  std::vector<std::uint64_t> out;
  for (std::uint64_t i = first; i <= last; ++i) out.push_back(i);
  return out;
}

// Each frame's depth carries its own number past `base`: its session's.
bool own_depth(const Frames& frames, std::uint32_t base = 0) {
  for (const sensor::RgbdFrame& f : frames) {
    if (testing::depth_index(f.depth, 256, 192) !=
        std::optional<std::uint32_t>(base +
                                     static_cast<std::uint32_t>(f.sequence))) {
      return false;
    }
  }
  return true;
}

std::unique_ptr<Door> open_door(std::uint16_t port, DoorOptions options = {}) {
  vkc::Result<std::unique_ptr<Door>> door =
      Door::open("127.0.0.1", port, options);
  if (!door.ok()) {
    std::fprintf(stderr, "%s\n", door.status().message().c_str());
    return nullptr;
  }
  return std::move(door).value();
}

// The next arrival, admitted with the sensor's defaults, or null, why on
// stderr; its handle to `handle` when given.
std::unique_ptr<RgbdStreamSensor> admit_next(
    Door& door, std::shared_ptr<SessionHandle>* handle = nullptr,
    std::string record_path = {}) {
  std::optional<Door::Arrival> arrival = door.take_arrival(kWait);
  if (!arrival) {
    std::fprintf(stderr, "no arrival within the wait\n");
    return nullptr;
  }
  if (handle != nullptr) *handle = arrival->handle;
  vkc::Result<std::unique_ptr<RgbdStreamSensor>> admitted =
      door.admit(std::move(*arrival), {}, std::move(record_path));
  if (!admitted.ok()) {
    std::fprintf(stderr, "admit: %s\n", admitted.status().message().c_str());
    return nullptr;
  }
  return std::move(admitted).value();
}

// Waits until the door holds `held` sessions.
bool holds(const Door& door, std::size_t held) {
  return vr_test::wait_for([&] { return door.stats().held == held; });
}

// Whether the writer's server reads `line` as its newest note within kWait.
bool heard(Dialler& d, const std::string& line) {
  return vr_test::wait_for([&] {
    const std::optional<gnet::ViewerNote> note = d.server().newest_note();
    return note && note->line == line;
  });
}

// Whether the door closed `fd`, a stray's, within `limit`: its end read.
bool closed(int fd, milliseconds limit = kWait) {
  std::vector<std::uint8_t> got;
  bool eof = false;
  vkc::test::read_upto(fd, 1, limit, &got, &eof);
  return eof && got.empty();
}

// A second address of this host from which a dialler reaches 127.0.0.1, if
// one serves: 127.0.0.2, or else the first other IPv4 address that does.
std::optional<std::string> second_peer() {
  std::vector<std::string> candidates{"127.0.0.2"};
  ifaddrs* all = nullptr;
  if (::getifaddrs(&all) == 0) {
    for (const ifaddrs* a = all; a != nullptr; a = a->ifa_next) {
      if (a->ifa_addr == nullptr || a->ifa_addr->sa_family != AF_INET) {
        continue;
      }
      const auto* in = reinterpret_cast<const sockaddr_in*>(a->ifa_addr);
      if ((ntohl(in->sin_addr.s_addr) >> 24) == 127) continue;
      char text[INET_ADDRSTRLEN];
      if (::inet_ntop(AF_INET, &in->sin_addr, text, sizeof text) != nullptr) {
        candidates.emplace_back(text);
      }
    }
    ::freeifaddrs(all);
  }
  const vkc::test::LoopbackListener probe = vkc::test::bind_loopback();
  if (!probe.fd.valid()) return std::nullopt;
  for (const std::string& from : candidates) {
    const vkc::Result<vkc::UniqueFd> fd =
        Dialler::connect_from(from, probe.port);
    if (fd.ok() &&
        vkc::test::accept_within(probe.fd.get(), milliseconds(500)).valid()) {
      return from;
    }
  }
  return std::nullopt;
}

bool refused(const vkc::Result<std::unique_ptr<Door>>& opened, Code code,
             const std::string& says) {
  if (opened.ok()) {
    std::fprintf(stderr, "opened\n");
    return false;
  }
  const std::string& message = opened.status().message();
  if (opened.status().domain() != code ||
      message.find(says) == std::string::npos ||
      message.rfind("RgbdStreamSensor::Door::open: ", 0) != 0) {
    std::fprintf(stderr, "refused as: %s\n", message.c_str());
    return false;
  }
  return true;
}

// open returns once bound, with no writer; its refusals, and an address that
// cannot be bound, which a later open binds once it is free.
int test_open() {
  const std::uint16_t port = vkc::test::closed_port();
  const auto start = Clock::now();
  vkc::Result<std::unique_ptr<Door>> door = Door::open("127.0.0.1", port);
  const auto took = Clock::now() - start;
  CHECK(door.ok());
  std::printf(
      "open took %lld us\n",
      static_cast<long long>(
          std::chrono::duration_cast<std::chrono::microseconds>(took).count()));
  CHECK(took < milliseconds(100));
  CHECK(!door.value()->take_arrival(milliseconds(0)));
  CHECK(!door.value()->take_arrival(milliseconds(20)));
  const Door::Stats none = door.value()->stats();
  CHECK(none.accepted == 0 && none.arrivals == 0 && none.held == 0);

  // Bound by another socket: an IoError, nothing kept, and once it is free
  // a later open binds it.
  vkc::test::LoopbackListener taken = vkc::test::bind_loopback();
  CHECK(taken.fd.valid());
  CHECK(refused(Door::open("127.0.0.1", taken.port), Code::IoError, ""));
  taken.fd.reset();
  CHECK(Door::open("127.0.0.1", taken.port).ok());
  CHECK(refused(Door::open("192.0.2.1", port), Code::IoError, ""));

  CHECK(
      refused(Door::open("0.0.0.0", port), Code::InvalidArgument, "wildcard"));
  CHECK(refused(Door::open("", port), Code::InvalidArgument, "empty"));
  CHECK(
      refused(Door::open("localhost", port), Code::InvalidArgument, "numeric"));
  CHECK(refused(Door::open("127.0.0.1", 0), Code::InvalidArgument, "port 0"));
  DoorOptions limits;
  limits.max_sessions = 0;
  CHECK(refused(Door::open("127.0.0.1", port, limits), Code::InvalidArgument,
                "at least 1"));
  limits = {};
  limits.max_pending_per_peer = 0;
  CHECK(refused(Door::open("127.0.0.1", port, limits), Code::InvalidArgument,
                "at least 1"));
  limits = {};
  limits.handshake = milliseconds(0);
  CHECK(refused(Door::open("127.0.0.1", port, limits), Code::InvalidArgument,
                "positive"));
  limits = {};
  limits.cool_down = milliseconds(-1);
  CHECK(refused(Door::open("127.0.0.1", port, limits), Code::InvalidArgument,
                "cool-down"));
  limits = {};
  limits.quiet = milliseconds(-1);
  CHECK(refused(Door::open("127.0.0.1", port, limits), Code::InvalidArgument,
                "quiet"));
  return 0;
}

// Two sessions at once, each polled on a thread of its own: each sensor
// hands out its own session's frames, and each writer hears its own note.
int test_two_sessions() {
  const PatternFrames a = vr_test::pattern_frames(vr_test::phone_info(), 6);
  const PatternFrames b = vr_test::pattern_frames(
      vr_test::phone_info(), 6,
      [](std::size_t i) { return static_cast<std::uint32_t>(100 + i); });
  const stream::Session session_a = vr_test::session_of(a.info, 0x01);
  const stream::Session session_b = vr_test::session_of(b.info, 0x02);
  const std::uint16_t port = vkc::test::closed_port();
  DoorOptions options;
  options.max_sessions = 2;
  std::unique_ptr<Door> door = open_door(port, options);
  CHECK(door);
  std::atomic<bool> said{false};
  std::atomic<bool> done{false};
  std::atomic<int> heard_own{0};
  bool a_own = false;
  bool b_own = false;
  const auto writer = [&](const stream::Session& session,
                          const PatternFrames& p, const std::string& line,
                          bool* own) {
    auto d = Dialler::dial(session, port, /*read_notes=*/true);
    if (!d.ok()) return;
    for (std::size_t i = 0; i < p.frames.size(); ++i) {
      if (!publish(**d, p, i).ok()) return;
    }
    if (!vr_test::wait_for([&] { return said.load(); })) return;
    if (!heard(**d, line)) return;
    ++heard_own;
    // The newest it holds is still its own once both are told.
    (void)vr_test::wait_for([&] { return done.load(); });
    const std::optional<gnet::ViewerNote> newest = (*d)->server().newest_note();
    *own = newest && newest->line == line;
  };
  std::thread wa([&] { writer(session_a, a, "to a", &a_own); });
  std::thread wb([&] { writer(session_b, b, "to b", &b_own); });
  std::shared_ptr<SessionHandle> ha;
  std::shared_ptr<SessionHandle> hb;
  std::unique_ptr<RgbdStreamSensor> sa = admit_next(*door, &ha);
  std::unique_ptr<RgbdStreamSensor> sb = admit_next(*door, &hb);
  Frames fa;
  Frames fb;
  bool flowed = false;
  bool told = false;
  if (sa && sb) {
    if (sa->session_id() == session_b.id) {
      std::swap(sa, sb);
      std::swap(ha, hb);
    }
    if (sa->start().ok() && sb->start().ok()) {
      Poller pa(*sa);
      Poller pb(*sb);
      flowed =
          vr_test::wait_for([&] { return pa.count() >= 6 && pb.count() >= 6; });
      ha->say({Tone::Good, "to a", {}});
      hb->say({Tone::Good, "to b", {}});
      said = true;
      told = vr_test::wait_for([&] { return heard_own.load() == 2; });
      fa = pa.stop();  // before the writers go, which would be failures
      fb = pb.stop();
    }
  }
  said = true;
  done = true;
  wa.join();
  wb.join();
  CHECK(sa && sb && flowed && told && a_own && b_own);
  CHECK(sa->session_id() == session_a.id && sb->session_id() == session_b.id);
  CHECK(sequences(fa) == from_to(0, 5) && own_depth(fa, 0));
  CHECK(sequences(fb) == from_to(0, 5) && own_depth(fb, 100));
  CHECK(testing::rgbd_breach(sa->info(), fa, sa->stats()).empty());
  CHECK(testing::rgbd_breach(sb->info(), fb, sb->stats()).empty());
  for (const RgbdStreamSensor* s : {sa.get(), sb.get()}) {
    const RgbdStreamSensor::LinkStats link = s->link_stats();
    CHECK(link.connections == 1 && link.failures == 0 && link.superseded == 0 &&
          link.connected);
  }
  CHECK(ha->connected() && hb->connected());
  const Door::Stats stats = door->stats();
  CHECK(stats.arrivals == 2 && stats.admitted == 2 && stats.held == 2 &&
        stats.refused == 0 && stats.reconnects == 0);
  return 0;
}

// Frames 0-3, a drop, then 2 and 3 again and 4-7: the reconnect goes to the
// session with no new arrival, the two sent again are skipped, and the one
// recording reads back as the frames.
int test_reconnect(const vkc::test::Scratch& scratch) {
  const PatternFrames p = vr_test::pattern_frames(vr_test::phone_info(), 8);
  const stream::Session session = vr_test::session_of(p.info);
  const std::string recording = (scratch.path / "phone.vrst").string();
  const std::uint16_t port = vkc::test::closed_port();
  std::unique_ptr<Door> door = open_door(port);
  CHECK(door);
  std::atomic<std::size_t> taken{0};
  std::atomic<SessionHandle*> watched{nullptr};
  std::atomic<bool> done{false};
  bool sent = false;
  std::thread writer([&] {
    {
      auto d = Dialler::dial(session, port);
      if (!d.ok()) return;
      for (std::size_t i = 0; i < 4; ++i) {
        if (!publish(**d, p, i).ok()) return;
      }
      if (!vr_test::wait_for([&] { return taken.load() >= 4; })) return;
    }  // the writer's connection drops; the link sees it go
    if (!vr_test::wait_for([&] { return !watched.load()->connected(); })) {
      return;
    }
    auto d = Dialler::dial(session, port);
    if (!d.ok()) return;
    for (const std::size_t i : {2, 3, 4, 5, 6, 7}) {
      if (!publish(**d, p, i).ok()) return;
    }
    sent = vr_test::wait_for([&] { return done.load(); });
  });
  std::shared_ptr<SessionHandle> handle;
  std::unique_ptr<RgbdStreamSensor> s = admit_next(*door, &handle, recording);
  Frames frames;
  bool no_arrival = false;
  if (s) {
    watched = handle.get();
    if (s->start().ok()) frames = poll_frames(*s, 8, &taken);
    no_arrival = !door->take_arrival(milliseconds(200));
  }
  done = true;
  writer.join();
  CHECK(s && sent && no_arrival);
  CHECK(sequences(frames) == from_to(0, 7) && own_depth(frames));
  CHECK(testing::rgbd_breach(s->info(), frames, s->stats()).empty());
  CHECK(s->skipped().empty());
  const RgbdStreamSensor::LinkStats link = s->link_stats();
  CHECK(link.connections == 2 && link.failures == 1 && link.superseded == 0 &&
        link.repeated == 2 && link.recorded == 8 && link.connected);
  const Door::Stats stats = door->stats();
  CHECK(stats.arrivals == 1 && stats.reconnects == 1 && stats.refused == 0);

  s.reset();  // finishes the recording
  vkc::Result<std::unique_ptr<stream::StreamReader>> reader =
      stream::StreamReader::open(recording);
  CHECK(reader.ok() && reader.value()->recovery().finished &&
        reader.value()->stats().size() == 1 &&
        reader.value()->stats()[0].records == 8);
  vkc::Result<std::unique_ptr<RgbdStreamSensor>> file =
      RgbdStreamSensor::open_file(recording, {});
  CHECK(file.ok() && file.value()->start().ok());
  const Frames read_back = poll_frames(*file.value(), ~std::size_t{0});
  CHECK(sequences(read_back) == from_to(0, 7) && own_depth(read_back));
  return 0;
}

// A second connection from the same peer while the first is read takes its
// place at once: one superseded, no failure, the first let go.
int test_supersede_same_peer() {
  const PatternFrames p = vr_test::pattern_frames(vr_test::phone_info(), 6);
  const stream::Session session = vr_test::session_of(p.info);
  const std::uint16_t port = vkc::test::closed_port();
  std::unique_ptr<Door> door = open_door(port);
  CHECK(door);
  std::atomic<std::size_t> taken{0};
  std::atomic<bool> done{false};
  bool sent = false;
  bool first_let_go = false;
  std::thread writer([&] {
    auto first = Dialler::dial(session, port);
    if (!first.ok()) return;
    for (std::size_t i = 0; i < 3; ++i) {
      if (!publish(**first, p, i).ok()) return;
    }
    if (!vr_test::wait_for([&] { return taken.load() >= 3; })) return;
    auto second = Dialler::dial(session, port);  // the first still open
    if (!second.ok()) return;
    for (std::size_t i = 3; i < 6; ++i) {
      if (!publish(**second, p, i).ok()) return;
    }
    first_let_go = (*first)->let_go();
    sent = vr_test::wait_for([&] { return done.load(); });
  });
  std::unique_ptr<RgbdStreamSensor> s = admit_next(*door);
  Frames frames;
  if (s && s->start().ok()) frames = poll_frames(*s, 6, &taken);
  done = true;
  writer.join();
  CHECK(s && sent && first_let_go);
  CHECK(sequences(frames) == from_to(0, 5) && own_depth(frames));
  const RgbdStreamSensor::LinkStats link = s->link_stats();
  CHECK(link.connections == 2 && link.superseded == 1 && link.failures == 0 &&
        link.repeated == 0 && link.connected);
  CHECK(door->stats().reconnects == 1 && door->stats().arrivals == 1);
  return 0;
}

// A connection from another peer waits while the one read keeps bringing
// frames, and takes its place once it has brought nothing for
// kSupersedeSilence.
int test_supersede_other_peer() {
  const std::optional<std::string> from = second_peer();
  if (!from) {
    std::printf(
        "another peer: skipped, no second address of this host "
        "reaches 127.0.0.1\n");
    return 0;
  }
  std::printf("another peer: from %s\n", from->c_str());
  const PatternFrames p = vr_test::pattern_frames(vr_test::phone_info(), 18);
  const stream::Session session = vr_test::session_of(p.info);
  const std::uint16_t port = vkc::test::closed_port();
  std::unique_ptr<Door> door = open_door(port);
  CHECK(door);
  std::atomic<std::size_t> taken{0};
  std::atomic<Clock::rep> stopped{0};  // when the first writer fell silent
  std::atomic<bool> done{false};
  bool sent = false;
  bool kept = false;
  bool first_let_go = false;
  std::thread writer([&] {
    auto first = Dialler::dial(session, port);
    if (!first.ok()) return;
    for (std::size_t i = 0; i < 3; ++i) {
      if (!publish(**first, p, i).ok()) return;
    }
    if (!vr_test::wait_for([&] { return taken.load() >= 3; })) return;
    // The other peer dials, its frames waiting with it; the first goes on at
    // 10 Hz, over a second in all, and keeps its place.
    auto second = Dialler::dial_from(session, port, *from);
    if (!second.ok()) {
      std::fprintf(stderr, "%s\n", second.status().message().c_str());
      return;
    }
    for (std::size_t i = 15; i < 18; ++i) {
      if (!publish(**second, p, i).ok()) return;
    }
    for (std::size_t i = 3; i < 15; ++i) {
      std::this_thread::sleep_for(milliseconds(100));
      if (!publish(**first, p, i).ok()) return;
    }
    stopped = Clock::now().time_since_epoch().count();
    kept = (*first)->server().viewers() == 1;
    first_let_go = (*first)->let_go(2 * kWait);
    sent = vr_test::wait_for([&] { return done.load(); });
  });
  std::unique_ptr<RgbdStreamSensor> s = admit_next(*door);
  Frames frames;
  std::uint64_t superseded_while_read = ~std::uint64_t{0};
  Clock::time_point took_over{};
  if (s && s->start().ok()) {
    const auto end = Clock::now() + 2 * kWait;
    while (frames.size() < 18 && Clock::now() < end) {
      vkc::Result<std::optional<sensor::RgbdFrame>> frame = s->poll();
      if (!frame.ok() || !*frame) continue;
      frames.push_back(std::move(**frame));
      taken = frames.size();
      if (frames.back().sequence == 14) {
        superseded_while_read = s->link_stats().superseded;
      }
      if (frames.back().sequence == 15) took_over = Clock::now();
    }
  }
  done = true;
  writer.join();
  CHECK(s && sent && kept && first_let_go);
  CHECK(sequences(frames) == from_to(0, 17) && own_depth(frames));
  CHECK(superseded_while_read == 0);
  const auto silence = took_over - Clock::time_point(Clock::duration(stopped));
  std::printf("the other peer took over %lld ms after the first fell silent\n",
              static_cast<long long>(
                  std::chrono::duration_cast<milliseconds>(silence).count()));
  CHECK(silence >= RgbdStreamSensor::kSupersedeSilence - milliseconds(50));
  CHECK(silence < 2 * RgbdStreamSensor::kSupersedeSilence + milliseconds(500));
  const RgbdStreamSensor::LinkStats link = s->link_stats();
  CHECK(link.connections == 2 && link.superseded == 1 && link.failures == 0);
  return 0;
}

// At the cap a new session is refused, closed at once, while the held one's
// reconnect goes through; once the held sensor is destroyed, the refused one
// arrives.
int test_cap() {
  const PatternFrames pa = vr_test::pattern_frames(vr_test::phone_info(), 2);
  const PatternFrames pb = vr_test::pattern_frames(vr_test::phone_info(), 1);
  const stream::Session session_a = vr_test::session_of(pa.info, 0x01);
  const stream::Session session_b = vr_test::session_of(pb.info, 0x02);
  const std::uint16_t port = vkc::test::closed_port();
  std::unique_ptr<Door> door = open_door(port);  // one session at most
  CHECK(door);
  auto a1 = Dialler::dial(session_a, port);
  CHECK(a1.ok() && publish(**a1, pa, 0).ok());
  std::unique_ptr<RgbdStreamSensor> sa = admit_next(*door);
  CHECK(sa && sa->start().ok());
  CHECK(sequences(poll_frames(*sa, 1)) == from_to(0, 0));

  auto b1 = Dialler::dial(session_b, port);
  CHECK(b1.ok() && (*b1)->let_go());  // refused: closed after its session
  CHECK(!door->take_arrival(milliseconds(100)));
  CHECK(door->stats().refused == 1 && door->stats().held == 1);

  auto a2 = Dialler::dial(session_a, port);  // at the cap: a reconnect
  CHECK(a2.ok() && publish(**a2, pa, 1).ok());
  CHECK(sequences(poll_frames(*sa, 1)) == from_to(1, 1));
  CHECK(door->stats().reconnects == 1 && sa->link_stats().superseded == 1);

  sa.reset();  // frees the slot
  CHECK(holds(*door, 0));
  auto b2 = Dialler::dial(session_b, port);
  CHECK(b2.ok() && publish(**b2, pb, 0).ok());
  std::unique_ptr<RgbdStreamSensor> sb = admit_next(*door);
  CHECK(sb && sb->session_id() == session_b.id && sb->start().ok());
  CHECK(sequences(poll_frames(*sb, 1)) == from_to(0, 0));
  const Door::Stats stats = door->stats();
  CHECK(stats.arrivals == 2 && stats.admitted == 2 && stats.held == 1 &&
        stats.refused == 1);
  return 0;
}

// A writer over the cap that redials is logged once; again once it has gone
// a quiet unrefused, and once it has arrived in between: a refusal is logged
// once an id a spell.
int test_said_once() {
  Warnings warnings;
  const auto refusals = [&] {
    return warnings.count("refused: 1 session(s) held");
  };
  const sensor::SensorInfo info = vr_test::phone_info();
  const stream::Session session_a = vr_test::session_of(info, 0x01);
  const stream::Session session_b = vr_test::session_of(info, 0x02);
  const std::uint16_t port = vkc::test::closed_port();
  DoorOptions options;
  options.quiet = milliseconds(500);
  std::unique_ptr<Door> door = open_door(port, options);
  CHECK(door);
  auto a = Dialler::dial(session_a, port);
  CHECK(a.ok());
  std::unique_ptr<RgbdStreamSensor> sa = admit_next(*door);
  CHECK(sa);
  for (int i = 0; i < 3; ++i) {
    auto b = Dialler::dial(session_b, port);
    CHECK(b.ok() && (*b)->let_go());
  }
  CHECK(door->stats().refused == 3 && refusals() == 1);

  // A quiet past, a new spell.
  std::this_thread::sleep_for(options.quiet);
  {
    auto b = Dialler::dial(session_b, port);
    CHECK(b.ok() && (*b)->let_go());
  }
  CHECK(door->stats().refused == 4 && refusals() == 2);

  sa.reset();
  CHECK(holds(*door, 0));
  auto b = Dialler::dial(session_b, port);
  CHECK(b.ok());
  std::unique_ptr<RgbdStreamSensor> sb = admit_next(*door);
  CHECK(sb);  // arrived: its spell is over
  sb.reset();
  CHECK(holds(*door, 0) && (*b)->let_go());
  auto a2 = Dialler::dial(session_a, port);
  CHECK(a2.ok());
  sa = admit_next(*door);
  CHECK(sa);
  auto b2 = Dialler::dial(session_b, port);
  CHECK(b2.ok() && (*b2)->let_go());
  CHECK(door->stats().refused == 5 && refusals() == 3);
  return 0;
}

// A session admit refuses frees its slot and leaves its id remembered: its
// next connection is refused at the door, and another session arrives.
int test_remembered() {
  const sensor::SensorInfo info = vr_test::phone_info();
  stream::Session no_depth = vr_test::session_of(info, 0x07);
  no_depth.tracks[0].kind = stream::TrackKind::Geometry;
  no_depth.tracks[0].params.clear();
  const std::uint16_t port = vkc::test::closed_port();
  std::unique_ptr<Door> door = open_door(port);
  CHECK(door);
  auto first = Dialler::dial(no_depth, port);
  CHECK(first.ok());
  std::optional<Door::Arrival> arrival = door->take_arrival(kWait);
  CHECK(arrival);
  const vkc::Result<std::unique_ptr<RgbdStreamSensor>> admitted =
      door->admit(std::move(*arrival), {});
  CHECK(!admitted.ok() && admitted.status().domain() == Code::InvalidArgument &&
        admitted.status().message().rfind("RgbdStreamSensor::Door::admit: ",
                                          0) == 0 &&
        admitted.status().message().find("no RgbdDepth track") !=
            std::string::npos);
  CHECK(!arrival->handle && !arrival->slot);  // consumed
  CHECK(holds(*door, 0) && (*first)->let_go());

  auto again = Dialler::dial(no_depth, port);
  CHECK(again.ok() && (*again)->let_go());
  CHECK(!door->take_arrival(milliseconds(100)));
  CHECK(door->stats().refused == 1);

  const PatternFrames p = vr_test::pattern_frames(info, 1);
  auto other = Dialler::dial(vr_test::session_of(info, 0x08), port);
  CHECK(other.ok() && publish(**other, p, 0).ok());
  std::unique_ptr<RgbdStreamSensor> s = admit_next(*door);
  CHECK(s && s->start().ok() && poll_frames(*s, 1).size() == 1);
  s.reset();
  CHECK(holds(*door, 0) && (*other)->let_go());

  // Refused for what the options ask of it, a sensor it lacks: not
  // remembered, and the id arrives again.
  const stream::Session asked = vr_test::session_of(info, 0x09);
  auto third = Dialler::dial(asked, port);
  CHECK(third.ok());
  std::optional<Door::Arrival> lacks = door->take_arrival(kWait);
  CHECK(lacks);
  RgbdStreamSensor::Options other_sensor;
  other_sensor.sensor = "lidar";
  const vkc::Result<std::unique_ptr<RgbdStreamSensor>> lacking =
      door->admit(std::move(*lacks), other_sensor);
  CHECK(!lacking.ok() && lacking.status().domain() == Code::InvalidArgument &&
        lacking.status().message().find("no depth track of sensor lidar") !=
            std::string::npos);
  CHECK(holds(*door, 0) && (*third)->let_go());
  auto fourth = Dialler::dial(asked, port);
  CHECK(fourth.ok() && publish(**fourth, p, 0).ok());
  s = admit_next(*door);
  CHECK(s && s->session_id() == asked.id);
  CHECK(door->stats().refused == 1);
  return 0;
}

// A connection that sends nothing waits beside a real one, which arrives at
// once, and goes at the handshake limit, closed.
int test_stray() {
  const std::uint16_t port = vkc::test::closed_port();
  DoorOptions options;
  options.handshake = milliseconds(1000);
  std::unique_ptr<Door> door = open_door(port, options);
  CHECK(door);
  const vkc::UniqueFd stray = vkc::test::connect_loopback(port);
  CHECK(stray.valid());
  CHECK(vr_test::wait_for([&] { return door->stats().accepted == 1; }));
  const auto start = Clock::now();
  auto d = Dialler::dial(vr_test::session_of(vr_test::phone_info()), port);
  CHECK(d.ok());
  const std::optional<Door::Arrival> arrival = door->take_arrival(kWait);
  const auto took = Clock::now() - start;
  const Door::Stats at = door->stats();
  std::printf("a session arrived %lld ms after its dial, a stray waiting\n",
              static_cast<long long>(
                  std::chrono::duration_cast<milliseconds>(took).count()));
  CHECK(arrival && at.strays == 0 && took < options.handshake);
  CHECK(vr_test::wait_for([&] { return door->stats().strays == 1; }));
  CHECK(closed(stray.get()));
  return 0;
}

// Nine silent connections from one peer let it hold all eight: the ninth
// lets the first go. At the default two a peer, the third lets that peer's
// first go.
int test_crowded() {
  for (const std::size_t per_peer : {std::size_t{16}, std::size_t{2}}) {
    const std::size_t count = per_peer == 2 ? 3 : 9;
    const std::uint16_t port = vkc::test::closed_port();
    DoorOptions options;
    options.max_pending_per_peer = per_peer;
    std::unique_ptr<Door> door = open_door(port, options);
    CHECK(door);
    std::vector<vkc::UniqueFd> strays;
    for (std::size_t i = 0; i < count; ++i) {
      strays.push_back(vkc::test::connect_loopback(port));
      CHECK(strays.back().valid());
      // Each accepted before the next dials: the order is the dials'.
      CHECK(vr_test::wait_for([&] { return door->stats().accepted == i + 1; }));
    }
    CHECK(door->stats().crowded == 1);
    CHECK(closed(strays[0].get()));
    CHECK(!closed(strays[1].get(), milliseconds(100)));
    CHECK(door->stats().strays == 0);
  }
  return 0;
}

// Three connections of one session that each bring a frame and drop end it,
// and it alone: the other session reads on. Its id cools down, refused for
// that, and then arrives as a new session.
int test_failure_cap() {
  Warnings warnings;
  const PatternFrames pa = vr_test::pattern_frames(vr_test::phone_info(), 3);
  const PatternFrames pb = vr_test::pattern_frames(
      vr_test::phone_info(), 2,
      [](std::size_t i) { return static_cast<std::uint32_t>(100 + i); });
  const stream::Session session_a = vr_test::session_of(pa.info, 0x01);
  const stream::Session session_b = vr_test::session_of(pb.info, 0x02);
  const std::uint16_t port = vkc::test::closed_port();
  DoorOptions options;
  options.max_sessions = 2;
  options.cool_down = milliseconds(500);
  std::unique_ptr<Door> door = open_door(port, options);
  CHECK(door);

  auto db = Dialler::dial(session_b, port);
  CHECK(db.ok() && publish(**db, pb, 0).ok());
  std::shared_ptr<SessionHandle> hb;
  std::unique_ptr<RgbdStreamSensor> sb = admit_next(*door, &hb);
  CHECK(sb && sb->start().ok());
  Poller pollb(*sb);

  std::atomic<std::size_t> taken{0};
  std::atomic<SessionHandle*> watched{nullptr};
  bool sent = false;
  std::thread writer([&] {
    for (std::size_t i = 0; i < 3; ++i) {
      // Each dial once the link has seen the last connection go, so each
      // drop is a failure, not a connection superseded.
      if (i > 0 &&
          !vr_test::wait_for([&] { return !watched.load()->connected(); })) {
        return;
      }
      auto d = Dialler::dial(session_a, port);
      if (!d.ok() || !publish(**d, pa, i).ok()) return;
      if (!vr_test::wait_for([&] { return taken.load() > i; })) return;
    }
    sent = true;
  });
  std::shared_ptr<SessionHandle> ha;
  std::unique_ptr<RgbdStreamSensor> sa = admit_next(*door, &ha);
  Frames frames;
  if (sa) {
    watched = ha.get();
    if (sa->start().ok()) frames = poll_frames(*sa, ~std::size_t{0}, &taken);
  }
  writer.join();
  CHECK(sa && sent);
  CHECK(sequences(frames) == from_to(0, 2) && sa->exhausted());
  CHECK(ha->ended() == End::FailureCap && !ha->connected());
  const RgbdStreamSensor::LinkStats link = sa->link_stats();
  CHECK(link.connections == 3 && link.failures == 3 && link.superseded == 0);

  // The other session reads on.
  CHECK(hb->ended() == End::None && hb->connected());
  CHECK(publish(**db, pb, 1).ok());
  CHECK(vr_test::wait_for([&] { return pollb.count() >= 2; }));

  // Its sensor destroyed, the ended session's id cools down: refused for
  // that, however soon it dials.
  sa.reset();
  const std::uint64_t refused = door->stats().refused;
  {
    auto d = Dialler::dial(session_a, port);
    CHECK(d.ok() && (*d)->let_go());
  }
  CHECK(!door->take_arrival(milliseconds(0)));
  CHECK(door->stats().refused == refused + 1 && door->stats().held == 1);
  CHECK(warnings.count("refused: its session ended by its failures or its "
                       "decoder within 500 ms") == 1);

  // Past it, the id arrives as a new session.
  std::this_thread::sleep_for(options.cool_down);
  auto d = Dialler::dial(session_a, port);
  CHECK(d.ok() && publish(**d, pa, 0).ok());
  std::shared_ptr<SessionHandle> again;
  std::unique_ptr<RgbdStreamSensor> sa2 = admit_next(*door, &again);
  CHECK(sa2 && again != ha && again->ended() == End::None);
  CHECK(sa2->start().ok() && sequences(poll_frames(*sa2, 1)) == from_to(0, 0));
  CHECK(sa2->link_stats().connections == 1);

  const Frames fb = pollb.stop();
  CHECK(sequences(fb) == from_to(0, 1) && own_depth(fb, 100));
  CHECK(door->stats().arrivals == 3);
  return 0;
}

// A thread that only watches the handle sees connected rise as the link
// takes the connection and fall as it goes, and the stream end once the
// sensor is destroyed.
int test_presence() {
  const PatternFrames p = vr_test::pattern_frames(vr_test::phone_info(), 2);
  const std::uint16_t port = vkc::test::closed_port();
  std::unique_ptr<Door> door = open_door(port);
  CHECK(door);
  auto d = Dialler::dial(vr_test::session_of(p.info), port);
  CHECK(d.ok() && publish(**d, p, 0).ok() && publish(**d, p, 1).ok());
  std::optional<Door::Arrival> arrival = door->take_arrival(kWait);
  CHECK(arrival);
  const std::shared_ptr<SessionHandle> handle = arrival->handle;
  CHECK(!handle->connected() && handle->ended() == End::None);
  vkc::Result<std::unique_ptr<RgbdStreamSensor>> admitted =
      door->admit(std::move(*arrival), {});
  CHECK(admitted.ok());
  std::unique_ptr<RgbdStreamSensor> s = std::move(admitted).value();

  std::atomic<bool> watching{false};
  std::atomic<bool> saw_connected{false};
  std::atomic<bool> stop{false};
  std::atomic<int> ended{static_cast<int>(End::None)};
  std::vector<bool> flips;
  std::thread watcher([&] {
    bool last = handle->connected();
    flips.push_back(last);
    watching = true;
    while (!stop.load()) {
      const bool now = handle->connected();
      if (now != last) {
        flips.push_back(now);
        last = now;
        if (now) saw_connected = true;
      }
      ended = static_cast<int>(handle->ended());
      std::this_thread::yield();
    }
  });
  bool gone = false;
  // The writer goes only once the watcher has seen the connection: the frames
  // were sent before the sensor started, so the connected spell can be shorter
  // than a busy host takes to schedule the watcher once.
  if (vr_test::wait_for([&] { return watching.load(); }) && s->start().ok() &&
      poll_frames(*s, 2).size() == 2 &&
      vr_test::wait_for([&] { return saw_connected.load(); })) {
    d.value().reset();  // the writer goes
    const auto end = Clock::now() + kWait;
    while (s->link_stats().connected && Clock::now() < end) (void)s->poll();
    gone = !s->link_stats().connected;
  }
  const bool dropped_is_no_end = handle->ended() == End::None;
  s.reset();
  const bool seen_end = vr_test::wait_for(
      [&] { return ended.load() == static_cast<int>(End::Closed); });
  stop = true;
  watcher.join();
  CHECK(gone && dropped_is_no_end && seen_end);
  CHECK((flips == std::vector<bool>{false, true, false}));
  CHECK(!handle->connected() && handle->ended() == End::Closed);
  return 0;
}

// A note said once the sensor is gone, from two threads, and once the door
// is, goes nowhere and harms nothing.
int test_say_after() {
  const PatternFrames p = vr_test::pattern_frames(vr_test::phone_info(), 1);
  const std::uint16_t port = vkc::test::closed_port();
  std::unique_ptr<Door> door = open_door(port);
  CHECK(door);
  auto d = Dialler::dial(vr_test::session_of(p.info), port, true);
  CHECK(d.ok() && publish(**d, p, 0).ok());
  std::shared_ptr<SessionHandle> handle;
  std::unique_ptr<RgbdStreamSensor> s = admit_next(*door, &handle);
  CHECK(s && s->start().ok() && poll_frames(*s, 1).size() == 1);
  handle->say({Tone::Good, "hello", {}});
  const auto end = Clock::now() + kWait;
  const auto said = [&] {
    const std::optional<gnet::ViewerNote> note = (*d)->server().newest_note();
    return note && note->line == "hello";
  };
  while (!said() && Clock::now() < end) (void)s->poll();
  CHECK(said());

  s.reset();
  std::thread other([&] {
    for (int i = 0; i < 200; ++i) {
      handle->say({Tone::Caution, "other " + std::to_string(i), {}});
    }
  });
  for (int i = 0; i < 200; ++i) {
    handle->say({Tone::Good, "after " + std::to_string(i), {}});
  }
  other.join();
  CHECK(handle->ended() == End::Closed && !handle->connected());
  CHECK((*d)->let_go() && holds(*door, 0));
  door.reset();
  handle->say({Tone::Good, "no door", {}});
  CHECK(handle->ended() == End::Closed);
  return 0;
}

// admit refuses an arrival not this door's, nor another door's, leaving it
// as it was, nor one admitted, which is spent. A recording it cannot make
// (something already there, a directory missing) leaves the session read
// unrecorded, the error logged and polled once. A file's sensor has no handle.
int test_admit_refusals(const vkc::test::Scratch& scratch) {
  Warnings warnings;
  const PatternFrames p = vr_test::pattern_frames(vr_test::phone_info(), 1);
  const stream::Session session = vr_test::session_of(p.info);
  const std::string there = (scratch.path / "there.vrst").string();
  {
    vkc::Result<std::unique_ptr<stream::StreamWriter>> writer =
        stream::StreamWriter::create(there, session);
    CHECK(writer.ok() && writer.value()->finish().ok());
  }
  const std::uintmax_t there_bytes = std::filesystem::file_size(there);
  const std::uint16_t port = vkc::test::closed_port();
  std::unique_ptr<Door> door = open_door(port);
  CHECK(door);
  const auto not_ours =
      [](const vkc::Result<std::unique_ptr<RgbdStreamSensor>>& admitted) {
        return !admitted.ok() &&
               admitted.status().domain() == Code::InvalidArgument &&
               admitted.status().message().find(
                   "not an arrival of this door") != std::string::npos;
      };
  Door::Arrival empty;
  CHECK(not_ours(door->admit(std::move(empty), {})));

  // Another door's: left as it was, and that door admits it.
  const std::uint16_t other_port = vkc::test::closed_port();
  std::unique_ptr<Door> other = open_door(other_port);
  CHECK(other);
  auto to_other = Dialler::dial(session, other_port);
  CHECK(to_other.ok());
  std::optional<Door::Arrival> foreign = other->take_arrival(kWait);
  CHECK(foreign);
  CHECK(not_ours(door->admit(std::move(*foreign), {})));
  CHECK(foreign->handle && foreign->slot);
  CHECK(other->admit(std::move(*foreign), {}).ok() && !foreign->slot);

  // Something already at the recording's path: read unrecorded, that file
  // untouched, and the first poll says why.
  auto first = Dialler::dial(session, port);
  CHECK(first.ok() && publish(**first, p, 0).ok());
  std::unique_ptr<RgbdStreamSensor> s = admit_next(*door, nullptr, there);
  CHECK(s && s->start().ok());
  const vkc::Result<std::optional<sensor::RgbdFrame>> taken = s->poll();
  CHECK(!taken.ok() && taken.status().domain() == Code::InvalidArgument &&
        taken.status().message().find(
            "the recording: something is already at") != std::string::npos);
  CHECK(sequences(poll_frames(*s, 1)) == from_to(0, 0));
  CHECK(s->link_stats().recorded == 0 && s->link_stats().connected);
  CHECK(warnings.count("; the session goes on unrecorded") == 1);
  CHECK(std::filesystem::file_size(there) == there_bytes);
  s.reset();
  CHECK(holds(*door, 0) && (*first)->let_go());

  // A directory missing, StreamWriter::create's error: the same. Once
  // admitted, the arrival is spent.
  const std::string nowhere =
      (scratch.path / "missing" / "phone.vrst").string();
  auto again = Dialler::dial(session, port);
  CHECK(again.ok() && publish(**again, p, 0).ok());
  std::optional<Door::Arrival> second = door->take_arrival(kWait);
  CHECK(second);
  vkc::Result<std::unique_ptr<RgbdStreamSensor>> admitted =
      door->admit(std::move(*second), {}, nowhere);
  CHECK(admitted.ok() && admitted.value()->handle() != nullptr);
  CHECK(not_ours(door->admit(std::move(*second), {})));
  RgbdStreamSensor& unrecorded = *admitted.value();
  CHECK(unrecorded.start().ok());
  const vkc::Result<std::optional<sensor::RgbdFrame>> missing =
      unrecorded.poll();
  CHECK(!missing.ok() &&
        missing.status().message().find("the recording") != std::string::npos);
  CHECK(sequences(poll_frames(unrecorded, 1)) == from_to(0, 0));
  CHECK(unrecorded.link_stats().recorded == 0 &&
        !std::filesystem::exists(nowhere));
  CHECK(warnings.count("; the session goes on unrecorded") == 2);
  CHECK(door->stats().refused == 0 && door->stats().admitted == 2);

  vkc::Result<std::unique_ptr<RgbdStreamSensor>> file =
      RgbdStreamSensor::open_file(there, {});
  CHECK(file.ok() && file.value()->handle() == nullptr);
  file.value()->say({Tone::Good, "to a file", {}});
  return 0;
}

// A connection under a held session's id with other bytes, another writer's,
// is refused and closed, a slot free or not; the held session keeps its
// connection and reads on, nothing superseded.
int test_other_bytes() {
  Warnings warnings;
  const PatternFrames p = vr_test::pattern_frames(vr_test::phone_info(), 2);
  const stream::Session session = vr_test::session_of(p.info);
  stream::Session other = session;
  other.writer = "another writer";
  const std::uint16_t port = vkc::test::closed_port();
  DoorOptions options;
  options.max_sessions = 2;
  std::unique_ptr<Door> door = open_door(port, options);
  CHECK(door);
  auto d = Dialler::dial(session, port);
  CHECK(d.ok() && publish(**d, p, 0).ok());
  std::unique_ptr<RgbdStreamSensor> s = admit_next(*door);
  CHECK(s && s->start().ok());
  CHECK(sequences(poll_frames(*s, 1)) == from_to(0, 0));

  auto hijack = Dialler::dial(other, port);
  CHECK(hijack.ok() && (*hijack)->let_go());
  CHECK(!door->take_arrival(milliseconds(0)));
  const Door::Stats stats = door->stats();
  CHECK(stats.refused == 1 && stats.reconnects == 0 && stats.arrivals == 1);
  CHECK(warnings.count("refused: another session under the id of one held") ==
        1);

  CHECK(publish(**d, p, 1).ok());
  CHECK(sequences(poll_frames(*s, 1)) == from_to(1, 1));
  const RgbdStreamSensor::LinkStats link = s->link_stats();
  CHECK(link.connections == 1 && link.superseded == 0 && link.failures == 0 &&
        link.connected);
  CHECK((*d)->server().viewers() == 1);
  return 0;
}

// listen's sensor has a handle of its own: connected while the writer's
// connection is read, not once it drops, which ends nothing, and ended once
// the sensor is destroyed.
int test_listen_handle() {
  const PatternFrames p = vr_test::pattern_frames(vr_test::phone_info(), 1);
  const std::uint16_t port = vkc::test::closed_port();
  std::unique_ptr<Dialler> writer;
  std::thread dialling([&] {
    auto d = Dialler::dial(vr_test::session_of(p.info), port);
    if (d.ok() && publish(**d, p, 0).ok()) writer = std::move(d).value();
  });
  vkc::Result<std::unique_ptr<RgbdStreamSensor>> live =
      RgbdStreamSensor::listen("127.0.0.1", port, {}, {}, kWait);
  dialling.join();
  CHECK(live.ok() && writer);
  RgbdStreamSensor& s = *live.value();
  const std::shared_ptr<SessionHandle> handle = s.handle();
  CHECK(handle && handle->connected() && handle->ended() == End::None);
  CHECK(s.start().ok() && poll_frames(s, 1).size() == 1);
  writer.reset();  // the writer goes
  const auto end = Clock::now() + kWait;
  while (s.link_stats().connected && Clock::now() < end) (void)s.poll();
  CHECK(!s.link_stats().connected && s.link_stats().failures == 1);
  CHECK(!handle->connected() && handle->ended() == End::None);
  live.value().reset();
  CHECK(handle->ended() == End::Closed && !handle->connected());
  return 0;
}

// An arrival the door goes before admitting ends with it: its handle reads
// Closed and its connection is let go. A session admitted reads on.
int test_door_gone() {
  const PatternFrames p = vr_test::pattern_frames(vr_test::phone_info(), 2);
  const stream::Session session_a = vr_test::session_of(p.info, 0x01);
  const stream::Session session_b = vr_test::session_of(p.info, 0x02);
  const std::uint16_t port = vkc::test::closed_port();
  DoorOptions options;
  options.max_sessions = 2;
  std::unique_ptr<Door> door = open_door(port, options);
  CHECK(door);
  auto da = Dialler::dial(session_a, port);
  CHECK(da.ok() && publish(**da, p, 0).ok());
  std::shared_ptr<SessionHandle> ha;
  std::unique_ptr<RgbdStreamSensor> sa = admit_next(*door, &ha);
  CHECK(sa && sa->start().ok() && poll_frames(*sa, 1).size() == 1);
  auto db = Dialler::dial(session_b, port);
  CHECK(db.ok());
  std::optional<Door::Arrival> arrival = door->take_arrival(kWait);
  CHECK(arrival && arrival->session.id == session_b.id);
  const std::shared_ptr<SessionHandle> hb = arrival->handle;
  CHECK(hb->ended() == End::None);

  door.reset();
  CHECK(hb->ended() == End::Closed && (*db)->let_go());
  arrival.reset();
  CHECK(hb->ended() == End::Closed);
  CHECK(ha->ended() == End::None && publish(**da, p, 1).ok());
  CHECK(sequences(poll_frames(*sa, 1)) == from_to(1, 1));
  return 0;
}

std::vector<std::string> lines_of(const std::filesystem::path& path) {
  std::ifstream in(path);
  std::vector<std::string> out;
  for (std::string line; std::getline(in, line);) out.push_back(line);
  return out;
}

// The screen, the cap and admit refuse; a reconnect supersedes; door.csv
// holds each, in order, written by the door's thread, made at its first row.
int test_screen_and_csv(const vkc::test::Scratch& scratch) {
  const sensor::SensorInfo info = vr_test::phone_info();
  const PatternFrames p = vr_test::pattern_frames(info, 2);
  const stream::Session held = vr_test::session_of(info, 0x01);
  const stream::Session screened = vr_test::session_of(info, 0x02);
  const stream::Session over = vr_test::session_of(info, 0x03);
  stream::Session no_depth = vr_test::session_of(info, 0x07);
  no_depth.tracks[0].kind = stream::TrackKind::Geometry;
  no_depth.tracks[0].params.clear();
  const std::filesystem::path csv = scratch.path / "door.csv";
  const std::uint16_t port = vkc::test::closed_port();
  DoorOptions options;
  options.csv = csv.string();
  std::size_t asked = 0;  // the door's thread's, read once it is gone
  options.screen = [&asked](const stream::Session& s) -> std::string {
    ++asked;
    return s.id[15] == 0x02 ? "not this one" : "";
  };
  std::unique_ptr<Door> door = open_door(port, options);
  CHECK(door);
  // admit's refusal, for what the session is.
  {
    auto d = Dialler::dial(no_depth, port);
    CHECK(d.ok());
    std::optional<Door::Arrival> arrival = door->take_arrival(kWait);
    CHECK(arrival);
    CHECK(!door->admit(std::move(*arrival), {}).ok());
    CHECK(holds(*door, 0) && (*d)->let_go());
  }
  auto a = Dialler::dial(held, port);
  CHECK(a.ok() && publish(**a, p, 0).ok());
  std::unique_ptr<RgbdStreamSensor> s = admit_next(*door);
  CHECK(s && s->start().ok() && poll_frames(*s, 1).size() == 1);
  // The screen's, though a slot is held; then the cap's.
  {
    auto b = Dialler::dial(screened, port);
    CHECK(b.ok() && (*b)->let_go());
    auto c = Dialler::dial(over, port);
    CHECK(c.ok() && (*c)->let_go());
  }
  Door::Stats stats = door->stats();
  CHECK(stats.refused == 2 && stats.screened == 1 && stats.held == 1);
  // A reconnect, at the cap, never screened: it supersedes the one read.
  auto a2 = Dialler::dial(held, port);
  CHECK(a2.ok() && publish(**a2, p, 1).ok());
  CHECK(sequences(poll_frames(*s, 1)) == from_to(1, 1));
  CHECK(s->link_stats().superseded == 1 && door->stats().reconnects == 1);
  door.reset();       // its last rows written
  CHECK(asked == 4);  // each new id: no_depth, held, screened, over
  const std::vector<std::string> rows = lines_of(csv);
  for (const std::string& r : rows) std::printf("door.csv: %s\n", r.c_str());
  CHECK(rows.size() == 5 && rows[0] == "time_ns,event,session,peer,reason");
  const auto row = [&](std::size_t k, const char* event,
                       const stream::Session& session, const char* why) {
    const std::string& r = rows[k];
    const std::string id = vr_test::hex_of(session.id);
    return r.find(std::string(",") + event + "," + id + ",127.0.0.1:") !=
               std::string::npos &&
           r.find(why) != std::string::npos;
  };
  CHECK(row(1, "refused", no_depth, ",no RgbdDepth track"));
  CHECK(row(2, "refused", screened, ",not this one"));
  CHECK(row(3, "refused", over, ",1 session(s) held, the most"));
  CHECK(row(4, "superseded", held, ",a newer connection from its address"));
  // A second door, refusing one, never writes over the first's file.
  const std::uint16_t again_port = vkc::test::closed_port();
  std::unique_ptr<Door> again = open_door(again_port, options);
  CHECK(again);
  {
    auto b = Dialler::dial(screened, again_port);
    CHECK(b.ok() && (*b)->let_go());
  }
  again.reset();
  CHECK(lines_of(csv) == rows);
  // A door that refuses nothing makes no door.csv.
  const std::filesystem::path unused = scratch.path / "unused" / "door.csv";
  std::filesystem::create_directories(unused.parent_path());
  options.csv = unused.string();
  std::unique_ptr<Door> quiet = open_door(vkc::test::closed_port(), options);
  CHECK(quiet);
  quiet.reset();
  CHECK(!std::filesystem::exists(unused));
  return 0;
}

}  // namespace

int main() {
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  const vkc::test::Scratch scratch("recon_link_door_");
  if (test_open() != 0) return 1;
  if (test_two_sessions() != 0) return 1;
  if (test_reconnect(scratch) != 0) return 1;
  if (test_supersede_same_peer() != 0) return 1;
  if (test_supersede_other_peer() != 0) return 1;
  if (test_cap() != 0) return 1;
  if (test_other_bytes() != 0) return 1;
  if (test_said_once() != 0) return 1;
  if (test_remembered() != 0) return 1;
  if (test_stray() != 0) return 1;
  if (test_crowded() != 0) return 1;
  if (test_failure_cap() != 0) return 1;
  if (test_presence() != 0) return 1;
  if (test_listen_handle() != 0) return 1;
  if (test_say_after() != 0) return 1;
  if (test_door_gone() != 0) return 1;
  if (test_admit_refusals(scratch) != 0) return 1;
  if (test_screen_and_csv(scratch) != 0) return 1;
  std::printf("rgbd stream sensor door tests passed\n");
  return 0;
}
