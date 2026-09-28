// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// How OrbbecRig turns per-camera frames into one set per sync trigger, and the
// order it starts a rig in -- with synthetic streams, no camera. The streams
// are shaped like the rig's: 30 fps, secondaries a fraction of a millisecond
// to ~2 ms off the primary, frames arriving in any order across cameras.

#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include "trigger_grouping.hpp"

namespace vr = volumetric_kit::recon;
namespace sensor = volumetric_kit::recon::sensor;
namespace orbbec = volumetric_kit::recon::sensor::orbbec;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

namespace {

constexpr std::uint64_t kPeriod = 33333;  // us, 30 fps
constexpr std::size_t kCameras = 4;
constexpr std::size_t kPrimary = 2;
// Each camera's offset from the primary on the synced clock, as measured.
constexpr std::uint64_t kOffset[kCameras] = {1300, 400, 0, 1600};

orbbec::TriggerGrouper::Config config() {
  orbbec::TriggerGrouper::Config c;
  c.cameras = kCameras;
  c.anchor = kPrimary;
  c.tolerance_us = 5000;
  c.max_wait_us = 50000;
  c.queue_depth = 4;
  return c;
}

// The id of camera `c`'s frame for trigger `t`.
std::uint64_t id_of(std::size_t c, std::uint64_t t) { return t * 10 + c; }
std::uint64_t ts_of(std::size_t c, std::uint64_t t) {
  return 1000000 + t * kPeriod + kOffset[c];
}

int test_perfect_stream() {
  orbbec::TriggerGrouper g(config());
  std::vector<std::uint64_t> released;
  std::uint64_t now = 0;
  for (std::uint64_t t = 0; t < 10; ++t) {
    // Frames of one trigger arrive in scrambled camera order.
    for (const std::size_t c : {3u, 0u, 2u, 1u}) {
      CHECK(!g.take(now, &released));  // nothing complete until the last
      g.add(c, ts_of(c, t), id_of(c, t), now, &released);
      now += 1000;
    }
    const auto group = g.take(now, &released);
    CHECK(group);
    for (std::size_t c = 0; c < kCameras; ++c) {
      CHECK(group->ids[c] == id_of(c, t));
    }
    // Named by the primary's frame, not the earliest.
    CHECK(group->timestamp_us == ts_of(kPrimary, t));
    now += kPeriod - 4000;
  }
  CHECK(released.empty());
  return 0;
}

int test_dropped_frame() {
  // Camera 1 drops trigger 1's frame. The set goes out without it once camera
  // 1's frame for trigger 2 shows it has moved past -- not a whole wait later.
  orbbec::TriggerGrouper g(config());
  std::vector<std::uint64_t> released;
  std::uint64_t now = 0;
  for (std::size_t c = 0; c < kCameras; ++c) {
    g.add(c, ts_of(c, 0), id_of(c, 0), now, &released);
  }
  CHECK(g.take(now, &released));
  for (std::size_t c : {0u, 2u, 3u}) {
    g.add(c, ts_of(c, 1), id_of(c, 1), now, &released);
  }
  CHECK(!g.take(now + 1000, &released));  // camera 1 may still send it
  g.add(1, ts_of(1, 2), id_of(1, 2), now + 2000, &released);
  const auto group = g.take(now + 2000, &released);
  CHECK(group);
  CHECK(group->ids[0] == id_of(0, 1) && group->ids[2] == id_of(2, 1) &&
        group->ids[3] == id_of(3, 1));
  CHECK(!group->ids[1]);
  CHECK(released.empty());
  return 0;
}

int test_silent_camera() {
  // Camera 3 sends nothing at all: each set waits max_wait for it, then goes
  // out without it.
  orbbec::TriggerGrouper g(config());
  std::vector<std::uint64_t> released;
  for (std::size_t c : {0u, 1u, 2u}) {
    g.add(c, ts_of(c, 0), id_of(c, 0), 100, &released);
  }
  CHECK(!g.take(100 + 49000, &released));
  const auto group = g.take(100 + 50000, &released);
  CHECK(group && group->ids[0] && group->ids[1] && group->ids[2] &&
        !group->ids[3]);
  return 0;
}

int test_slow_poll() {
  // Six triggers arrive before one poll: the newest goes out; the older five
  // are released, and the queues never hold more than their depth.
  orbbec::TriggerGrouper g(config());
  std::vector<std::uint64_t> released;
  for (std::uint64_t t = 0; t < 6; ++t) {
    for (std::size_t c = 0; c < kCameras; ++c) {
      g.add(c, ts_of(c, t), id_of(c, t), t * kPeriod, &released);
    }
  }
  // Depth 4: triggers 0 and 1 were let go as trigger 4 and 5 arrived.
  CHECK(released.size() == 2 * kCameras);
  released.clear();
  const auto group = g.take(6 * kPeriod, &released);
  CHECK(group);
  for (std::size_t c = 0; c < kCameras; ++c) {
    CHECK(group->ids[c] == id_of(c, 5));
  }
  CHECK(released.size() == 3 * kCameras);  // triggers 2, 3 and 4
  CHECK(!g.take(6 * kPeriod, &released));
  return 0;
}

int test_clock_beyond_tolerance() {
  // Camera 0's clock runs 8 ms off, past the 5 ms tolerance: its frames never
  // join a trigger. The rig keeps handing out the other three cameras' sets --
  // the ones holding the primary -- and lets camera 0's frames go, rather than
  // handing out camera 0 alone.
  orbbec::TriggerGrouper g(config());
  std::vector<std::uint64_t> released;
  std::uint64_t now = 0;
  int sets = 0;
  for (std::uint64_t t = 0; t < 8; ++t) {
    for (std::size_t c = 0; c < kCameras; ++c) {
      const std::uint64_t ts = ts_of(c, t) + (c == 0 ? 8000 : 0);
      g.add(c, ts, id_of(c, t), now, &released);
    }
    now += kPeriod;
    if (const auto group = g.take(now, &released)) {
      ++sets;
      CHECK(group->ids[kPrimary]);
      CHECK(!group->ids[0]);
    }
  }
  CHECK(sets >= 6);
  return 0;
}

int test_out_of_order_within_camera() {
  // A clock re-sync can step a camera's clock back a little; its frames are
  // still grouped by timestamp.
  orbbec::TriggerGrouper g(config());
  std::vector<std::uint64_t> released;
  g.add(1, ts_of(1, 1), id_of(1, 1), 0, &released);
  g.add(1, ts_of(1, 0), id_of(1, 0), 0, &released);
  for (std::size_t c : {0u, 2u, 3u}) {
    g.add(c, ts_of(c, 0), id_of(c, 0), 0, &released);
  }
  const auto group = g.take(0, &released);
  CHECK(group && group->ids[1] == id_of(1, 0));
  // clear() gives everything held back.
  g.clear(&released);
  CHECK(released.size() == 1 && released[0] == id_of(1, 1));
  return 0;
}

int test_start_order() {
  using M = sensor::OrbbecSyncMode;
  const std::vector<std::string> sns = {"G", "A4", "N", "6G"};
  // The rig as wired: the primary is started last.
  const auto order = orbbec::rig_start_order(
      {M::SecondarySynced, M::SecondarySynced, M::Primary, M::SecondarySynced},
      sns);
  CHECK(order.ok());
  CHECK((order.value() == std::vector<std::size_t>{0, 1, 3, 2}));
  const auto unsupported = [&](std::vector<M> modes) {
    const auto r = orbbec::rig_start_order(modes, sns);
    if (r.ok()) return false;
    std::printf("  refused as expected: %s\n", r.status().message().c_str());
    return r.status().domain() == vr::Status::Code::Unsupported;
  };
  CHECK(unsupported({M::SecondarySynced, M::SecondarySynced, M::SecondarySynced,
                     M::SecondarySynced}));  // no primary
  CHECK(unsupported(
      {M::Primary, M::SecondarySynced, M::Primary, M::SecondarySynced}));
  CHECK(unsupported(
      {M::Standalone, M::SecondarySynced, M::Primary, M::SecondarySynced}));
  return 0;
}

}  // namespace

int main() {
  if (test_perfect_stream() != 0) return 1;
  if (test_dropped_frame() != 0) return 1;
  if (test_silent_camera() != 0) return 1;
  if (test_slow_poll() != 0) return 1;
  if (test_clock_beyond_tolerance() != 0) return 1;
  if (test_out_of_order_within_camera() != 0) return 1;
  if (test_start_order() != 0) return 1;
  std::printf("orbbec grouping tests passed\n");
  return 0;
}
