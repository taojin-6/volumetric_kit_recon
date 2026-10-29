// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// A sender's per-set log (examples/common/set_log.hpp): each set's counters
// the device's across it, begin to publish; P-frame, I-frame and uncoded sets
// apart, a set never published left out; a row's mean, nearest-rank p50, p95
// and max over the sets that ran it, n/t where some set did not time its
// device half and a dash for a host-only row; a row first seen in a later set
// placed after the row before it there, a breakdown inside its stage; "set"
// waiting its rows' waits since arrival; the latencies, n/a where no stamp is
// known; and the sets fused again and the payload tails. A viewer's sets: a
// remesh's rows merged into the set that ran it, counted once, its wait in
// "set"; uncoded sets tabled by kind; and the newest sets kept. Host only.

#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

#include "set_log.hpp"
#include "stage_table.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"

namespace vkc = volumetric_kit::core;
namespace codec = volumetric_kit::grid::codec;

namespace {

// Whether `table` holds `line` whole.
bool has_line(const std::string& table, const std::string& line) {
  return table.find("\n" + line + "\n") != std::string::npos;
}

}  // namespace

int main() {
  // The nearest rank: the ceil(p n)-th smallest.
  CHECK(vgrid_example::nearest_rank(std::vector<int>{1, 2, 3, 4}, 50) == 2);
  CHECK(vgrid_example::nearest_rank(std::vector<int>{1, 2, 3, 4}, 95) == 4);
  CHECK(vgrid_example::nearest_rank(std::vector<int>{7}, 50) == 7);

  // A device whose counters move as the sets run: two submits and a staging
  // buffer a P-frame set, one submit an uncoded one.
  vkc::DeviceCounters device;
  vr_example::SetLog log([&device] { return device; });
  CHECK(log.table("per set").empty());
  for (int i = 0; i < 4; ++i) {
    vr_example::SetLog::Set& s = log.begin();
    if (i == 0) {  // before the set: not its wait
      s.rows.add_cpu("wait: cameras", 9.0);
      s.rows.add_wait("wait: cameras", 9.0);
    }
    s.arrived(vr_example::system_now_ns(), 0);
    s.rows.add_cpu("frame prep", 1.0 + i);
    s.rows.add_wait("frame prep", 0.5);
    s.rows.add_gpu("frame prep", 0.25 * (i + 1));
    if (i == 2) s.rows.add_cpu("  ..tail", 0.2);  // a breakdown, this set's
    s.rows.add_cpu("write", 0.1);
    if (i == 3) s.rows.add_cpu("block stamps", 2.0);
    device.submits += 2;
    device.host_waits += 2;
    device.staging_buffers += i == 0 ? 1 : 0;
    s.type = codec::FrameType::kInter;
    s.redone = i == 1;
    // Longer than its rows' waits, so "set" holds them whole.
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    s.published();
    s.published();  // once only
  }
  {
    vr_example::SetLog::Set& s = log.begin();
    s.arrived(0, 0);
    s.rows.add_cpu("frame prep", 1.0);
    s.rows.add_device("frame prep", vkc::DeviceTime::NotTimed);
    device.submits += 1;
    s.published();
  }
  log.begin();  // never published: an exhausted source's last poll

  const std::string table = log.table("per set");
  std::fputs(table.c_str(), stdout);
  CHECK(table.find("per set: 4 P-frame sets") != std::string::npos);
  CHECK(table.find("per set: 1 uncoded sets") != std::string::npos);
  CHECK(table.find("I-frame") == std::string::npos);
  // Host busy 0.5, 1.5, 2.5, 3.5: mean 2.0, p50 1.5, p95 3.5; device 0.25 to
  // 1.0.
  CHECK(has_line(table,
                 "  frame prep               4     2.000    1.500    3.500    "
                 "3.500     0.500    0.500    0.500    0.500     0.625    "
                 "0.500    1.000    1.000"));
  // A host-only row, and one only a set ran.
  CHECK(has_line(table,
                 "  write                    4     0.100    0.100    0.100    "
                 "0.100     0.000    0.000    0.000    0.000         -        "
                 "-        -        -"));
  CHECK(table.find("  block stamps             1     2.000") !=
        std::string::npos);
  // A breakdown only a later set ran sits inside its stage, before the row
  // after it there.
  const std::size_t prep_at = table.find("\n  frame prep ");
  const std::size_t tail_at = table.find("\n    ..tail ");
  const std::size_t write_at = table.find("\n  write ");
  CHECK(prep_at < tail_at && tail_at < write_at &&
        write_at != std::string::npos);
  // "set" waits what its rows waited from arrival to publish, 0.5 ms, not the
  // wait before it.
  const std::size_t set_at = table.find("\n  set ");
  CHECK(set_at != std::string::npos);
  const std::string set_line =
      table.substr(set_at + 1, table.find('\n', set_at + 1) - set_at - 1);
  const std::string set_wait =
      "     0.500    0.500    0.500    0.500         -        -        -  "
      "      -";
  CHECK(set_line.size() > set_wait.size() &&
        set_line.compare(set_line.size() - set_wait.size(), set_wait.size(),
                         set_wait) == 0);
  // The uncoded set's device half was not timed.
  CHECK(table.find("     n/t      n/t      n/t      n/t") != std::string::npos);
  CHECK(has_line(table,
                 "    submits               2.000    2.000    2.000    2.000"));
  CHECK(has_line(table,
                 "    staging buffers       0.250    0.000    1.000    1.000"));
  CHECK(has_line(table,
                 "    submits               1.000    1.000    1.000    1.000"));
  // Arrival is known for the P-frame sets, capture for none.
  CHECK(table.find("    arrival->published      n/a") != std::string::npos);
  CHECK(table.find("    capture->published      n/a      n/a      n/a      "
                   "n/a") != std::string::npos);
  CHECK(has_line(table, "    fused again 1, payload tails 0"));
  const vr_example::SetLog::Set& first = log.sets().front();
  CHECK(first.is_published() && first.arrival_ms().has_value() &&
        !first.capture_ms().has_value() && first.counted().submits == 2);
  CHECK(log.dropped() == 0 && log.sets().size() == 6);

  // A viewer's sets, as rig_viewer records them: each set's own rows, and on
  // a set that meshed, the remesh's rows -- timed into rows the viewer holds
  // between remeshes -- merged after them, before the mesh's hand-over
  // publishes the set. A remesh's rows land in their set, after its fuse
  // rows, counted once; their wait counts in its "set"; the meshed and
  // unmeshed sets are tabled apart, in the order first seen; and the log
  // keeps the newest sets.
  vr_example::SetLog viewer(nullptr, 4);
  vkc::StageMetrics remesh;  // the newest remesh's rows
  for (int i = 0; i < 6; ++i) {
    vr_example::SetLog::Set& s = viewer.begin();
    s.kind = "unmeshed";
    s.rows.add_cpu("poll", 0.1);
    s.arrived(vr_example::system_now_ns(), 0);
    s.rows.add_cpu("integrate", 1.0);
    if (i % 2 == 1) {  // sets 1, 3 and 5 mesh
      remesh.clear();
      remesh.add_cpu("extract", 2.0 + i);
      remesh.add_cpu("texture", 1.0);
      remesh.add_wait("texture", 0.75);
      // Longer than the rows' waits, so "set" holds them whole.
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      s.rows.merge(remesh);
      s.kind = "meshed";
      s.published();
    }
    s.published();  // an unmeshed set's last stage; a meshed one's no-op
  }
  CHECK(viewer.dropped() == 2 && viewer.sets().size() == 4);
  const std::string sets = viewer.table("per set");
  std::fputs(sets.c_str(), stdout);
  // Sets 2 to 5 kept: 2 and 4 unmeshed, first seen, then 3 and 5.
  const std::size_t unmeshed_at = sets.find("per set: 2 unmeshed sets");
  const std::size_t meshed_at = sets.find("per set: 2 meshed sets");
  CHECK(unmeshed_at == 0 && meshed_at != std::string::npos);
  CHECK(sets.find("uncoded") == std::string::npos);
  const std::string unmeshed = sets.substr(0, meshed_at);
  const std::string meshed = sets.substr(meshed_at);
  CHECK(unmeshed.find("extract") == std::string::npos &&
        unmeshed.find("texture") == std::string::npos);
  // Sets 3 and 5's extracts, 5 and 7 ms, once each.
  CHECK(has_line(meshed,
                 "  extract                  2     6.000    5.000    7.000    "
                 "7.000     0.000    0.000    0.000    0.000         -        "
                 "-        -        -"));
  CHECK(has_line(meshed,
                 "  texture                  2     0.250    0.250    0.250    "
                 "0.250     0.750    0.750    0.750    0.750         -        "
                 "-        -        -"));
  const std::size_t integrate_at = meshed.find("\n  integrate ");
  const std::size_t extract_at = meshed.find("\n  extract ");
  const std::size_t meshed_set_at = meshed.find("\n  set ");
  CHECK(meshed.find("\n  poll ") < integrate_at && integrate_at < extract_at &&
        extract_at < meshed_set_at && meshed_set_at != std::string::npos);
  const std::string meshed_set =
      meshed.substr(meshed_set_at + 1,
                    meshed.find('\n', meshed_set_at + 1) - meshed_set_at - 1);
  const std::string remesh_wait =
      "     0.750    0.750    0.750    0.750         -        -        -  "
      "      -";
  CHECK(meshed_set.size() > remesh_wait.size() &&
        meshed_set.compare(meshed_set.size() - remesh_wait.size(),
                           remesh_wait.size(), remesh_wait) == 0);
  CHECK(meshed.find("    arrival->published      n/a") == std::string::npos);
  std::puts("example_set_log: OK");
  return 0;
}
