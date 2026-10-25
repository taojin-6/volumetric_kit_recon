// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// A sender's per-set log (examples/common/set_log.hpp): each set's counters
// the device's across it, begin to publish; P-frame, I-frame and uncoded sets
// apart, a set never published left out; a row's mean, nearest-rank p50, p95
// and max over the sets that ran it, n/t where some set did not time its
// device half and a dash for a host-only row; the latencies, n/a where no
// stamp is known; and the sets fused again and the payload tails. Host only.

#include <cstdio>
#include <string>

#include "set_log.hpp"
#include "stage_table.hpp"
#include "test_check.hpp"

namespace vkc = volumetric_kit::core;
namespace codec = volumetric_kit::recon::codec;

namespace {

// Whether `table` holds `line` whole.
bool has_line(const std::string& table, const std::string& line) {
  return table.find("\n" + line + "\n") != std::string::npos;
}

}  // namespace

int main() {
  // The nearest rank: the ceil(p n)-th smallest.
  CHECK(vr_example::nearest_rank(std::vector<int>{1, 2, 3, 4}, 50) == 2);
  CHECK(vr_example::nearest_rank(std::vector<int>{1, 2, 3, 4}, 95) == 4);
  CHECK(vr_example::nearest_rank(std::vector<int>{7}, 50) == 7);

  // A device whose counters move as the sets run: two submits and a staging
  // buffer a P-frame set, one submit an uncoded one.
  vkc::DeviceCounters device;
  vr_example::SetLog log([&device] { return device; });
  CHECK(log.table("per set").empty());
  for (int i = 0; i < 4; ++i) {
    vr_example::SetLog::Set& s = log.begin();
    s.arrived(vr_example::system_now_ns(), 0);
    s.rows.add_cpu("frame prep", 1.0 + i);
    s.rows.add_wait("frame prep", 0.5);
    s.rows.add_gpu("frame prep", 0.25 * (i + 1));
    s.rows.add_cpu("write", 0.1);
    if (i == 3) s.rows.add_cpu("block stamps", 2.0);
    device.submits += 2;
    device.host_waits += 2;
    device.staging_buffers += i == 0 ? 1 : 0;
    s.type = codec::FrameType::kInter;
    s.redone = i == 1;
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
  std::puts("example_set_log: OK");
  return 0;
}
