// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// What rig_viewer prints at the end (examples/viewer/rig_report.hpp): the
// atlas copies' device time, read from profiler snapshots that lag their
// frame by the frames in flight -- a snapshot counts only for a slot whose
// frame copied, and once; a row with no device half is untimed; the newest
// copies are kept -- and a count's rate from its first rise to its last.
// Host only.

#include <chrono>
#include <cstdio>
#include <string>
#include <vector>

#include "rig_report.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"

namespace vkc = volumetric_kit::core;

namespace {

// A row as gfx's profiler publishes one: timed through has_gpu alone.
vkc::StageRow gfx_row(const char* name, double gpu_ms, bool timed) {
  vkc::StageRow row;
  row.name = name;
  row.cpu_ms = 0.1;
  row.gpu_ms = gpu_ms;
  row.has_gpu = timed;
  return row;
}

// A profiler snapshot: a row before the copy's, the copy's, and one after.
std::vector<vkc::StageRow> snapshot(double copy_ms, bool timed) {
  return {gfx_row("acquire", 0.0, false),
          gfx_row(fuse_viewer::kAtlasCopyRow, copy_ms, timed),
          gfx_row("view pass", 1.0, true)};
}

}  // namespace

int main() {
  // Two slots: frames 0 to 5 run in slots 0, 1, 0, 1, 0, 1, and frames 0 and
  // 3 copy. A slot's begin publishes the frame it carried before.
  {
    fuse_viewer::AtlasCopyTimes times(2, 0);
    CHECK(times.report() == "atlas copy, device ms: n/a; 0 untimed\n");
    times.begun(0, {});  // frame 0: nothing published yet
    times.copied(0);
    times.begun(1, {});                   // frame 1
    times.begun(0, snapshot(0.5, true));  // frame 2: frame 0's, a copy
    times.begun(1, snapshot(9.0, true));  // frame 3: frame 1's, none
    times.copied(1);                      // frame 3 copies
    times.begun(0, snapshot(9.0, true));  // frame 4: frame 2's, none
    times.begun(1, snapshot(1.5, true));  // frame 5: frame 3's, a copy
    times.begun(1, snapshot(9.0, true));  // the slot forgot it: once only
    times.begun(7, snapshot(9.0, true));  // no such slot
    times.copied(7);
    const std::string report = times.report();
    std::fputs(report.c_str(), stdout);
    CHECK(report ==
          "atlas copy, device ms (mean / p50 / p95 / max) over 2 timed:    "
          "1.000    0.500    1.500    1.500; 0 untimed\n");
  }

  // A copy the queue did not time, a snapshot without the copy's row, and a
  // row measured the core's way.
  {
    fuse_viewer::AtlasCopyTimes times(1, 0);
    times.copied(0);
    times.begun(0, snapshot(0.7, false));
    CHECK(times.report() == "atlas copy, device ms: n/a; 1 untimed\n");
    times.copied(0);
    times.begun(0, {gfx_row("view pass", 1.0, true)});
    vkc::StageMetrics measured;
    measured.add_gpu(fuse_viewer::kAtlasCopyRow, 0.25);
    times.copied(0);
    times.begun(0, measured.rows());
    CHECK(times.report() ==
          "atlas copy, device ms (mean / p50 / p95 / max) over 1 timed:    "
          "0.250    0.250    0.250    0.250; 1 untimed\n");
  }

  // The newest copies kept.
  {
    fuse_viewer::AtlasCopyTimes times(1, 2);
    for (const double ms : {8.0, 1.0, 3.0}) {
      times.copied(0);
      times.begun(0, snapshot(ms, true));
    }
    CHECK(times.report() ==
          "atlas copy, device ms (mean / p50 / p95 / max) over the newest 2 "
          "timed:    2.000    1.000    3.000    3.000; 0 untimed\n");
  }

  // A count's rate: from the first count above zero to the last rise, so
  // neither the zeros before it nor an idle tail dilutes it.
  {
    using Clock = fuse_viewer::CountRate::Clock;
    const Clock::time_point t0{};
    const auto at = [t0](int seconds) {
      return t0 + std::chrono::seconds(seconds);
    };
    fuse_viewer::CountRate rate;
    CHECK(rate.per_second() == 0.0);
    rate.at(0, at(1));
    rate.at(1, at(2));
    CHECK(rate.per_second() == 0.0);  // one count: no rise yet
    rate.at(1, at(3));
    rate.at(5, at(4));
    CHECK(rate.per_second() == 2.0);  // 4 over the 2 s from 2 to 4
    rate.at(5, at(10));
    rate.at(3, at(11));  // a count that fell
    CHECK(rate.per_second() == 2.0);
  }
  std::puts("example_viewer_rig_report: OK");
  return 0;
}
