// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The live viewers' host-side figures (examples/viewer/viewer_metrics.hpp):
// the atlas copies' device time, read from profiler snapshots that lag their
// frame by the frames in flight -- a snapshot counts only for a slot whose
// frame copied, and once; a row with no device half is untimed; the newest
// copies are kept -- a count's rate from its first rise to its last, and over
// the newest window of a second or more; each camera's share of a mesh, by
// the tile its uv0 fall in; the frame each camera textures a remesh from,
// this set's or one held within the hold; and the per-set table's title.
// Host only, and built without the viewer.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include "viewer_metrics.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"

namespace vkc = volumetric_kit::core;
namespace rmesh = volumetric_kit::grid::mesh;
namespace rtex = volumetric_kit::recon::texture;

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

// An unshared triangle's three vertices, each with uv0 (`u`, `v`).
void add_triangle(std::vector<rmesh::Vertex>& vertices, float u, float v) {
  rmesh::Vertex vertex{};
  vertex.uv0 = {u, v};
  vertices.insert(vertices.end(), 3, vertex);
}

// A camera's newest frame, from `set`, taken at `taken_ns`.
fuse_viewer::NewestFrame newest_frame(std::uint64_t set,
                                      std::uint64_t taken_ns) {
  fuse_viewer::NewestFrame newest;
  newest.frame.timestamp_ns = taken_ns;
  newest.set = set;
  return newest;
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
  // A count's rate over the newest window: held under a second, closed at a
  // second or more over the rise since the window opened.
  {
    using Clock = fuse_viewer::WindowRate::Clock;
    const Clock::time_point t0{};
    const auto at = [t0](int ms) { return t0 + std::chrono::milliseconds(ms); };
    fuse_viewer::WindowRate rate(t0);
    CHECK(rate.at(5, at(999)) == 0.0);   // under a second: none closed yet
    CHECK(rate.at(6, at(1000)) == 6.0);  // 6 over the 1 s from 0
    CHECK(rate.at(8, at(1500)) == 6.0);  // held
    CHECK(rate.at(9, at(1999)) == 6.0);
    CHECK(rate.at(9, at(3000)) == 1.5);  // 3 over the 2 s from 1000
    CHECK(rate.at(9, at(4000)) == 0.0);  // no rise
  }

  // Each camera's share: two 2 x 2 tiles side by side in a 4 x 2 atlas, a
  // triangle in each, one untextured (a negative uv0, carried), and one on
  // the first tile's right edge, which is the second tile's.
  {
    rtex::AtlasLayout layout;
    layout.width = 4;
    layout.height = 2;
    layout.tiles = {rtex::AtlasTile{0, 0, 2, 2}, rtex::AtlasTile{2, 0, 2, 2}};
    std::vector<rmesh::Vertex> vertices;
    add_triangle(vertices, 0.25f, 0.5f);     // pixel (1, 1): tile 0
    add_triangle(vertices, 0.75f, 0.5f);     // pixel (3, 1): tile 1
    add_triangle(vertices, -1.62f, -1.31f);  // untextured
    add_triangle(vertices, 0.5f, 0.25f);     // pixel (2, 0.5): tile 1
    const fuse_viewer::TextureShares shares =
        fuse_viewer::count_texture_shares(vertices, layout);
    CHECK(shares.triangles == 4);
    CHECK(shares.untextured == 1);
    CHECK(shares.per_camera.size() == 2);
    CHECK(shares.per_camera[0] == 1);
    CHECK(shares.per_camera[1] == 2);

    const fuse_viewer::TextureShares none =
        fuse_viewer::count_texture_shares({}, layout);
    CHECK(none.triangles == 0 && none.untextured == 0);
    CHECK(none.per_camera.size() == 2);
    CHECK(none.per_camera[0] == 0 && none.per_camera[1] == 0);
  }

  // The frame each camera textures set 5 from, taken at 1000 ns with a hold
  // of 100 ns: this set's, held within the hold, held past it, taken after
  // the set, and none.
  {
    std::vector<std::optional<fuse_viewer::NewestFrame>> newest(5);
    newest[0] = newest_frame(5, 1000);
    newest[1] = newest_frame(4, 900);   // 100 ns before: held
    newest[2] = newest_frame(3, 899);   // 101 ns before: too old
    newest[3] = newest_frame(4, 1001);  // after the set: dropped
    const std::vector<fuse_viewer::TextureSource> sources =
        fuse_viewer::texture_sources(newest, 5, 1000, 100);
    CHECK(sources.size() == 5);
    CHECK(sources[0].frame == &newest[0]->frame && !sources[0].held);
    CHECK(sources[1].frame == &newest[1]->frame && sources[1].held);
    CHECK(sources[2].frame == nullptr);
    CHECK(sources[3].frame == nullptr);
    CHECK(sources[4].frame == nullptr && !sources[4].held);

    // No hold: this set's frames alone, and a held one taken at the set's
    // time.
    const std::vector<fuse_viewer::TextureSource> unheld =
        fuse_viewer::texture_sources(newest, 5, 1000, 0);
    CHECK(unheld[0].frame == &newest[0]->frame && !unheld[0].held);
    CHECK(unheld[1].frame == nullptr);
    newest[1] = newest_frame(4, 1000);
    CHECK(fuse_viewer::texture_sources(newest, 5, 1000, 0)[1].held);
  }

  // The per-set table's title, with the log's oldest sets let go or not.
  CHECK(fuse_viewer::per_set_title(7, 0) == "per set");
  CHECK(fuse_viewer::per_set_title(15000, 3) ==
        "per set (the newest 15000 of 15003)");
  CHECK(fuse_viewer::per_set_title(0, 0) == "per set");

  std::puts("example_viewer_metrics: OK");
  return 0;
}
