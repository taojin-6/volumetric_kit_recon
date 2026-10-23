// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file tests/tier_stage_metrics_test.cpp
/// @brief Every tier reports, and reports the *device* half — not the wall
///        clock relabelled.
///
/// The assertions with teeth, in order of what they catch:
///
/// 1. **Every tier contributes its row**, including the breakdown rows for
///    the parts of a stage. A tier whose row is missing is indistinguishable,
///    on an overlay, from a tier that ran instantly.
/// 2. **`gpu_ms < cpu_ms` and `wait_ms <= cpu_ms`, per row.** The host row is
///    wall clock around a fence-blocked submit, its wait the fence's; the
///    device row is the work inside it. Equal numbers would mean the span is
///    the wall clock relabelled, which is the failure this whole change
///    exists to prevent — and the one a passing "some number came back"
///    assertion would wave through.
/// 3. **A breakdown is inside its stage, device time included**: a stage's
///    device half holds its breakdowns', the totals skip them, and two
///    breakdowns of one submit do not hold each other's.
/// 4. **Timing is per call, not per lifetime** — from both sides. A timer's
///    pool holds a fixed number of queries, so *running more calls than it
///    holds and asserting the last still reports a device span* catches both
///    "an untimed call recorded a span anyway" (which would make this a
///    global sink, the thing the 2026-08-01 decision refused) and "a call
///    kept its queries" (the timer would stop timing). Null metrics also
///    leave a scope open around the call with no device half. Asserting on
///    counts rather than on the ratio between two ~16 µs spans is what keeps
///    this deterministic: the ratio flaked ~8% of runs on correct code.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/gpu_timer.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"
#include "volumetric_kit/recon/texture/projective_texturer.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"

#include "buffer_readback.hpp"
#include "gpu_test.hpp"

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;

#define CHECK(cond)                                                          \
  do {                                                                       \
    if (!(cond)) {                                                           \
      std::fprintf(stderr, "FAIL (%s:%d): %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                              \
    }                                                                        \
  } while (0)

namespace {

constexpr std::uint32_t kWidth = 64;
constexpr std::uint32_t kHeight = 64;

// More calls than one window holds: GpuTimer::create's default max_spans is 32,
// past which a timer whose calls kept their queries finds no free range and
// the tier's GPU column reads n/t. Any number above that works; this leaves
// margin without costing real time (the fixture's fuse is a fraction of a
// millisecond).
constexpr int kPastOneWindow = 40;

const vkc::StageRow* find(const vkc::StageMetrics& m, const char* name) {
  for (const vkc::StageRow& row : m.rows()) {
    if (std::strcmp(row.name, name) == 0) return &row;
  }
  return nullptr;
}

// One call's row: present, host-timed, and -- where the device can time at all
// -- carrying a device span that fits inside the host one.
//
// `gpu_ms < cpu_ms` is the load-bearing half and it is structural, not a
// tolerance: the span is recorded inside the very submit the host row wraps, so
// under it is the only place it can land. That makes it the assertion that
// catches a call reporting more than it put in -- N accumulated spans against
// one call's wall clock is a factor, not a coin flip -- where a Measured state
// alone would wave a leak through, since spans left over from earlier calls
// publish under the same label and set it. The wait is part of the host span.
bool row_reports_both_halves(const vkc::StageMetrics& m, const char* stage,
                             bool device_can_time, const char* context) {
  const vkc::StageRow* row = find(m, stage);
  if (row == nullptr) {
    std::fprintf(stderr, "FAIL (%s): no row for stage '%s'\n", context, stage);
    return false;
  }
  // Wall clock around a blocking submit, so it cannot be zero for work that
  // ran.
  if (row->cpu_ms <= 0.0) {
    std::fprintf(stderr, "FAIL (%s): '%s' host span is %.6f ms\n", context,
                 stage, row->cpu_ms);
    return false;
  }
  if (!(row->wait_ms >= 0.0 && row->wait_ms <= row->cpu_ms)) {
    std::fprintf(stderr, "FAIL (%s): '%s' wait %.4f outside host %.4f ms\n",
                 context, stage, row->wait_ms, row->cpu_ms);
    return false;
  }
  if (!device_can_time) return true;
  if (row->device != vkc::DeviceTime::Measured) {
    // The device supports timestamps and this tier still produced no span: its
    // timer was never created, never reached the dispatch, or was never
    // published. All three are silent in production -- the row simply goes
    // missing, which on an overlay is indistinguishable from a stage that ran
    // instantly.
    std::fprintf(stderr,
                 "FAIL (%s): '%s' reported no device span on a device that "
                 "supports timestamps\n",
                 context, stage);
    return false;
  }
  if (!(row->gpu_ms < row->cpu_ms)) {
    std::fprintf(stderr, "FAIL (%s): '%s' gpu %.4f >= cpu %.4f ms\n", context,
                 stage, row->gpu_ms, row->cpu_ms);
    return false;
  }
  return true;
}

// A camera looking down +Z at a plane 1 m away -- enough surface to allocate
// blocks, fuse them, and mesh something.
vr::DepthCameraParams plane_camera() {
  vr::DepthCameraParams cam{};
  cam.width = kWidth;
  cam.height = kHeight;
  cam.fx = 64.0f;
  cam.fy = 64.0f;
  cam.cx = static_cast<float>(kWidth) * 0.5f;
  cam.cy = static_cast<float>(kHeight) * 0.5f;
  cam.min_depth = 0.1f;
  cam.max_depth = 5.0f;
  cam.cam_to_world = vr::Mat4f(1.0f);
  return cam;
}

int gpu_main(vr_test::GpuContext& gpu) {
  const vr::volume::VoxelGridParams params{
      /*voxel_size=*/0.02f,     /*block_size=*/8,
      /*voxels_per_block=*/512,
      /*trunc_dist=*/0.04f,     /*bucket_size=*/8,
      /*num_buckets=*/64,
      /*num_blocks=*/512,       /*max_chain=*/128};

  const vr::volume::AttributeSpec attrs[] = {{"tsdf", sizeof(float)},
                                             {"weight", sizeof(float)}};
  vkc::Result<vr::volume::VoxelBlockGrid> grid =
      vr::volume::VoxelBlockGrid::create(gpu.device, gpu.allocator, params,
                                         attrs, 2);
  CHECK(grid);

  vkc::Result<vr::tsdf::TsdfIntegrator> integrator =
      vr::tsdf::TsdfIntegrator::create(gpu.device, gpu.allocator);
  CHECK(integrator);

  vkc::Result<vr::texture::ProjectiveTexturer> texturer =
      vr::texture::ProjectiveTexturer::create(gpu.device, gpu.allocator);
  CHECK(texturer);

  const vr::DepthCameraParams cam = plane_camera();
  const std::vector<float> depth(kWidth * kHeight, 1.0f);

  // Whether THIS device can time at all, asked directly rather than inferred
  // from whether the tiers produced spans.
  //
  // Without this the loops below degrade to a note and pass, which is exactly
  // how the first cut of this test went green against tiers that had a GpuTimer
  // member and never created it: every row was host-only, every row printed
  // "timestamps unavailable", and nothing failed. A capability the test can
  // establish independently must not be inferred from the thing under test.
  vkc::Result<vkc::GpuTimer> probe = vkc::GpuTimer::create(gpu.device);
  CHECK(probe);
  const bool device_can_time = probe.value().available();

  // --- (1) an untimed call must not consume the window ----------------------
  //
  // The whole spine, untimed, far more times than one window holds. A tier that
  // recorded spans regardless of its null out-param -- a global sink by another
  // name -- would fill its timer here, and the timed call that follows would
  // come back host-only on a device that can time. Asserting instead that a
  // StageMetrics no call was handed stays empty proves only that a default-
  // constructed vector is empty; it holds against precisely that tier.
  for (int n = 0; n < kPastOneWindow; ++n) {
    CHECK(grid.value().map().allocate_from_depth(depth.data(), cam));
    CHECK(integrator.value().integrate(grid.value(), depth.data(), cam));
  }
  vkc::StageMetrics after_untimed;
  CHECK(integrator.value().integrate(grid.value(), depth.data(), cam, 5.0f,
                                     vr::tsdf::IntegrationMode::Classic,
                                     nullptr, &after_untimed));
  // Both failure shapes land here. A tier that filled its window while unasked
  // has none left, so this reports no device span at all; a tier that recorded
  // and never drained publishes those calls' spans under this call's label, so
  // the device half overruns the host one.
  CHECK(row_reports_both_halves(after_untimed, "integrate", device_can_time,
                                "after untimed calls"));

  // --- (2) + (3) every tier reports, and the halves differ ------------------
  // Cleared, so the allocation takes blocks for the integrate to compact.
  CHECK(grid.value().clear().ok());
  vkc::StageMetrics metrics;
  vkc::Result<std::uint32_t> allocated = grid.value().map().allocate_from_depth(
      depth.data(), cam, nullptr, &metrics);
  CHECK(allocated);
  CHECK(integrator.value().integrate(grid.value(), depth.data(), cam, 5.0f,
                                     vr::tsdf::IntegrationMode::Classic,
                                     nullptr, &metrics));

  // A mesh the texturer can chew on: three vertices is enough, since what is
  // under test is that the tier *reports*, not what it computes (that is
  // texture_projective_test's job).
  vr::mesh::Mesh mesh;
  mesh.vertices.resize(3);
  mesh.vertices[0].position = {0.0f, 0.0f, 1.0f};
  mesh.vertices[1].position = {0.1f, 0.0f, 1.0f};
  mesh.vertices[2].position = {0.0f, 0.1f, 1.0f};
  mesh.indices = {0, 1, 2};
  CHECK(texturer.value().texture(mesh, depth.data(), cam, 0.02f, &metrics));

  const char* kStages[] = {"allocate", "integrate", "texture"};
  for (const char* stage : kStages) {
    CHECK(row_reports_both_halves(metrics, stage, device_can_time, "spine"));
    const vkc::StageRow* row = find(metrics, stage);
    if (row->device != vkc::DeviceTime::Measured) {
      // A queue family reporting zero timestampValidBits is a supported
      // configuration, so reporting host-only is right -- but only once the
      // probe above has independently confirmed that is what happened.
      std::fprintf(stderr,
                   "note: '%s' host-only (this queue family reports no "
                   "timestamps)\n",
                   stage);
      continue;
    }
    std::fprintf(
        stderr, "%-14s cpu %7.3f ms   gpu %7.3f ms   (%4.1f%% device)\n", stage,
        row->cpu_ms, row->gpu_ms, 100.0 * row->gpu_ms / row->cpu_ms);
  }

  // "  ..active set" is the compaction integrate() records ahead of the
  // fusion, in the same submit: a breakdown with its device half alone, which
  // "integrate"'s holds too, since a tile counts to every scope open around
  // it. So both totals skip it.
  {
    const vkc::StageRow* active_set = find(metrics, "  ..active set");
    const vkc::StageRow* integrate = find(metrics, "integrate");
    CHECK(active_set != nullptr && active_set->cpu_ms == 0.0);
    CHECK(active_set->device == vkc::DeviceTime::Measured || !device_can_time);
    if (active_set->device == vkc::DeviceTime::Measured) {
      CHECK(integrate->gpu_ms >= active_set->gpu_ms);
    }
    CHECK(metrics.total_gpu_ms() == metrics.total_gpu_ms("  ..active set"));
    CHECK(metrics.total_cpu_ms() == metrics.total_cpu_ms("  ..active set"));
  }

  // --- a set allocated in its fuse's submit (the Fuser's steady state) -------
  //
  // "integrate" holds the allocation round's tiles, the round is its
  // "  ..allocate" breakdown, and "  ..active set" is the compaction alone: two
  // breakdowns of one submit do not hold each other's time. A narrow camera
  // over a large image makes the round far longer than the fuse, so a
  // compaction row that still held the round would overrun "integrate".
  {
    vr::DepthCameraParams narrow = cam;
    narrow.width = narrow.height = 1024;
    narrow.fx = narrow.fy = 16384.0f;
    narrow.cx = narrow.cy = 512.0f;
    const std::vector<float> plane(std::size_t{1024} * 1024, 1.0f);
    vkc::Result<vkc::Buffer> on_device = vr_test::upload_device_buffer(
        gpu.device, gpu.allocator, plane.data(), plane.size() * sizeof(float));
    CHECK(on_device);
    const vr::tsdf::FrameInput frame{
        {vkc::StorageInput(on_device.value()), narrow}, nullptr};
    vkc::StageMetrics fused;
    CHECK(integrator.value().allocate_and_integrate(
        grid.value(), {frame}, 5.0f, vr::tsdf::IntegrationMode::Classic,
        nullptr, &fused));
    CHECK(row_reports_both_halves(fused, "integrate", device_can_time,
                                  "fused set"));
    CHECK(find(fused, "allocate") == nullptr);
    const vkc::StageRow* round = find(fused, "  ..allocate");
    const vkc::StageRow* compaction = find(fused, "  ..active set");
    const vkc::StageRow* whole = find(fused, "integrate");
    CHECK(round != nullptr && compaction != nullptr);
    CHECK(round->cpu_ms == 0.0 && compaction->cpu_ms == 0.0);
    if (device_can_time) {
      CHECK(round->device == vkc::DeviceTime::Measured &&
            compaction->device == vkc::DeviceTime::Measured);
      CHECK(round->gpu_ms > 0.0 && compaction->gpu_ms >= 0.0);
      CHECK(round->gpu_ms + compaction->gpu_ms <= whole->gpu_ms + 1e-9);
      std::fprintf(stderr,
                   "fused set: integrate %.4f ms, ..allocate %.4f, "
                   "..active set %.4f\n",
                   whole->gpu_ms, round->gpu_ms, compaction->gpu_ms);
    }
    CHECK(fused.total_gpu_ms() == whole->gpu_ms);
    CHECK(fused.total_cpu_ms() == whole->cpu_ms);
  }

  // --- the mesh tier: an extract over its compaction and its dispatch --------
  {
    vkc::Result<vr::mesh::MarchingCubes> mc =
        vr::mesh::MarchingCubes::create(gpu.device, gpu.allocator);
    CHECK(mc);
    // The fuses above moved the active set, so this one compacts.
    vkc::StageMetrics first;
    CHECK(mc.value().extract_device(grid.value(), 0.0f, nullptr, &first));
    CHECK(row_reports_both_halves(first, "marching cubes", device_can_time,
                                  "extract"));
    const vkc::StageRow* whole = find(first, "marching cubes");
    const vkc::StageRow* compaction = find(first, "  ..active set");
    const vkc::StageRow* dispatch = find(first, "  ..dispatch");
    CHECK(compaction != nullptr && dispatch != nullptr);
    CHECK(compaction->cpu_ms <= whole->cpu_ms &&
          dispatch->cpu_ms <= whole->cpu_ms);
    if (device_can_time) {
      CHECK(compaction->device == vkc::DeviceTime::Measured &&
            dispatch->device == vkc::DeviceTime::Measured);
      // Its own tiles too: the first extract grows the arena.
      CHECK(compaction->gpu_ms + dispatch->gpu_ms <= whole->gpu_ms + 1e-9);
    }
    CHECK(first.total_gpu_ms() == whole->gpu_ms);
    CHECK(first.total_cpu_ms() == whole->cpu_ms);

    // Nothing changed since, so the map's list holds and nothing compacts; a
    // steady extract is its one dispatch's submit, which is the stage's tile.
    vkc::StageMetrics again;
    vr::mesh::ExtractTimings timings;
    CHECK(mc.value().extract_device(grid.value(), 0.0f, &timings, &again));
    CHECK(find(again, "  ..active set") == nullptr);
    const vkc::StageRow* steady = find(again, "  ..dispatch");
    CHECK(steady != nullptr);
    if (device_can_time && timings.dispatches == 1) {
      CHECK(std::fabs(steady->gpu_ms - find(again, "marching cubes")->gpu_ms) <
            1e-9);
    }

    // Null metrics tile nothing: a scope open around the extract gets its
    // fence waits but no device half.
    vkc::StageMetrics outer;
    {
      const vkc::StageScope around(outer, "around");
      CHECK(mc.value().extract_device(grid.value()));
    }
    CHECK(outer.rows().size() == 1);
    CHECK(outer.rows()[0].device == vkc::DeviceTime::None);
    CHECK(outer.rows()[0].wait_ms <= outer.rows()[0].cpu_ms);
  }

  // The cull is the frame's own, so the next integrate compacts again, with
  // nothing allocated since, and reports it again.
  {
    vkc::StageMetrics again;
    CHECK(integrator.value().integrate(grid.value(), depth.data(), cam, 5.0f,
                                       vr::tsdf::IntegrationMode::Classic,
                                       nullptr, &again));
    CHECK(find(again, "integrate") != nullptr);
    CHECK(find(again, "  ..active set") != nullptr);
  }

  // --- the same compaction, asked for from both positions --------------------
  //
  // Breakdown or stage is a property of where the call sits, not of what it
  // does. Under a caller's own scope the prefix keeps the sub-row out of that
  // caller's host total; at top level the same prefix would leave the row out
  // of every total, so the row is named as the stage it is.
  {
    vkc::StageMetrics nested;
    {
      vkc::StageScope outer(nested, "fuse");
      CHECK(grid.value().map().compact_active_blocks(&nested));
    }
    CHECK(find(nested, "  ..active set") != nullptr);
    CHECK(find(nested, "active set") == nullptr);
    // Counted once, through the stage that wraps it.
    CHECK(nested.total_cpu_ms() == find(nested, "fuse")->cpu_ms);

    vkc::StageMetrics top;
    CHECK(grid.value().map().compact_active_blocks(&top));
    CHECK(find(top, "active set") != nullptr);
    CHECK(find(top, "  ..active set") == nullptr);
    // The caller's only stage: it must reach the total it will be read from.
    CHECK(top.total_cpu_ms() > 0.0);
  }

  // The frustum compaction reports through the same row -- a caller who
  // switches to it to make the trip cheaper has to be able to read what that
  // bought, not watch the row vanish.
  {
    vkc::StageMetrics frustum;
    CHECK(grid.value().map().compact_active_blocks_in_frusta_on_device(
        {vr::volume::make_frustum_planes(cam.fx, cam.fy, cam.cx, cam.cy,
                                         cam.width, cam.height, cam.min_depth,
                                         cam.max_depth, cam.cam_to_world)},
        &frustum));
    CHECK(row_reports_both_halves(frustum, "active set", device_can_time,
                                  "frustum compaction"));
  }

  // --- publishing ends the window -------------------------------------------
  //
  // The same window bound, from the publishing side: each call must report its
  // OWN span. A call that kept its queries would report a running total under
  // each frame's label, the defect review caught in #62, and once they reach
  // max_spans the timer stops timing, so the last of these calls comes back
  // untimed.
  //
  // Deterministic where comparing two spans' magnitudes is not: these are
  // ~16 µs on this fixture, small enough that ordinary jitter clears any ratio
  // wide enough to be meaningful, and a span that resolves inside one timestamp
  // tick is a legitimate 0.0 that fails every `<` against another 0.0.
  vkc::StageMetrics last;
  for (int n = 0; n < kPastOneWindow; ++n) {
    last.clear();
    CHECK(integrator.value().integrate(grid.value(), depth.data(), cam, 5.0f,
                                       vr::tsdf::IntegrationMode::Classic,
                                       nullptr, &last));
  }
  CHECK(row_reports_both_halves(last, "integrate", device_can_time,
                                "after repeated timed calls"));

  return 0;
}

}  // namespace

int main() { return vr_test::run_on_gpu(gpu_main); }
