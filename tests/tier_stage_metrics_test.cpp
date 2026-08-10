// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file tests/tier_stage_metrics_test.cpp
/// @brief Every tier reports, and reports the *device* half — not the wall
///        clock relabelled.
///
/// The assertions with teeth, in order of what they catch:
///
/// 1. **Every tier contributes its row**, including the breakdown row for the
///    second dispatch `integrate` makes. A tier whose row is missing is
///    indistinguishable, on an overlay, from a tier that ran instantly.
/// 2. **`gpu_ms < cpu_ms`, per row.** The host row is wall clock around a
///    fence-blocked submit; the device row is the dispatch inside it. Equal
///    numbers would mean the span is the wall clock relabelled, which is the
///    failure this whole change exists to prevent — and the one a passing
///    "some number came back" assertion would wave through.
/// 3. **A window is per call, not per lifetime** — from both sides, and this
///    is where the interesting bugs live. Each timer holds at most `max_spans`
///    spans and then silently stops timing, so *running more calls than one
///    window holds and asserting the last still reports a device span* catches
///    both "an untimed call recorded a span anyway" (which would make this a
///    global sink, the thing the 2026-08-01 decision refused) and "publishing
///    did not end the window" (which would report every earlier frame's device
///    time under this frame's label until the window filled). Asserting on
///    counts rather than on the ratio between two ~16 µs spans is what keeps
///    this deterministic: the ratio flaked ~8% of runs on correct code.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/gpu_timer.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/stage_metrics.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"
#include "volumetric_kit/recon/texture/projective_texturer.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"

namespace vr = volumetric_kit::recon;

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
// past which begin() refuses every span and the tier's GPU column freezes. Any
// number above that works; this leaves margin without costing real time (the
// fixture's fuse is a fraction of a millisecond).
constexpr int kPastOneWindow = 40;

const vr::StageRow* find(const vr::StageMetrics& m, const char* name) {
  for (const vr::StageRow& row : m.rows()) {
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
// catches a window carrying more than this call put in it -- N accumulated
// spans against one call's wall clock is a factor, not a coin flip -- where
// `has_gpu` alone would wave a leak through, since spans left over from earlier
// calls publish under the same label and set it.
bool row_reports_both_halves(const vr::StageMetrics& m, const char* stage,
                             bool device_can_time, const char* context) {
  const vr::StageRow* row = find(m, stage);
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
  if (!device_can_time) return true;
  if (!row->has_gpu) {
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

}  // namespace

int main() {
  vr::Result<vr::Instance> instance = vr::Instance::create({});
  if (!instance) {
    std::fprintf(stderr, "no Vulkan instance; skipping\n");
    return 0;
  }
  vr::Result<VkPhysicalDevice> gpu = instance.value().select_physical_device();
  if (!gpu) {
    std::fprintf(stderr, "no compute device; skipping\n");
    return 0;
  }
  vr::Result<vr::Device> device =
      vr::Device::create(instance.value().handle(), gpu.value(), {});
  if (!device) {
    std::fprintf(stderr, "no device; skipping\n");
    return 0;
  }
  vr::Result<vr::Allocator> allocator =
      vr::Allocator::create(instance.value().handle(), device.value());
  CHECK(allocator);

  vr::volume::VoxelGridParams params = vr::volume::VoxelGridParams::defaults();
  params.num_buckets = 64;
  params.bucket_size = 8;
  params.num_blocks = params.num_buckets * params.bucket_size;
  params.block_size = 8;
  params.voxels_per_block =
      params.block_size * params.block_size * params.block_size;
  params.voxel_size = 0.02f;

  const vr::volume::AttributeSpec attrs[] = {{"tsdf", sizeof(float)},
                                             {"weight", sizeof(float)}};
  vr::Result<vr::volume::VoxelBlockGrid> grid =
      vr::volume::VoxelBlockGrid::create(device.value(), allocator.value(),
                                         params, attrs, 2);
  CHECK(grid);

  vr::Result<vr::tsdf::TsdfIntegrator> integrator =
      vr::tsdf::TsdfIntegrator::create(device.value(), allocator.value(), {});
  CHECK(integrator);

  vr::Result<vr::texture::ProjectiveTexturer> texturer =
      vr::texture::ProjectiveTexturer::create(device.value(),
                                              allocator.value());
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
  vr::Result<vr::GpuTimer> probe = vr::GpuTimer::create(device.value());
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
  vr::StageMetrics after_untimed;
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
  vr::StageMetrics metrics;
  vr::Result<std::uint32_t> allocated = grid.value().map().allocate_from_depth(
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

  // "  ..active set" is the compaction dispatch integrate() makes before the
  // fusion one. It is listed here because a stage that runs two kernels and
  // times one reports the other's device time as submit overhead -- the gap
  // between the halves then means something different from what the row above
  // it claims.
  const char* kStages[] = {"allocate", "integrate", "  ..active set",
                           "texture"};
  for (const char* stage : kStages) {
    CHECK(row_reports_both_halves(metrics, stage, device_can_time, "spine"));
    const vr::StageRow* row = find(metrics, stage);
    if (!row->has_gpu) {
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

  // --- publishing ends the window -------------------------------------------
  //
  // The same window bound, from the publishing side: each call must report its
  // OWN span. Without the drain in report_into the spans accumulate across
  // calls -- reporting a running total under each frame's label, the defect
  // review caught in #62 -- and once they reach max_spans the timer stops
  // recording, so the last of these calls comes back host-only.
  //
  // Deterministic where comparing two spans' magnitudes is not: these are
  // ~16 µs on this fixture, small enough that ordinary jitter clears any ratio
  // wide enough to be meaningful, and a span that resolves inside one timestamp
  // tick is a legitimate 0.0 that fails every `<` against another 0.0.
  vr::StageMetrics last;
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
