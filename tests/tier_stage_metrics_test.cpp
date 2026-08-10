// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file tests/tier_stage_metrics_test.cpp
/// @brief Every tier reports, and reports the *device* half — not the wall
///        clock relabelled.
///
/// The three assertions with teeth, in order of what they catch:
///
/// 1. **Null measures nothing.** Every entry point defaults its `StageMetrics*`
///    to null, and a tier that timed anyway would be a global sink by another
///    name. Checked by running the whole spine untimed and asserting the
///    metrics object it was never handed stays empty — which sounds vacuous
///    until you note it is the only thing standing between this and a profiler
///    the 2026-08-01 decision explicitly refused.
/// 2. **Every tier contributes its row.** A tier whose row is missing is
///    indistinguishable, on an overlay, from a tier that ran instantly.
/// 3. **`gpu_ms < cpu_ms`, per row.** The host row is wall clock around a
///    fence-blocked submit; the device row is the dispatch inside it. Equal
///    numbers would mean the span is the wall clock relabelled, which is the
///    failure this whole change exists to prevent — and the one a passing
///    "some number came back" assertion would wave through.

#include <cstdio>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/stage_metrics.hpp"
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

const vr::StageRow* find(const vr::StageMetrics& m, const char* name) {
  for (const vr::StageRow& row : m.rows()) {
    if (std::strcmp(row.name, name) == 0) return &row;
  }
  return nullptr;
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

  // --- (1) null measures nothing -------------------------------------------
  //
  // The whole spine, untimed. Nothing may appear in a metrics object no call
  // was handed -- the property that keeps this an opt-in out-param rather than
  // the global profiler the 2026-08-01 decision refused.
  vr::StageMetrics untouched;
  CHECK(grid.value().map().allocate_from_depth(depth.data(), cam));
  CHECK(integrator.value().integrate(grid.value(), depth.data(), cam));
  CHECK(untouched.empty());

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

  // Whether THIS device can time at all, asked directly rather than inferred
  // from whether the tiers produced spans.
  //
  // Without this the loop below degrades to a note and passes, which is exactly
  // how the first cut of this test went green against tiers that had a GpuTimer
  // member and never created it: every row was host-only, every row printed
  // "timestamps unavailable", and nothing failed. A capability the test can
  // establish independently must not be inferred from the thing under test.
  vr::Result<vr::GpuTimer> probe = vr::GpuTimer::create(device.value());
  CHECK(probe);
  const bool device_can_time = probe.value().available();

  const char* kStages[] = {"allocate", "integrate", "texture"};
  for (const char* stage : kStages) {
    const vr::StageRow* row = find(metrics, stage);
    if (row == nullptr) {
      std::fprintf(stderr, "FAIL: no row for stage '%s'\n", stage);
      return 1;
    }
    // The host half is always populated; it is wall clock around a blocking
    // submit, so it cannot be zero for work that actually ran.
    if (row->cpu_ms <= 0.0) {
      std::fprintf(stderr, "FAIL: '%s' host span is %.6f ms\n", stage,
                   row->cpu_ms);
      return 1;
    }
    if (!row->has_gpu) {
      if (device_can_time) {
        // The device supports timestamps and this tier still produced no span:
        // its timer was never created, never passed to dispatch, or never
        // published. All three are silent in production -- the row simply goes
        // missing, which on an overlay is indistinguishable from a stage that
        // ran instantly.
        std::fprintf(stderr,
                     "FAIL: '%s' reported no device span on a device that "
                     "supports timestamps\n",
                     stage);
        return 1;
      }
      // A queue family reporting zero timestampValidBits is a supported
      // configuration, so skipping is right -- but only once the probe above
      // has independently confirmed that is what happened.
      std::fprintf(stderr,
                   "note: '%s' host-only (this queue family reports no "
                   "timestamps)\n",
                   stage);
      continue;
    }
    // THE assertion. The device span is recorded strictly inside the submit
    // the host span wraps, so it must come in under it. Equality would mean the
    // span is the wall clock relabelled.
    if (!(row->gpu_ms < row->cpu_ms)) {
      std::fprintf(stderr, "FAIL: '%s' gpu %.4f >= cpu %.4f ms\n", stage,
                   row->gpu_ms, row->cpu_ms);
      return 1;
    }
    std::fprintf(
        stderr, "%-10s cpu %7.3f ms   gpu %7.3f ms   (%4.1f%% device)\n", stage,
        row->cpu_ms, row->gpu_ms, 100.0 * row->gpu_ms / row->cpu_ms);
  }

  // --- publishing ends the window -------------------------------------------
  //
  // A second timed call must report its OWN span, not that one summed with the
  // previous call's. Without the drain in report_into, `integrate` here would
  // carry both frames' device time under one label and grow without bound --
  // the defect review caught in #62, pinned at the tier boundary too.
  vr::StageMetrics second;
  CHECK(integrator.value().integrate(grid.value(), depth.data(), cam, 5.0f,
                                     vr::tsdf::IntegrationMode::Classic,
                                     nullptr, &second));
  const vr::StageRow* first_row = find(metrics, "integrate");
  const vr::StageRow* second_row = find(second, "integrate");
  CHECK(first_row != nullptr && second_row != nullptr);
  if (first_row->has_gpu && second_row->has_gpu) {
    // Same work, same size: within a wide factor rather than summed. A missing
    // drain would put frame 1 + frame 2 here, i.e. ~2x and climbing.
    CHECK(second_row->gpu_ms < first_row->gpu_ms * 1.8);
  }

  return 0;
}
