// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Host OOM returns a Status, preserves existing voxel data, and permits retry.
// A live fuser backs off a refused grow, then retries after its private delay.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <new>
#include <string>
#include <vector>

#include "gpu_test.hpp"
#include "sphere_scene.hpp"
#include "test_allocation_failure.hpp"
#include "test_check.hpp"
#include "volumetric_kit/core/vulkan/instance.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/tsdf/fuser.hpp"
#include "volumetric_kit/recon/volume/grid_growth.hpp"

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;
namespace vol = vr::volume;
namespace tsdf = vr::tsdf;

namespace {

vkc::Result<vol::VoxelBlockGrid> make_grid(vkc::Device& device,
                                           vkc::Allocator& allocator,
                                           int buckets) {
  const vol::VoxelGridParams params{0.01f, 8,       512,         0.04f,
                                    8,     buckets, 8 * buckets, 128};
  const vol::AttributeSpec attrs[] = {{"tsdf", 4}, {"weight", 4}, {"color", 4}};
  return vol::VoxelBlockGrid::create(device, allocator, params, attrs, 3);
}

void fail_allocation(std::size_t bytes, const void* site = nullptr) {
  vr_test::allocation_failure = {};
  vr_test::allocation_failure.bytes = bytes;
  vr_test::allocation_failure.armed = true;
  vr_test::allocation_failure.site = site;
}

int gpu_main(vr_test::GpuContext& gpu) {
  std::vector<vr_test::SphereView> views;
  views.reserve(3);
  for (int c = 0; c < 3; ++c) views.push_back(vr_test::sphere_view(c));
  std::vector<tsdf::FrameInput> frames;
  for (auto& view : views) {
    frames.push_back(
        {{vkc::StorageInput(view.depth.data()), view.cam}, nullptr});
  }

  auto grid = make_grid(gpu.device, gpu.allocator, 127);
  auto fuser = tsdf::Fuser::create(gpu.device, gpu.allocator);
  CHECK(grid.ok() && fuser.ok());
  CHECK(fuser->fuse(*grid, frames).ok());
  const auto before_blocks = vr_test::blocks_of(*grid);
  const auto before = vr_test::read_attribute<float>(gpu.device, gpu.allocator,
                                                     *grid, "weight");
  CHECK(before_blocks.ok() && before.ok());
  const std::uint32_t tick = grid->map().tick();
  auto& failure = vr_test::allocation_failure;

  // The input-vector allocation used to throw across the public Fuser API.
  fail_allocation(frames.size() * sizeof(vol::DepthInput));
  auto refused = fuser->fuse(*grid, frames);
  failure.armed = false;
  CHECK(failure.injected && !refused.ok());
  CHECK(refused.status().domain() == vkc::Status::Code::OutOfMemory);
  CHECK(grid->map().tick() == tick);
  auto after = vr_test::read_attribute<float>(gpu.device, gpu.allocator, *grid,
                                              "weight");
  CHECK(after.ok() && *before == *after);

  // Resize allocates its attribute-buffer owners before any Vulkan work.
  // Capture that site in a direct grow; later fuser calls must fail there,
  // even when the validation layer first allocates the same number of bytes.
  fail_allocation(3 * sizeof(vkc::Buffer));
  const auto refused_resize = vol::grow_grid(*grid);
  failure.armed = false;
  CHECK(failure.injected &&
        refused_resize.domain() == vkc::Status::Code::OutOfMemory);
  CHECK(refused_resize.message().find("grow_grid") != std::string::npos);
  const void* resize_site = failure.injected_site;
  CHECK(resize_site != nullptr && grid->grid().num_buckets == 127);

  // Fail the heap-rebuild scratch after resize has installed its new table.
  // The helper must roll back before returning OutOfMemory; otherwise later
  // fusion sees a grown map with the old, undersized attribute arrays.
  fail_allocation(std::size_t(grid->grid().num_blocks) * 2);
  const auto failed_grow = vol::grow_grid(*grid);
  failure.armed = false;
  CHECK(failure.injected &&
        failed_grow.domain() == vkc::Status::Code::OutOfMemory);
  CHECK(grid->grid().num_buckets == 127);
  const auto kept = vr_test::blocks_of(*grid);
  CHECK(kept.ok() && *kept == *before_blocks);
  after = vr_test::read_attribute<float>(gpu.device, gpu.allocator, *grid,
                                         "weight");
  CHECK(after.ok() && *before == *after);
  CHECK(vol::grow_grid(*grid).ok());
  CHECK(grid->grid().num_buckets == 254);
  after = vr_test::read_attribute<float>(gpu.device, gpu.allocator, *grid,
                                         "weight");
  CHECK(after.ok() && after->size() == 2 * before->size());
  CHECK(std::equal(before->begin(), before->end(), after->begin()));
  CHECK(fuser->fuse(*grid, frames).ok());

  // A failed grow-ahead need not reject a strict set already fully present.
  {
    auto strict = tsdf::Fuser::create(gpu.device, gpu.allocator);
    auto near_full = make_grid(gpu.device, gpu.allocator, 101);
    CHECK(strict.ok() && near_full.ok());
    CHECK(strict->fuse(*near_full, frames).ok());
    CHECK(near_full->map().load_factor().value() >
          vol::VoxelHashMap::kGrowThreshold);
    const auto before_tick = near_full->map().tick();
    fail_allocation(3 * sizeof(vkc::Buffer), resize_site);
    const auto report = strict->fuse(*near_full, frames);
    failure.armed = false;
    CHECK(failure.injected && report.ok() && report->dropped == 0);
    CHECK(report->growth_error.domain() == vkc::Status::Code::OutOfMemory);
    CHECK(near_full->grid().num_buckets == 101);
    CHECK(near_full->map().tick() == before_tick + 1);
  }

  // Two frames keep their input-buffer array distinct from the three
  // attribute-buffer owners whose allocation the following cases fail.
  const std::vector<tsdf::FrameInput> pair{frames[0], frames[1]};

  // A resize OOM on an incomplete strict set is reported before integration.
  {
    auto strict = tsdf::Fuser::create(gpu.device, gpu.allocator);
    auto small = make_grid(gpu.device, gpu.allocator, 8);
    CHECK(strict.ok() && small.ok());
    const auto before_tick = small->map().tick();
    fail_allocation(3 * sizeof(vkc::Buffer), resize_site);
    const auto failed = strict->fuse(*small, pair);
    failure.armed = false;
    CHECK(failure.injected && !failed.ok());
    CHECK(failed.status().domain() == vkc::Status::Code::OutOfMemory);
    CHECK(failed.status().message().find("grow_grid") != std::string::npos);
    CHECK(small->map().tick() == before_tick);
    auto weights = vr_test::read_attribute<float>(gpu.device, gpu.allocator,
                                                  *small, "weight");
    CHECK(weights.ok() &&
          std::all_of(weights->begin(), weights->end(),
                      [](float weight) { return weight == 0.0f; }));
    CHECK(strict->fuse(*small, pair).ok());
  }

  // An actual failed grow, not a synthetic policy decline, exercises private
  // live backoff. Attribute-buffer bookkeeping has three Buffer owners.
  tsdf::FuserConfig live;
  live.max_grows_per_set = 2;
  live.allow_partial = true;
  auto partial = tsdf::Fuser::create(gpu.device, gpu.allocator, live);
  auto small = make_grid(gpu.device, gpu.allocator, 8);
  CHECK(partial.ok() && small.ok());
  const auto refused_tick = small->map().tick();
  fail_allocation(3 * sizeof(vkc::Buffer), resize_site);
  auto report = partial->fuse(*small, pair);
  failure.armed = false;
  CHECK(failure.injected && report.ok() && report->grows == 0);
  CHECK(report->growth_error.domain() == vkc::Status::Code::OutOfMemory);
  CHECK(report->dropped > 0 && small->map().tick() == refused_tick + 1);
  auto weights = vr_test::read_attribute<float>(gpu.device, gpu.allocator,
                                                *small, "weight");
  CHECK(weights.ok() &&
        std::any_of(weights->begin(), weights->end(),
                    [](float weight) { return weight > 0.0f; }));

  fail_allocation(3 * sizeof(vkc::Buffer), resize_site);
  report = partial->fuse(*small, pair);
  failure.armed = false;
  CHECK(!failure.injected && report.ok() && report->grows == 0);
  CHECK(report->growth_error.ok());
  while (small->map().tick() - refused_tick < 60) small->map().advance_tick();
  report = partial->fuse(*small, pair);
  CHECK(report.ok() && report->grows == 2);
  CHECK(report->growth_error.ok() && small->grid().num_buckets == 32);

  std::puts("tsdf_fuser_oom: OK");
  return 0;
}

}  // namespace

int main() try {
  return vr_test::run_on_gpu(gpu_main);
} catch (const std::bad_alloc&) {
  vr_test::allocation_failure.armed = false;
  std::fputs("FAIL: bad_alloc escaped fusion or growth\n", stderr);
  return 1;
}
