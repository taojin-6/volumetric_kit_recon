// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// tsdf::Fuser on three cameras round a sphere. A set fused through it into a
// grid too small to hold it, growing on the way, matches the tier calls --
// allocate every band, then integrate -- on a grid that never needed to grow,
// bit for bit. It grows ahead of need, before any allocation fails. A grow
// the headroom declines (a reading at the limit included), the policy
// forbids, or the per-set bound (the grow ahead included) stops leaves blocks
// out but still fuses the set, and so does a resize that runs out of memory,
// which is asked again after retry_after sets rather than never. Past
// refuse_allocation_above a set allocates nothing new; with growth on, a
// value below kGrowThreshold is refused. Exits 0 (skip) where no device is
// present.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <optional>
#include <set>
#include <utility>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/instance.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/tsdf/fuser.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/grid_growth.hpp"
#include "volumetric_kit/recon/volume/hash.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"

#include "grid_readback.hpp"
#include "no_device.hpp"
#include "sphere_scene.hpp"

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;
namespace vol = volumetric_kit::recon::volume;
namespace tsdf = volumetric_kit::recon::tsdf;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

namespace {

using vol::GrowthOutcome;

constexpr int kCameras = 3;

vol::VoxelGridParams params(std::int32_t num_buckets) {
  vol::VoxelGridParams grid{};
  grid.voxel_size = 0.01f;
  grid.block_size = 8;
  grid.voxels_per_block = 512;
  grid.trunc_dist = 0.04f;
  grid.bucket_size = 8;
  grid.num_buckets = num_buckets;
  grid.num_blocks = 8 * num_buckets;
  grid.max_chain = 128;
  return grid;
}

vkc::Result<vol::VoxelBlockGrid> make_grid(vkc::Device& device,
                                           vkc::Allocator& allocator,
                                           const vol::VoxelGridParams& grid) {
  const vol::AttributeSpec attrs[] = {{"tsdf", sizeof(float)},
                                      {"weight", sizeof(float)},
                                      {"color", sizeof(std::uint32_t)}};
  return vol::VoxelBlockGrid::create(device, allocator, grid, attrs, 3);
}

vkc::Result<tsdf::Fuser> make_fuser(vkc::Device& device,
                                    vkc::Allocator& allocator,
                                    tsdf::FuserConfig config = {}) {
  return tsdf::Fuser::create(device, allocator, std::move(config));
}

// Whether any voxel of `grid` holds weight: the set was integrated.
vkc::Result<bool> fused_any(const vr_test::Gpu& gpu,
                            const vol::VoxelBlockGrid& grid) {
  VKC_ASSIGN(const std::vector<float> weight,
             vr_test::read_attribute<float>(gpu.device, gpu.allocator, grid,
                                            "weight"));
  for (const float w : weight) {
    if (w > 0.0f) return true;
  }
  return false;
}

}  // namespace

int main() {
  auto instance = vkc::Instance::create({});
  if (!instance) {
    return vr_test::no_device("no Vulkan instance",
                              instance.status().message());
  }
  auto gpu = instance->select_physical_device(vr::device_requirements());
  if (!gpu) {
    return vr_test::no_device("no compute-capable device",
                              gpu.status().message());
  }
  auto device = vkc::Device::create(*instance, *gpu, vr::device_requirements());
  CHECK(device.ok());
  auto allocator = vkc::Allocator::create(instance->handle(), *device);
  CHECK(allocator.ok());
  vkc::Device& dev = *device;
  vkc::Allocator& alloc = *allocator;
  const vr_test::Gpu ctx{dev, alloc};

  // The set: three cameras, depth and colour on the host.
  std::vector<vr_test::SphereView> views;
  for (int c = 0; c < kCameras; ++c) views.push_back(vr_test::sphere_view(c));
  std::vector<tsdf::ColorFrame> colors;
  for (const vr_test::SphereView& v : views) {
    colors.push_back({v.color.data(),
                      {v.cam.fx, v.cam.fy, v.cam.cx, v.cam.cy, v.cam.width,
                       v.cam.height, v.cam.cam_to_world},
                      {}});
  }
  std::vector<tsdf::FrameInput> set;
  for (int c = 0; c < kCameras; ++c) {
    set.push_back({{vkc::StorageInput(views[c].depth.data()), views[c].cam},
                   &colors[std::size_t(c)]});
  }
  const std::vector<tsdf::FrameInput> first{set[0]};
  const std::vector<tsdf::FrameInput> second{set[1]};

  // The reference: the tier calls on a grid with room for the set, twice, so
  // the running average and the colour blend run too.
  auto integrator = tsdf::TsdfIntegrator::create(dev, alloc);
  auto reference = make_grid(dev, alloc, params(1024));
  CHECK(integrator.ok() && reference.ok());
  const std::vector<vol::DepthInput> depths(set.begin(), set.end());
  for (int pass = 0; pass < 2; ++pass) {
    auto failed = reference->map().allocate_from_depth(depths);
    CHECK(failed.ok() && failed.value() == 0);
    CHECK(integrator->integrate(*reference, set).ok());
  }
  auto reference_blocks = vr_test::blocks_of(*reference);
  CHECK(reference_blocks.ok());
  const auto blocks = std::int32_t(reference_blocks->size());
  std::printf("  the set allocates %d blocks\n", blocks);
  CHECK(blocks > 256);

  // Through the fuser, from two buckets: the first set grows for capacity
  // until it fits, the second ahead of need; neither leaves a block out, and
  // the grid holds what the reference does, bit for bit.
  {
    tsdf::FuserConfig config;
    config.max_grows_per_set = 16;
    auto fuser = make_fuser(dev, alloc, config);
    auto grid = make_grid(dev, alloc, params(2));
    CHECK(fuser.ok() && grid.ok());
    int grows = 0;
    for (int pass = 0; pass < 2; ++pass) {
      auto report = fuser->fuse(*grid, set);
      CHECK(report.ok());
      CHECK(report->dropped == 0 && !report->allocation_refused);
      CHECK(report->load_factor == grid->map().load_factor().value());
      grows += report->grows;
      if (pass == 0) CHECK(report->grow.grew());
    }
    std::printf("  fuser grew 2 -> %d buckets in %d grows\n",
                grid->grid().num_buckets, grows);
    CHECK(grows >= 2 && grid->grid().num_buckets > 2);
    std::size_t observed = 0;
    CHECK(vr_test::same_grids(ctx, *reference, *grid, &observed));
    CHECK(observed > 10000);
  }

  // Ahead of need: a grid the set fills past kGrowThreshold without a
  // failure grows before the next set allocates, which then fails nothing.
  {
    const std::int32_t buckets = (blocks + 5) / 6;  // about 0.75 full
    auto fuser = make_fuser(dev, alloc);
    auto grid = make_grid(dev, alloc, params(buckets));
    CHECK(fuser.ok() && grid.ok());
    auto report = fuser->fuse(*grid, set);
    CHECK(report.ok());
    CHECK(report->grows == 0 && report->dropped == 0);
    CHECK(report->failures.total == 0);
    CHECK(report->load_factor > vol::VoxelHashMap::kGrowThreshold);
    report = fuser->fuse(*grid, set);
    CHECK(report.ok());
    CHECK(report->ahead.grew() && report->grows == 1);
    CHECK(report->ahead.from_buckets == buckets);
    CHECK(grid->grid().num_buckets == 2 * buckets);
    CHECK(report->dropped == 0 && report->failures.total == 0);
    CHECK(report->grow.outcome == GrowthOutcome::NotDue);
  }

  // A headroom reading at the limit declines the grow the set needs; the set
  // fuses what fits. The refusal holds without asking again until
  // retry_after sets have passed, by either route.
  {
    int asked = 0;
    tsdf::FuserConfig config;
    config.growth.retry_after = 2;
    config.growth.headroom = [&]() -> std::optional<std::uint64_t> {
      ++asked;
      return 0;
    };
    auto fuser = make_fuser(dev, alloc, config);
    auto grid = make_grid(dev, alloc, params(8));
    CHECK(fuser.ok() && grid.ok());
    const std::uint32_t tick = grid->map().tick();
    auto report = fuser->fuse(*grid, set);
    CHECK(report.ok());
    CHECK(report->grow.outcome == GrowthOutcome::DeclinedForMemory);
    CHECK(report->grow.needed_bytes == grid->bytes_at(16));
    CHECK(report->dropped > 0 && report->failures.capacity_limited());
    CHECK(report->grows == 0 && grid->grid().num_buckets == 8);
    CHECK(grid->map().tick() == tick + 1);
    auto any = fused_any(ctx, *grid);
    CHECK(any.ok() && any.value());
    CHECK(asked == 1);
    report = fuser->fuse(*grid, set);
    CHECK(report.ok());
    CHECK(report->ahead.outcome == GrowthOutcome::Waiting);
    CHECK(report->grow.outcome == GrowthOutcome::Waiting);
    CHECK(asked == 1);
    report = fuser->fuse(*grid, set);
    CHECK(report.ok());
    CHECK(report->ahead.outcome == GrowthOutcome::DeclinedForMemory);
    CHECK(asked == 2);
  }

  // Growth off: the set fuses into what fits and reports the rest.
  {
    tsdf::FuserConfig config;
    config.growth.enabled = false;
    auto fuser = make_fuser(dev, alloc, config);
    auto grid = make_grid(dev, alloc, params(8));
    CHECK(fuser.ok() && grid.ok());
    auto report = fuser->fuse(*grid, set);
    CHECK(report.ok());
    CHECK(report->grow.outcome == GrowthOutcome::Disabled);
    CHECK(report->grows == 0 && grid->grid().num_buckets == 8);
    CHECK(report->dropped > 0 && report->failures.capacity_limited());
    auto any = fused_any(ctx, *grid);
    CHECK(any.ok() && any.value());
  }

  // The per-set bound: one grow, then the set fuses what fits; none, and the
  // set never asks. The grow ahead counts against it: the second set, on a
  // full grid, grows ahead once under a bound of one and not at all under
  // zero, and asks for no grow for its capacity limit.
  for (const int bound : {0, 1}) {
    tsdf::FuserConfig config;
    config.max_grows_per_set = bound;
    auto fuser = make_fuser(dev, alloc, config);
    auto grid = make_grid(dev, alloc, params(1));
    CHECK(fuser.ok() && grid.ok());
    auto report = fuser->fuse(*grid, set);
    CHECK(report.ok());
    CHECK(report->grows == bound);
    CHECK(grid->grid().num_buckets == 1 << bound);
    CHECK(report->grow.outcome ==
          (bound == 0 ? GrowthOutcome::NotDue : GrowthOutcome::Grew));
    CHECK(report->dropped > 0 && report->failures.capacity_limited());
    CHECK(report->load_factor > vol::VoxelHashMap::kGrowThreshold);
    report = fuser->fuse(*grid, set);
    CHECK(report.ok());
    CHECK(report->grows == bound);
    CHECK(report->ahead.outcome ==
          (bound == 0 ? GrowthOutcome::NotDue : GrowthOutcome::Grew));
    CHECK(report->grow.outcome == GrowthOutcome::NotDue);
    CHECK(grid->grid().num_buckets == 1 << (2 * bound));
    CHECK(report->dropped > 0 && report->failures.capacity_limited());
  }

  // Past refuse_allocation_above, a set allocates nothing new and still
  // fuses into the blocks there.
  {
    tsdf::FuserConfig config;
    config.growth.enabled = false;
    config.refuse_allocation_above = 0.5f;
    auto fuser = make_fuser(dev, alloc, config);
    auto grid = make_grid(dev, alloc, params(8));
    CHECK(fuser.ok() && grid.ok());
    auto report = fuser->fuse(*grid, first);
    CHECK(report.ok() && !report->allocation_refused);
    CHECK(report->load_factor > 0.5f);
    const auto before = vr_test::blocks_of(*grid);
    const std::uint32_t tick = grid->map().tick();
    report = fuser->fuse(*grid, second);
    CHECK(report.ok() && report->allocation_refused);
    CHECK(report->dropped == 0);
    const auto after = vr_test::blocks_of(*grid);
    CHECK(before.ok() && after.ok() && before.value() == after.value());
    CHECK(grid->map().tick() == tick + 1);
  }

  // A resize that runs out of memory: the set still fuses, and the grow is
  // asked again after retry_after sets. Five blocks in five buckets of two
  // all land in one bucket of nine, where two slots and a chain of two hold
  // four, so growing 8 -> 9 cannot rehash them.
  {
    vol::VoxelGridParams tight = params(8);
    tight.bucket_size = 2;
    tight.num_blocks = 16;
    tight.max_chain = 2;
    auto grid = make_grid(dev, alloc, tight);
    CHECK(grid.ok());
    std::vector<vol::BlockIndex> clash;
    std::set<std::uint32_t> old_buckets;
    for (int i = 0; i < 4096 && clash.size() < 5; ++i) {
      const vr::Vec3i c(i % 64, i / 64, 100);
      if (vol::hash_bucket(c, 9) == 0 &&
          old_buckets.insert(vol::hash_bucket(c, 8)).second) {
        vol::BlockIndex block{};
        block.coord = c;
        clash.push_back(block);
      }
    }
    CHECK(clash.size() == 5);
    auto placed = grid->map().allocate(clash.data(), 5);
    CHECK(placed.ok() && placed.value() == 0);
    tsdf::FuserConfig config;
    config.growth.max_buckets = 9;
    config.growth.retry_after = 2;
    auto fuser = make_fuser(dev, alloc, config);
    CHECK(fuser.ok());
    const std::uint32_t tick = grid->map().tick();
    auto report = fuser->fuse(*grid, set);
    CHECK(report.ok());
    CHECK(report->grow.outcome == GrowthOutcome::ResizeFailed);
    CHECK(report->grow.error.domain() == vkc::Status::Code::OutOfMemory);
    CHECK(report->dropped > 0 && grid->grid().num_buckets == 8);
    CHECK(grid->map().tick() == tick + 1);
    auto any = fused_any(ctx, *grid);
    CHECK(any.ok() && any.value());
    report = fuser->fuse(*grid, set);
    CHECK(report.ok() && report->ahead.outcome == GrowthOutcome::Waiting);
    report = fuser->fuse(*grid, set);
    CHECK(report.ok() && report->ahead.outcome == GrowthOutcome::ResizeFailed);
  }

  // A configuration that cannot work is refused; a moved-from fuser refuses
  // to fuse.
  {
    tsdf::FuserConfig bad;
    bad.max_grows_per_set = -1;
    CHECK(make_fuser(dev, alloc, bad).status().domain() ==
          vkc::Status::Code::InvalidArgument);
    for (const float above :
         {0.0f, 1.5f, std::numeric_limits<float>::quiet_NaN()}) {
      bad = {};
      bad.refuse_allocation_above = above;
      CHECK(make_fuser(dev, alloc, bad).status().domain() ==
            vkc::Status::Code::InvalidArgument);
    }
    bad = {};
    bad.growth.max_buckets = -1;
    CHECK(make_fuser(dev, alloc, bad).status().domain() ==
          vkc::Status::Code::InvalidArgument);
    // A stop below the grow threshold would refuse allocation before a grow
    // ahead is due, for good; at it, the grow ahead runs first.
    bad = {};
    bad.refuse_allocation_above = 0.6f;
    CHECK(make_fuser(dev, alloc, bad).status().domain() ==
          vkc::Status::Code::InvalidArgument);
    bad.refuse_allocation_above = vol::VoxelHashMap::kGrowThreshold;
    CHECK(make_fuser(dev, alloc, bad).ok());

    auto fuser = make_fuser(dev, alloc);
    auto grid = make_grid(dev, alloc, params(1024));
    CHECK(fuser.ok() && grid.ok());
    tsdf::Fuser moved(std::move(fuser).value());
    CHECK(!fuser->valid() && moved.valid());
    CHECK(fuser->fuse(*grid, set).status().domain() ==
          vkc::Status::Code::InvalidArgument);
    CHECK(moved.fuse(*grid, set).ok());
  }

  std::puts("tsdf_fuser: OK");
  return 0;
}
