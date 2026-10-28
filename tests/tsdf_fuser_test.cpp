// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Complete fusion is the default; live callers explicitly accept partial
// bands. Growth, grow-ahead, and a bounded failure preserve the expected
// voxel data and integration tick. A live grid holding a block fuses a set in
// one submit to the same voxels, stamps and tick, its new blocks included,
// and grows a set late. The record form (record_fuse, finish) gives what the
// tier calls give from a grid's first set on, a refused set fused again at
// its own tick; a token is the fuser's last record only, and a set the fuse
// refuses records nothing and keeps the last one; and a warm strict
// set is one submit with no staging buffer. Skips when no device is
// available.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <utility>
#include <vector>

#include "test_check.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/instance.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/tsdf/fuser.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"

#include "gpu_test.hpp"
#include "grid_readback.hpp"
#include "sphere_scene.hpp"

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;
namespace vol = volumetric_kit::recon::volume;
namespace tsdf = volumetric_kit::recon::tsdf;

namespace {

constexpr int kCameras = 3;

using View = vr_test::SphereView<vr::DepthCameraParams>;

// The frames' depth, as allocation takes it.
std::vector<vol::DepthInput> depth_inputs(
    const std::vector<tsdf::FrameInput>& frames) {
  std::vector<vol::DepthInput> out;
  for (const tsdf::FrameInput& f : frames) {
    out.push_back({f.depth, tsdf::to_depth_pinhole(f.camera)});
  }
  return out;
}

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

// Whether two grids' blocks carry the same stamps, block by block.
bool same_stamps(vol::VoxelBlockGrid& a, vol::VoxelBlockGrid& b) {
  auto blocks_a = vr_test::blocks_of(a);
  auto blocks_b = vr_test::blocks_of(b);
  auto stamps_a = a.map().read_block_stamps();
  auto stamps_b = b.map().read_block_stamps();
  if (!blocks_a.ok() || !blocks_b.ok() || !stamps_a.ok() || !stamps_b.ok() ||
      blocks_a->size() != blocks_b->size()) {
    return false;
  }
  const auto vpb = std::uint32_t(a.grid().voxels_per_block);
  for (const auto& [coord, ptr] : *blocks_a) {
    const auto other = blocks_b->find(coord);
    if (other == blocks_b->end()) return false;
    const vol::BlockStamp& x = (*stamps_a)[std::uint32_t(ptr) / vpb];
    const vol::BlockStamp& y = (*stamps_b)[std::uint32_t(other->second) / vpb];
    if (x.requested != y.requested || x.weighted != y.weighted ||
        x.changed != y.changed) {
      return false;
    }
  }
  return true;
}

// Whether each block `grid` holds carries the bits `expected` holds there.
bool held_blocks_match(const vr_test::Gpu& gpu, vol::VoxelBlockGrid& expected,
                       vol::VoxelBlockGrid& grid) {
  auto all = vr_test::blocks_of(expected);
  auto held = grid.map().compact_active_blocks();
  if (!all.ok() || !held.ok()) return false;
  vr_test::detail::MatchedBlocks blocks;
  for (const vol::BlockIndex& b : *held) {
    const auto there =
        all->find(vr_test::BlockCoord{b.coord.x, b.coord.y, b.coord.z});
    if (there == all->end()) return false;
    blocks.a.push_back({b.coord, there->second});
    blocks.b.push_back(b);
  }
  for (const char* name : {"weight", "tsdf", "color"}) {
    auto a = vr_test::read_blocks<std::uint32_t>(gpu, expected, name, blocks.a);
    auto b = vr_test::read_blocks<std::uint32_t>(gpu, grid, name, blocks.b);
    if (!a.ok() || !b.ok() ||
        !vr_test::detail::compare_voxels(
            blocks, std::size_t(grid.grid().voxels_per_block),
            [&](std::size_t i) { return a.value()[i] == b.value()[i]; })) {
      return false;
    }
  }
  return true;
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

int gpu_main(vr_test::GpuContext& gpu) {
  vkc::Device& dev = gpu.device;
  vkc::Allocator& alloc = gpu.allocator;
  const vr_test::Gpu ctx{dev, alloc};

  // The set: three cameras, depth and colour on the host.
  std::vector<View> views;
  for (int c = 0; c < kCameras; ++c) {
    views.push_back(vr_test::sphere_view<vr::DepthCameraParams>(c));
  }
  std::vector<tsdf::ColorFrame> colors;
  for (const View& v : views) {
    colors.push_back({v.color.data(),
                      {v.cam.fx, v.cam.fy, v.cam.cx, v.cam.cy, v.cam.width,
                       v.cam.height, v.cam.cam_to_world},
                      {}});
  }
  std::vector<tsdf::FrameInput> set;
  for (int c = 0; c < kCameras; ++c) {
    set.push_back({vkc::StorageInput(views[c].depth.data()), views[c].cam,
                   &colors[std::size_t(c)]});
  }

  // The reference: the tier calls on a grid with room for the set, twice, so
  // the running average and the colour blend run too.
  auto integrator = tsdf::TsdfIntegrator::create(dev, alloc);
  auto reference = make_grid(dev, alloc, params(256));
  CHECK(integrator.ok() && reference.ok());
  const std::vector<vol::DepthInput> depths = depth_inputs(set);
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

  // Default settings complete a band requiring four doublings. The former
  // two-grow default returned success with only 256 of these 604 blocks. The
  // first set's round meets the capacity limit, so its gated fuse runs
  // nothing and finish fuses it again, growing, at the tick it was recorded
  // at: each set takes one.
  {
    auto fuser = make_fuser(dev, alloc);
    auto grid = make_grid(dev, alloc, params(8));
    CHECK(fuser.ok() && grid.ok());
    for (int pass = 0; pass < 2; ++pass) {
      const std::uint32_t tick = grid->map().tick();
      auto report = fuser->fuse(*grid, set);
      CHECK(report.ok() && report->dropped == 0);
      CHECK(report->load_factor == grid->map().load_factor().value());
      CHECK(report->redone == (pass == 0));
      CHECK(grid->map().tick() == tick + 1);
      if (pass == 0) {
        CHECK(report->grows == 4);
        CHECK(report->from_buckets == 8 && report->to_buckets == 128);
      }
    }
    std::size_t observed = 0;
    CHECK(vr_test::same_grids(ctx, *reference, *grid, &observed));
    CHECK(observed > 10000);
  }

  // Grow ahead before a failure, retaining all previously fused voxels.
  {
    const std::int32_t buckets = (blocks + 5) / 6;
    auto fuser = make_fuser(dev, alloc);
    auto grid = make_grid(dev, alloc, params(buckets));
    CHECK(fuser.ok() && grid.ok());
    auto report = fuser->fuse(*grid, set);
    CHECK(report.ok() && report->grows == 0 && report->dropped == 0);
    CHECK(report->load_factor > vol::VoxelHashMap::kGrowThreshold);
    report = fuser->fuse(*grid, set);
    CHECK(report.ok() && report->grows == 1 && report->dropped == 0);
    CHECK(report->from_buckets == buckets && report->to_buckets == 2 * buckets);
    CHECK(vr_test::same_grids(ctx, *reference, *grid));
    const std::uint32_t tick = grid->map().tick();
    report = fuser->fuse(*grid, {});
    CHECK(report.ok() && report->grows == 0 && grid->map().tick() == tick);
  }

  // An incomplete band is an error before integration, which leaves the
  // tick. A later fuser with enough growth allowance can complete the same
  // grid and frame.
  {
    tsdf::FuserConfig config;
    config.max_grows_per_set = 2;
    auto fuser = make_fuser(dev, alloc, config);
    auto grid = make_grid(dev, alloc, params(8));
    CHECK(fuser.ok() && grid.ok());
    const std::uint32_t tick = grid->map().tick();
    auto refused = fuser->fuse(*grid, set);
    CHECK(!refused.ok() &&
          refused.status().domain() == vkc::Status::Code::OutOfMemory);
    CHECK(grid->map().tick() == tick);
    auto any = fused_any(ctx, *grid);
    CHECK(any.ok() && !any.value());
    auto retry = make_fuser(dev, alloc);
    CHECK(retry.ok());
    auto report = retry->fuse(*grid, set);
    CHECK(report.ok() && report->dropped == 0);
    CHECK(retry->fuse(*grid, set).ok());
    CHECK(vr_test::same_grids(ctx, *reference, *grid));
  }

  // A live grid's steady state: once it holds a block (the tier calls fuse
  // the first set on both grids), a fuser, partial or strict, allocates,
  // compacts and fuses a set in one submit, to the voxels, stamps and tick
  // the tier calls give, in either mode.
  for (const bool partial : {true, false}) {
    for (const tsdf::IntegrationMode mode :
         {tsdf::IntegrationMode::Classic, tsdf::IntegrationMode::Dynamic}) {
      tsdf::FuserConfig config;
      config.allow_partial = partial;
      auto fuser = make_fuser(dev, alloc, config);
      auto expected = make_grid(dev, alloc, params(1024));
      auto grid = make_grid(dev, alloc, params(1024));
      CHECK(fuser.ok() && expected.ok() && grid.ok());
      for (int pass = 0; pass < 3; ++pass) {
        auto failed = expected->map().allocate_from_depth(depths);
        CHECK(failed.ok() && failed.value() == 0);
        CHECK(integrator->integrate(*expected, set, 5.0f, mode).ok());
        if (pass == 0) {
          failed = grid->map().allocate_from_depth(depths);
          CHECK(failed.ok() && failed.value() == 0);
          CHECK(integrator->integrate(*grid, set, 5.0f, mode).ok());
          continue;
        }
        auto report = fuser->fuse(*grid, set, 5.0f, mode);
        CHECK(report.ok() && report->dropped == 0 && report->grows == 0);
        CHECK(!report->redone);
      }
      CHECK(vr_test::same_grids(ctx, *expected, *grid));
      CHECK(same_stamps(*expected, *grid));
      CHECK(grid->map().tick() == expected->map().tick());
    }
  }

  // The record form, the set's fusion recorded into a caller's batch, is
  // fuse(): the same voxels, stamps and tick from a grid's first set on, in
  // either mode, one camera or the set, partial or strict. A first set holds
  // no block, so it needs its whole band in either mode: gated, and fused
  // again, at its own tick, should its round lose a race.
  for (const bool partial : {true, false}) {
    for (const tsdf::IntegrationMode mode :
         {tsdf::IntegrationMode::Classic, tsdf::IntegrationMode::Dynamic}) {
      for (const std::size_t cameras : {std::size_t{1}, set.size()}) {
        const std::vector<tsdf::FrameInput> frames(set.begin(),
                                                   set.begin() + cameras);
        const std::vector<vol::DepthInput> band = depth_inputs(frames);
        tsdf::FuserConfig config;
        config.allow_partial = partial;
        auto fuser = make_fuser(dev, alloc, config);
        auto expected = make_grid(dev, alloc, params(1024));
        auto grid = make_grid(dev, alloc, params(1024));
        CHECK(fuser.ok() && expected.ok() && grid.ok());
        for (int pass = 0; pass < 2; ++pass) {
          auto failed = expected->map().allocate_from_depth(band);
          CHECK(failed.ok() && failed.value() == 0);
          CHECK(integrator->integrate(*expected, frames, 5.0f, mode).ok());
          vkc::CommandBatch batch(dev, alloc);
          auto pending = fuser->record_fuse(batch, *grid, frames, 5.0f, mode);
          CHECK(pending.ok());
          CHECK(batch.submit().ok());
          auto report = fuser->finish(*pending, *grid);
          CHECK(report.ok() && report->dropped == 0 && report->grows == 0);
          CHECK(pass == 0 || !report->redone);
        }
        CHECK(vr_test::same_grids(ctx, *expected, *grid));
        CHECK(same_stamps(*expected, *grid));
        CHECK(grid->map().tick() == expected->map().tick());
      }
    }
  }

  // A token is the fuser's last record only: a record supersedes one, which
  // finish then refuses, as it refuses an abandoned one and another grid; an
  // abandoned record's result slot goes back, so records past the map's
  // slots still claim one.
  {
    auto fuser = make_fuser(dev, alloc);
    auto grid = make_grid(dev, alloc, params(1024));
    auto other = make_grid(dev, alloc, params(1024));
    CHECK(fuser.ok() && grid.ok() && other.ok());
    for (std::uint32_t r = 0; r < vol::VoxelHashMap::kResultSlots + 2; ++r) {
      vkc::CommandBatch batch(dev, alloc);
      auto pending = fuser->record_fuse(batch, *grid, set);
      CHECK(pending.ok());
      CHECK(batch.submit().ok());
      fuser->abandon(*pending);
      CHECK(fuser->finish(*pending, *grid).status().domain() ==
            vkc::Status::Code::InvalidArgument);
    }
    vkc::CommandBatch first_batch(dev, alloc);
    auto first = fuser->record_fuse(first_batch, *grid, set);
    CHECK(first.ok() && first_batch.submit().ok());
    vkc::CommandBatch second_batch(dev, alloc);
    auto second = fuser->record_fuse(second_batch, *grid, set);
    CHECK(second.ok() && second_batch.submit().ok());
    // A set the fuse refuses, here for an empty colour frame, records nothing
    // -- no round ahead of the fuse -- and leaves the last record the
    // fuser's.
    {
      std::vector<tsdf::FrameInput> refused = set;
      const tsdf::ColorFrame empty{};
      refused[0].color = &empty;
      vkc::CommandBatch batch(dev, alloc);
      const vkc::DeviceCounters was = dev.counters();
      CHECK(fuser->record_fuse(batch, *grid, refused).status().domain() ==
            vkc::Status::Code::InvalidArgument);
      CHECK(batch.submit().ok());
      CHECK(dev.counters().submits == was.submits);
    }
    CHECK(fuser->finish(*first, *grid).status().domain() ==
          vkc::Status::Code::InvalidArgument);
    CHECK(fuser->finish(*second, *other).status().domain() ==
          vkc::Status::Code::InvalidArgument);
    CHECK(fuser->finish(*second, *grid).ok());
  }

  // A warm set of device frames is one submit, one host wait and no
  // staging buffer, strict as the senders fuse.
  {
    std::vector<vkc::Buffer> depth_buffers;
    std::vector<vkc::Buffer> color_buffers;
    std::vector<tsdf::ColorFrame> device_colors = colors;
    for (int c = 0; c < kCameras; ++c) {
      auto depth =
          vr_test::upload_device_buffer(dev, alloc, views[c].depth.data(),
                                        views[c].depth.size() * sizeof(float));
      auto color = vr_test::upload_device_buffer(
          dev, alloc, views[c].color.data(),
          views[c].color.size() * sizeof(std::uint32_t));
      CHECK(depth.ok() && color.ok());
      depth_buffers.push_back(std::move(depth).value());
      color_buffers.push_back(std::move(color).value());
    }
    std::vector<tsdf::FrameInput> device_set;
    for (int c = 0; c < kCameras; ++c) {
      device_colors[std::size_t(c)].pixels = nullptr;
      device_colors[std::size_t(c)].buffer = &color_buffers[std::size_t(c)];
      device_set.push_back({vkc::StorageInput(depth_buffers[std::size_t(c)]),
                            views[c].cam, &device_colors[std::size_t(c)]});
    }
    auto fuser = make_fuser(dev, alloc);
    auto grid = make_grid(dev, alloc, params(1024));
    CHECK(fuser.ok() && grid.ok());
    CHECK(fuser->fuse(*grid, device_set).ok());
    CHECK(fuser->fuse(*grid, device_set).ok());
    const vkc::DeviceCounters was = dev.counters();
    auto report = fuser->fuse(*grid, device_set);
    const vkc::DeviceCounters now = dev.counters();
    CHECK(report.ok() && !report->redone);
    CHECK(now.submits - was.submits == 1);
    CHECK(now.host_waits - was.host_waits <= 1);
    CHECK(now.staging_buffers == was.staging_buffers);
  }

  // There, a set's new blocks are fused in its own submit: once a grid holds
  // set[0]'s band, each block it holds after the whole set carries the
  // voxels the tier calls give. A block a lock race left out is not held.
  {
    tsdf::FuserConfig config;
    config.allow_partial = true;
    auto fuser = make_fuser(dev, alloc, config);
    auto expected = make_grid(dev, alloc, params(1024));
    auto grid = make_grid(dev, alloc, params(1024));
    CHECK(fuser.ok() && expected.ok() && grid.ok());
    std::vector<tsdf::FrameInput> first{set[0]};
    std::uint32_t first_blocks = 0;
    for (const std::vector<tsdf::FrameInput>* frames : {&first, &set}) {
      const std::vector<vol::DepthInput> band = depth_inputs(*frames);
      auto failed = expected->map().allocate_from_depth(band);
      CHECK(failed.ok() && failed.value() == 0);
      CHECK(integrator->integrate(*expected, *frames).ok());
      auto report = fuser->fuse(*grid, *frames);
      CHECK(report.ok() && report->grows == 0);
      if (frames == &first) first_blocks = grid->map().allocated_blocks();
    }
    std::printf("  one submit allocated %u of the set's blocks after %u\n",
                grid->map().allocated_blocks() - first_blocks, first_blocks);
    CHECK(grid->map().allocated_blocks() > first_blocks + 100);
    CHECK(held_blocks_match(ctx, *expected, *grid));
  }

  // There, a capacity limit is grown for the next set, after this one: the set
  // fuses what its round allocated, and the next completes the band, capacity
  // no longer limiting it. A lock race in its one round lands a set later.
  {
    tsdf::FuserConfig config;
    config.allow_partial = true;
    auto fuser = make_fuser(dev, alloc, config);
    auto grid = make_grid(dev, alloc, params(64));
    CHECK(fuser.ok() && grid.ok());
    auto first = fuser->fuse(*grid, {set[0]});
    CHECK(first.ok() && first->dropped == 0 && first->grows == 0);
    CHECK(first->load_factor <= vol::VoxelHashMap::kGrowThreshold);
    auto late = fuser->fuse(*grid, set);
    CHECK(late.ok() && late->dropped > 0 && late->failures.capacity_limited());
    CHECK(late->grows == 1 && late->to_buckets == 128);
    auto next = fuser->fuse(*grid, set);
    CHECK(next.ok() && next->grows == 0 && !next->failures.capacity_limited());
    std::uint32_t dropped = next->dropped;
    for (int later = 0; dropped != 0 && later < 4; ++later) {
      auto again = fuser->fuse(*grid, set);
      CHECK(again.ok() && again->grows == 0);
      dropped = again->dropped;
    }
    CHECK(dropped == 0);
  }

  // Live callers opt in: each bound includes the next set's grow-ahead,
  // still integrates what fits, and reports the allocation failures.
  for (const int bound : {0, 1, 2}) {
    tsdf::FuserConfig config;
    config.max_grows_per_set = bound;
    config.allow_partial = true;
    auto fuser = make_fuser(dev, alloc, config);
    auto grid = make_grid(dev, alloc, params(1));
    CHECK(fuser.ok() && grid.ok());
    for (int pass = 1; pass <= 2; ++pass) {
      auto report = fuser->fuse(*grid, set);
      CHECK(report.ok() && report->grows == bound);
      CHECK(report->dropped > 0 && report->failures.capacity_limited());
      CHECK(grid->grid().num_buckets == 1 << (bound * pass));
      auto any = fused_any(ctx, *grid);
      CHECK(any.ok() && any.value());
    }
    auto invalid = set;
    invalid[0].depth = vkc::StorageInput(static_cast<const float*>(nullptr));
    CHECK(fuser->fuse(*grid, invalid).status().domain() ==
          vkc::Status::Code::InvalidArgument);
  }

  // Reject invalid configuration and moved-from objects.
  {
    tsdf::FuserConfig bad;
    bad.max_grows_per_set = -1;
    CHECK(make_fuser(dev, alloc, bad).status().domain() ==
          vkc::Status::Code::InvalidArgument);
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

int main() { return vr_test::run_on_gpu(gpu_main); }
