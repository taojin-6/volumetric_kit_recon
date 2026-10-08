// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// fuse_replica's --dirty-every survey (examples/common/remesh_report.hpp): a
// block stamped changed after the window's tick counts as changed, and puts
// back to marching cubes every block whose +{0,1}^3 neighbourhood holds it
// -- the 2x2x2 at and below it -- and nothing else; a window closes at the
// map's tick, so the next sees only later changes, and an empty map samples
// no window. Exits 0 (skip) where no device is present.

#include <cstdint>
#include <cstdio>
#include <vector>

#include "grid_layout.hpp"
#include "no_device.hpp"
#include "remesh_report.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/instance.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;
namespace vol = volumetric_kit::recon::volume;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

namespace {

// Stamp the block at `coord` changed at the map's tick, as a pass writing
// its voxels would.
vkc::Status touch(vkc::Device& device, vkc::Allocator& allocator,
                  vol::VoxelBlockGrid& grid, vr::Vec3i coord) {
  VKC_ASSIGN(const std::vector<vol::BlockIndex> active,
             grid.map().compact_active_blocks());
  VKC_ASSIGN(std::vector<vol::BlockStamp> stamps,
             grid.map().read_block_stamps());
  for (const vol::BlockIndex& b : active) {
    if (b.coord == coord) {
      stamps[static_cast<std::uint32_t>(b.ptr) / 512u].changed =
          grid.map().tick();
    }
  }
  vkc::CommandBatch batch(device, allocator);
  VKC_TRY(batch.upload(grid.map().stamps_buffer(), 0, stamps.data(),
                       stamps.size() * sizeof(vol::BlockStamp)));
  return batch.submit();
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
  const vol::AttributeSpec attrs[] = {{"tsdf", sizeof(float)},
                                      {"weight", sizeof(float)}};
  auto made = vol::VoxelBlockGrid::create(
      *device, *allocator, vr_example::example_grid_params(0.01f, 0.04f, 256),
      attrs, 2);
  CHECK(made.ok());
  vol::VoxelBlockGrid& grid = made.value();

  // An empty map is no window.
  vr_example::DirtySurvey survey(grid);
  CHECK(survey.sample(grid).ok() && survey.windows() == 0);

  // A 3x3x3 cube of blocks from the origin, then one changed in the middle.
  std::vector<vol::BlockIndex> cube;
  for (int z = 0; z < 3; ++z) {
    for (int y = 0; y < 3; ++y) {
      for (int x = 0; x < 3; ++x) {
        vol::BlockIndex b{};
        b.coord = vr::Vec3i(x, y, z);
        cube.push_back(b);
      }
    }
  }
  CHECK(grid.map().allocate(cube.data(), 27).ok());
  const std::uint32_t before = grid.map().tick();
  grid.map().advance_tick();
  CHECK(touch(*device, *allocator, grid, vr::Vec3i(1, 1, 1)).ok());

  auto active = grid.map().compact_active_blocks();
  CHECK(active.ok() && active.value().size() == 27);
  // Changed after `before`: the one block, and the eight blocks whose cells
  // read its corners -- itself and its seven neighbours toward the origin.
  auto blocks = vr_example::changed_since(grid, active.value(), before);
  CHECK(blocks.ok());
  CHECK(blocks.value().changed == 1 && blocks.value().remesh == 8);
  // A corner block reaches only itself inside the cube.
  grid.map().advance_tick();
  const std::uint32_t middle = grid.map().tick() - 1;
  CHECK(touch(*device, *allocator, grid, vr::Vec3i(0, 0, 0)).ok());
  blocks = vr_example::changed_since(grid, active.value(), middle);
  CHECK(blocks.ok());
  CHECK(blocks.value().changed == 1 && blocks.value().remesh == 1);
  // Nothing changed after the map's tick.
  blocks = vr_example::changed_since(grid, active.value(), grid.map().tick());
  CHECK(blocks.ok() && blocks.value().changed == 0 &&
        blocks.value().remesh == 0);

  // The survey's window opened at construction, before any change, so it
  // sees both; it closes at the map's tick, so the next window sees none.
  CHECK(survey.sample(grid).ok() && survey.windows() == 1);
  CHECK(survey.last().changed == 2 && survey.last().remesh == 8);
  CHECK(survey.sample(grid).ok() && survey.windows() == 2);
  CHECK(survey.last().changed == 0 && survey.last().remesh == 0);
  survey.print(1);
  std::puts("example_remesh_report: OK");
  return 0;
}
