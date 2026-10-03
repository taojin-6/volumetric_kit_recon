// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <set>
#include <utility>
#include <vector>

#include "buffer_readback.hpp"
#include "volumetric_kit/recon/core/compute_util.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/log.hpp"
#include "volumetric_kit/recon/volume/hierarchical_grid.hpp"

namespace vr = volumetric_kit::recon;
namespace vol = volumetric_kit::recon::volume;
#define CHECK(c)                                                        \
  do {                                                                  \
    if (!(c)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); \
      return 1;                                                         \
    }                                                                   \
  } while (0)

std::atomic<int> validation_errors{0};

int main() {
  vr::set_log_handler([](vr::LogLevel level, std::string_view message) {
    if (level == vr::LogLevel::Error) {
      ++validation_errors;
      std::fprintf(stderr, "%.*s\n", static_cast<int>(message.size()),
                   message.data());
    }
  });
  vr::InstanceConfig instance_config;
  instance_config.enable_validation = true;
  auto instance = vr::Instance::create(instance_config);
  if (!instance) {
    std::printf("no Vulkan instance; skipping\n");
    return 0;
  }
  auto physical = instance->select_physical_device();
  if (!physical) {
    std::printf("no Vulkan device; skipping\n");
    return 0;
  }
  auto device = vr::Device::create(instance.value(), physical.value(), {});
  CHECK(device.ok());
  auto allocator = vr::Allocator::create(instance->handle(), device.value());
  CHECK(allocator.ok());
  auto& dev = device.value();
  auto& alloc = allocator.value();
  vol::HierarchicalGridConfig config;
  config.finest.voxel_size = 0.03125f;
  config.finest.trunc_dist = 1.0f;  // exactly the root extent at level 2
  config.finest.bucket_size = 8;
  config.finest.num_buckets = 16;
  config.finest.num_blocks = 128;
  config.level_count = 3;
  config.child_block_capacity = 16;
  config.color = true;
  auto created = vol::HierarchicalGrid::create(dev, alloc, config);
  CHECK(created.ok());
  auto grid = std::move(created).value();
  CHECK(grid.valid() && grid.node_capacity() == 144);
  auto empty = grid.prepare_leaves();
  CHECK(empty.ok() && empty->validate().ok() && empty->leaf_count == 0);
  CHECK(empty->nodes->mapped() == nullptr && empty->tsdf->mapped() == nullptr);

  const vol::BlockIndex roots[] = {{vr::Vec3i(-1, 0, 0), -1},
                                   {vr::Vec3i(0, 0, 0), -1},
                                   {vr::Vec3i(1, 0, 0), -1}};
  auto allocated = grid.allocate_roots(roots, 3);
  CHECK(allocated.ok() && allocated.value() == 0);
  CHECK(!empty->is_current());
  auto before = grid.prepare_leaves();
  CHECK(before.ok() && before->validate().ok() && before->leaf_count == 3);
  CHECK(before->root_grid.trunc_dist == config.finest.trunc_dist);
  auto cached = grid.prepare_leaves();
  CHECK(cached.ok() && cached->generation == before->generation);
  auto nodes_before = vr_test::read_back<vol::HierarchicalNode>(
      dev, alloc, *before->nodes, grid.node_capacity());
  auto leaves_before = vr_test::read_back<std::uint32_t>(
      dev, alloc, *before->leaf_indices, before->leaf_count);
  CHECK(nodes_before.ok() && leaves_before.ok());
  std::set<int> xs;
  for (auto id : leaves_before.value()) {
    const auto& n = nodes_before.value()[id];
    CHECK(n.ptr == static_cast<std::int32_t>(id * 512u));
    CHECK(n.level == 2 && n.children == 0);
    CHECK(n.coord.y == 0 && n.coord.z == 0);
    xs.insert(n.coord.x);
  }
  CHECK(xs ==
        std::set<int>({-1, 0, 1}));  // device ABI, including signed coords

  // Distinct parent samples reveal incorrect octants, local addressing, or a
  // color-copy stride. The transfer must cap confidence without changing TSDF.
  const auto samples = static_cast<std::size_t>(grid.node_capacity()) * 512u;
  std::vector<float> tsdf(samples, 0.0f), weight(samples, 0.0f);
  std::vector<std::uint32_t> color(samples, 0u);
  for (auto id : leaves_before.value()) {
    for (std::uint32_t local = 0; local < 512; ++local) {
      const auto i = static_cast<std::size_t>(id) * 512u + local;
      tsdf[i] = static_cast<float>(local) * 0.001f;
      weight[i] = 5.0f;
      color[i] = 0xff000000u + local;
    }
  }
  CHECK(vr_test::write_back(dev, alloc, *before->tsdf, tsdf));
  CHECK(vr_test::write_back(dev, alloc, *before->weight, weight));
  CHECK(vr_test::write_back(dev, alloc, *before->color, color));
  auto requests = vr::device_storage_buffer(
      alloc, grid.node_capacity() * sizeof(std::uint32_t));
  CHECK(requests.ok());
  std::vector<std::uint32_t> desired(grid.node_capacity(),
                                     std::numeric_limits<std::uint32_t>::max());
  for (auto id : leaves_before.value()) desired[id] = 0;
  CHECK(vr_test::write_back(dev, alloc, requests.value(), desired));
  auto split = grid.split(requests.value(), 1);
  CHECK(split.ok() && split->split == 1 && split->deferred == 2 &&
        split->exhausted == 0 && split->rejected == 0);
  CHECK(!before->is_current());
  auto after = grid.prepare_leaves();
  CHECK(after.ok() && after->validate().ok() && after->leaf_count == 10);
  auto nodes_after = vr_test::read_back<vol::HierarchicalNode>(
      dev, alloc, *after->nodes, grid.node_capacity());
  auto tsdf_after =
      vr_test::read_back<float>(dev, alloc, *after->tsdf, samples);
  auto weight_after =
      vr_test::read_back<float>(dev, alloc, *after->weight, samples);
  auto color_after =
      vr_test::read_back<std::uint32_t>(dev, alloc, *after->color, samples);
  CHECK(nodes_after.ok() && tsdf_after.ok() && weight_after.ok() &&
        color_after.ok());
  std::uint32_t changed = 0;
  for (auto id : leaves_before.value()) {
    const auto& parent = nodes_after.value()[id];
    if (parent.children == 0) continue;
    ++changed;
    for (std::uint32_t oct = 0; oct < 8; ++oct) {
      const auto& child = nodes_after.value()[parent.children - 1 + oct];
      CHECK(child.level == 1 && child.children == 0);
      CHECK(child.coord ==
            parent.coord * 2 +
                vr::Vec3i(int(oct & 1u), int((oct >> 1) & 1u), int(oct >> 2)));
      CHECK(child.ptr ==
            static_cast<std::int32_t>((parent.children - 1 + oct) * 512u));
      for (std::uint32_t local = 0; local < 512; ++local) {
        const auto x = ((local & 7u) + (oct & 1u) * 8u) / 2u;
        const auto y = (((local >> 3) & 7u) + ((oct >> 1) & 1u) * 8u) / 2u;
        const auto z = ((local >> 6) + (oct >> 2) * 8u) / 2u;
        const auto src =
            static_cast<std::size_t>(parent.ptr) + x + 8u * (y + 8u * z);
        const auto dst = static_cast<std::size_t>(child.ptr) + local;
        CHECK(tsdf_after.value()[dst] == tsdf[src]);
        CHECK(weight_after.value()[dst] == 1.0f);
        CHECK(color_after.value()[dst] == color[src]);
      }
    }
  }
  CHECK(changed == 1);
  // Fill the second group. One of the two remaining requested roots survives
  // unchanged under exhaustion; retry cannot leak another partial group.
  split = grid.split(requests.value(), 10);
  CHECK(split.ok() && split->split == 1 && split->exhausted == 1);
  split = grid.split(requests.value(), 10);
  CHECK(split.ok() && split->split == 0 && split->exhausted == 1);
  after = grid.prepare_leaves();
  CHECK(after.ok() && after->leaf_count == 17);
  CHECK(!grid.split(requests.value(), 1, 0.0f));
  CHECK(!grid.split(vr::Buffer{}, 1));

  // Moves invalidate borrowed generations and empty the source; assignment
  // over a live grid releases that grid, while self-move preserves this one.
  vol::HierarchicalGrid moved(std::move(grid));
  CHECK(!grid.valid() && grid.node_capacity() == 0);
  CHECK(!after->is_current());
  CHECK(moved.valid() && moved.node_capacity() == 144);
  CHECK(!grid.prepare_leaves());
  auto another = vol::HierarchicalGrid::create(dev, alloc, config);
  CHECK(another.ok());
  auto destination = std::move(another).value();
  destination = std::move(moved);
  CHECK(destination.valid() && !moved.valid() && moved.node_capacity() == 0);
  auto* self = &destination;
  destination = std::move(*self);
  CHECK(destination.valid());
  CHECK(destination.clear());
  after = destination.prepare_leaves();
  CHECK(after.ok() && after->leaf_count == 0);
  auto zeroed = vr_test::read_back<float>(dev, alloc, *after->weight, samples);
  CHECK(zeroed.ok() && std::all_of(zeroed->begin(), zeroed->end(),
                                   [](float w) { return w == 0.0f; }));

  // A surface just before x=1 selects nearest root 1. With T=1 the root -1
  // can still contain fine samples within the band. Allocation-only padding
  // must include it; the actual TSDF truncation remains exactly 1 metre.
  config.finest.num_buckets = 64;
  config.finest.num_blocks = 512;
  auto band_grid = vol::HierarchicalGrid::create(dev, alloc, config);
  CHECK(band_grid.ok());
  destination = std::move(band_grid).value();
  const float depth = 1.0f;
  vr::DepthCameraParams camera{};
  camera.fx = camera.fy = 1.0f;
  camera.min_depth = 0.1f;
  camera.max_depth = 3.0f;
  camera.width = camera.height = 1;
  camera.cam_to_world = vr::Mat4f(1.0f);
  camera.cam_to_world[3][0] = 0.96875f;
  const std::vector<vol::DepthInput> frames{{vr::StorageInput(&depth), camera}};
  allocated = destination.allocate_from_depth(frames);
  CHECK(allocated.ok() && allocated.value() == 0);
  after = destination.prepare_leaves();
  CHECK(after.ok() && after->root_grid.trunc_dist == 1.0f);
  nodes_after = vr_test::read_back<vol::HierarchicalNode>(
      dev, alloc, *after->nodes, destination.node_capacity());
  CHECK(nodes_after.ok());
  bool left_band = false;
  for (const auto& n : nodes_after.value()) {
    if (n.ptr >= 0 && n.level == 2 && n.coord == vr::Vec3i(-1, 0, 1))
      left_band = true;
  }
  CHECK(left_band);

  config.child_block_capacity = 7;
  CHECK(!vol::HierarchicalGrid::create(dev, alloc, config));
  config.child_block_capacity = 16;
  config.level_count = 5;
  CHECK(!vol::HierarchicalGrid::create(dev, alloc, config));
  CHECK(validation_errors == 0);
  std::printf("recon hierarchical grid test passed\n");
  return 0;
}
