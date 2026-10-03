// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <set>
#include <utility>
#include <vector>

#include "buffer_readback.hpp"
#include "volumetric_kit/recon/core/color_space.hpp"
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

int check_prolongation_support(vr::Device& dev, vr::Allocator& alloc) {
  vol::HierarchicalGridConfig config;
  config.finest.voxel_size = 0.005f;
  config.finest.trunc_dist = 0.4f;
  config.finest.bucket_size = 8;
  config.finest.num_buckets = 4;
  config.finest.num_blocks = 32;
  config.level_count = 2;
  config.child_block_capacity = 8;
  config.color = true;
  auto made = vol::HierarchicalGrid::create(dev, alloc, config);
  CHECK(made);
  auto grid = std::move(made).value();
  const vol::BlockIndex root{vr::Vec3i(-1, -1, -1), -1};
  auto allocated = grid.allocate_roots(&root, 1);
  CHECK(allocated && allocated.value() == 0);
  auto field = grid.prepare_leaves();
  CHECK(field);
  auto ids =
      vr_test::read_back<std::uint32_t>(dev, alloc, *field->leaf_indices, 1);
  CHECK(ids);
  const auto parent = ids->front();
  const auto offset = VkDeviceSize(parent) * 512 * sizeof(float);
  std::vector<float> sdf(512), weights(512, 5.0f);
  std::vector<std::uint32_t> colors(512);
  for (unsigned i = 0; i < 512; ++i) {
    sdf[i] = (float(i) - 255.5f) * 0.001f;
    colors[i] = (i & 1u) != 0 ? 0xff0000ffu : 0xffff0000u;
  }
  weights[3 + 8 * (3 + 8 * 3)] = 0.0f;
  colors[3 + 8 * (3 + 8 * 3)] = 0u;
  weights[5 + 8 * (5 + 8 * 5)] = 0.25f;
  colors[1 + 8 * (1 + 8 * 1)] = 0u;
  vr::CommandBatch batch(dev, alloc);
  CHECK(batch.upload(*field->tsdf, offset, sdf.data(), 512 * sizeof(float)));
  CHECK(batch.upload(*field->weight, offset, weights.data(),
                     512 * sizeof(float)));
  CHECK(batch.upload(*field->color, offset, colors.data(),
                     512 * sizeof(std::uint32_t)));
  CHECK(batch.submit());
  auto requests = vr::device_storage_buffer(
      alloc, grid.node_capacity() * sizeof(std::uint32_t));
  CHECK(requests);
  std::vector<std::uint32_t> desired(grid.node_capacity(),
                                     std::numeric_limits<std::uint32_t>::max());
  desired[parent] = 0;
  CHECK(vr_test::write_back(dev, alloc, requests.value(), desired));
  auto split = grid.split(requests.value(), 1, 0.5f);
  CHECK(split && split->split == 1);
  field = grid.prepare_leaves();
  CHECK(field && field->leaf_count == 8);
  auto nodes = vr_test::read_back<vol::HierarchicalNode>(
      dev, alloc, *field->nodes, grid.node_capacity());
  const auto sample_count = std::size_t(grid.node_capacity()) * 512;
  auto out_sdf =
      vr_test::read_back<float>(dev, alloc, *field->tsdf, sample_count);
  auto out_weights =
      vr_test::read_back<float>(dev, alloc, *field->weight, sample_count);
  auto out_colors = vr_test::read_back<std::uint32_t>(dev, alloc, *field->color,
                                                      sample_count);
  CHECK(nodes && out_sdf && out_weights && out_colors);
  const auto at = [&](unsigned x, unsigned y, unsigned z) {
    const auto octant = x / 8 + 2 * (y / 8) + 4 * (z / 8);
    const auto child = nodes.value()[parent].children - 1 + octant;
    return std::size_t(child) * 512 + x % 8 + 8 * (y % 8 + 8 * (z % 8));
  };
  // An unobserved nearest parent remains unobserved. A known nearest sample
  // with an unknown interpolation neighbor retains the nearest value.
  CHECK(out_weights.value()[at(6, 6, 6)] == 0.0f);
  CHECK(out_colors.value()[at(6, 6, 6)] == 0u);
  CHECK(out_sdf.value()[at(5, 5, 5)] == sdf[2 + 8 * (2 + 8 * 2)]);
  CHECK(out_weights.value()[at(5, 5, 5)] == 0.5f);
  CHECK(out_colors.value()[at(5, 5, 5)] == colors[2 + 8 * (2 + 8 * 2)]);
  // Interpolation cannot give a weak contributor its stronger neighbors'
  // confidence; the caller cap is an additional upper bound.
  CHECK(out_weights.value()[at(9, 9, 9)] == 0.25f);
  CHECK(std::abs(out_sdf.value()[at(9, 9, 9)] -
                 (4.25f * 73 - 255.5f) * 0.001f) < 1e-6f);
  // Unknown color does not invalidate geometry or become observed black.
  CHECK(out_colors.value()[at(3, 3, 3)] == 0u);
  CHECK(out_weights.value()[at(3, 3, 3)] == 0.5f);
  CHECK(std::abs(out_sdf.value()[at(3, 3, 3)] -
                 (1.25f * 73 - 255.5f) * 0.001f) < 1e-6f);
  CHECK(out_colors.value()[at(1, 1, 1)] == colors[0]);
  // The outer sample layer keeps its parent value instead of extrapolating.
  CHECK(out_sdf.value()[at(0, 0, 0)] == sdf[0]);
  CHECK(out_sdf.value()[at(15, 15, 15)] == sdf[511]);
  return 0;
}

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
  vol::HierarchicalGrid uncreated;
  CHECK(!uncreated.valid() && uncreated.node_capacity() == 0);
  CHECK(!uncreated.prepare_leaves() && !uncreated.clear());
  vol::HierarchicalGridConfig config;
  config.finest.voxel_size = 0.03125f;
  config.finest.trunc_dist = 1.0f;  // exactly the root extent at level 2
  config.finest.bucket_size = 8;
  config.finest.num_buckets = 16;
  config.finest.num_blocks = 128;
  config.level_count = 3;
  config.child_block_capacity = 16;
  config.color = true;
  auto saved_device = std::move(dev);
  CHECK(!vol::HierarchicalGrid::create(dev, alloc, config));
  dev = std::move(saved_device);
  auto saved_allocator = std::move(alloc);
  CHECK(!vol::HierarchicalGrid::create(dev, alloc, config));
  alloc = std::move(saved_allocator);
  auto created = vol::HierarchicalGrid::create(dev, alloc, config);
  CHECK(created.ok());
  auto grid = std::move(created).value();
  CHECK(grid.valid() && grid.node_capacity() == 144);
  auto empty = grid.prepare_leaves();
  CHECK(empty.ok() && empty->validate().ok() && empty->leaf_count == 0);
  CHECK(empty->nodes->mapped() == nullptr && empty->tsdf->mapped() == nullptr);

  // Reject the entire invalid batch before any allocation or view
  // invalidation. Child addressing must remain inside the signed cell domain.
  for (int axis = 0; axis < 3; ++axis) {
    for (auto extreme : {std::numeric_limits<std::int32_t>::min(),
                         std::numeric_limits<std::int32_t>::max()}) {
      vol::BlockIndex invalid_roots[2]{{vr::Vec3i(0), 0}, {vr::Vec3i(0), 0}};
      invalid_roots[1].coord[axis] = extreme;
      CHECK(!grid.allocate_roots(invalid_roots, 2));
      CHECK(empty->is_current());
    }
  }
  CHECK(!grid.allocate_roots(nullptr, 1));
  CHECK(empty->is_current());
  float boundary_depth = 1.0f;
  vr::DepthCameraParams far_camera{1, 1, 0, 0, 0.1f, 2.0f, 1, 1, vr::Mat4f(1)};
  far_camera.cam_to_world[3].x = 1e12f;
  CHECK(!grid.allocate_from_depth(
      {{vr::StorageInput(&boundary_depth), far_camera}}));
  CHECK(empty->is_current());
  far_camera.cam_to_world = vr::Mat4f(1);
  far_camera.fx = std::numeric_limits<float>::min();
  CHECK(!grid.allocate_from_depth(
      {{vr::StorageInput(&boundary_depth), far_camera}}));
  CHECK(empty->is_current());
  far_camera.fx = 1.0f;
  far_camera.cam_to_world[0][0] = std::numeric_limits<float>::quiet_NaN();
  CHECK(!grid.allocate_from_depth(
      {{vr::StorageInput(&boundary_depth), far_camera}}));
  CHECK(empty->is_current());
  CHECK(grid.prepare_leaves()->leaf_count == 0);

  const vol::BlockIndex roots[] = {{vr::Vec3i(-1, 0, 0), -1},
                                   {vr::Vec3i(0, 0, 0), -1},
                                   {vr::Vec3i(1, 0, 0), -1}};
  auto allocated = grid.allocate_roots(roots, 3);
  CHECK(allocated.ok() && allocated.value() == 0);
  CHECK(!empty->is_current());
  auto before = grid.prepare_leaves();
  CHECK(before.ok() && before->validate().ok() && before->leaf_count == 3);
  CHECK(grid.leaf_counts()[2] == 3 && grid.leaf_counts()[0] == 0);
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

  // An affine signed plane and alternating linear red/blue samples reveal
  // incorrect octants, cell-center offsets, encoded-color interpolation or
  // confidence growth. Interior prolongation must reproduce the plane.
  const auto samples = static_cast<std::size_t>(grid.node_capacity()) * 512u;
  std::vector<float> tsdf(samples, 0.0f), weight(samples, 0.0f);
  std::vector<std::uint32_t> color(samples, 0u);
  for (auto id : leaves_before.value()) {
    for (std::uint32_t local = 0; local < 512; ++local) {
      const auto i = static_cast<std::size_t>(id) * 512u + local;
      tsdf[i] = (static_cast<float>(local) - 255.5f) * 0.001f;
      weight[i] = 5.0f;
      color[i] = (local & 1u) != 0 ? 0xff0000ffu : 0xffff0000u;
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
  CHECK(grid.leaf_counts()[1] == 8 && grid.leaf_counts()[2] == 2);
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
        const auto fx = (local & 7u) + (oct & 1u) * 8u;
        const auto fy = ((local >> 3) & 7u) + ((oct >> 1) & 1u) * 8u;
        const auto fz = (local >> 6) + (oct >> 2) * 8u;
        const auto x = ((local & 7u) + (oct & 1u) * 8u) / 2u;
        const auto y = (((local >> 3) & 7u) + ((oct >> 1) & 1u) * 8u) / 2u;
        const auto z = ((local >> 6) + (oct >> 2) * 8u) / 2u;
        const auto src =
            static_cast<std::size_t>(parent.ptr) + x + 8u * (y + 8u * z);
        const auto dst = static_cast<std::size_t>(child.ptr) + local;
        if (fx > 0 && fx < 15 && fy > 0 && fy < 15 && fz > 0 && fz < 15) {
          const float px = float(fx) * 0.5f - 0.25f;
          const float py = float(fy) * 0.5f - 0.25f;
          const float pz = float(fz) * 0.5f - 0.25f;
          const float expected = (px + 8 * py + 64 * pz - 255.5f) * 0.001f;
          CHECK(std::abs(tsdf_after.value()[dst] - expected) < 1e-6f);
          const float fraction = px - std::floor(px);
          const float red =
              (int(std::floor(px)) & 1) != 0 ? 1 - fraction : fraction;
          CHECK(color_after.value()[dst] ==
                vr::pack_linear_to_srgb(vr::Vec3f(red, 0, 1 - red)));
        } else {
          CHECK(tsdf_after.value()[dst] == tsdf[src]);
          CHECK(color_after.value()[dst] == color[src]);
        }
        CHECK(weight_after.value()[dst] == 1.0f);
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

  // Coarsening needs all eight observed coarse requests for consecutive
  // updates. Losing one child's evidence resets the age, not merely pauses it.
  nodes_after = vr_test::read_back<vol::HierarchicalNode>(
      dev, alloc, *after->nodes, grid.node_capacity());
  CHECK(nodes_after.ok());
  std::uint32_t merge_parent = 0;
  for (auto id : leaves_before.value()) {
    if (nodes_after.value()[id].children != 0) {
      merge_parent = id;
      break;
    }
  }
  const auto first_child = nodes_after.value()[merge_parent].children - 1;
  std::fill(desired.begin(), desired.end(),
            std::numeric_limits<std::uint32_t>::max());
  for (std::uint32_t j = 0; j < 8; ++j) desired[first_child + j] = 2;
  CHECK(vr_test::write_back(dev, alloc, requests.value(), desired));
  auto merge = grid.merge(requests.value(), 1, 2);
  CHECK(merge.ok() && merge->merged == 0 && merge->pending == 1);
  desired[first_child] = std::numeric_limits<std::uint32_t>::max();
  CHECK(vr_test::write_back(dev, alloc, requests.value(), desired));
  merge = grid.merge(requests.value(), 1, 2);
  CHECK(merge.ok() && merge->merged == 0 && merge->pending == 0);
  desired[first_child] = 2;
  CHECK(vr_test::write_back(dev, alloc, requests.value(), desired));
  merge = grid.merge(requests.value(), 1, 2);
  CHECK(merge.ok() && merge->merged == 0 && merge->pending == 1);

  // An affine fine field restricts to the parent's sample positions. An
  // unobserved fine tap makes its coarse sample unobserved; encoded colors
  // average in linear light (half red / half blue becomes sRGB 188,0,188).
  for (std::uint32_t j = 0; j < 8; ++j) {
    const auto& n = nodes_after.value()[first_child + j];
    for (std::uint32_t local = 0; local < 512; ++local) {
      const auto i = static_cast<std::size_t>(n.ptr) + local;
      const float x = (static_cast<float>(n.coord.x * 8) +
                       static_cast<float>(local & 7u) + 0.5f) *
                      0.0625f;
      tsdf_after.value()[i] = x * 0.01f;
      weight_after.value()[i] = 2.0f;
      color_after.value()[i] = (local & 1u) != 0 ? 0xff0000ffu : 0xffff0000u;
    }
  }
  weight_after
      .value()[static_cast<std::size_t>(nodes_after.value()[first_child].ptr)] =
      0.0f;
  after = grid.prepare_leaves();
  CHECK(after.ok());
  CHECK(vr_test::write_back(dev, alloc, *after->tsdf, tsdf_after.value()));
  CHECK(vr_test::write_back(dev, alloc, *after->weight, weight_after.value()));
  CHECK(vr_test::write_back(dev, alloc, *after->color, color_after.value()));
  merge = grid.merge(requests.value(), 1, 2);
  CHECK(merge.ok() && merge->merged == 1 && merge->deferred == 0);
  after = grid.prepare_leaves();
  CHECK(after.ok() && after->leaf_count == 10);
  auto merged_nodes = vr_test::read_back<vol::HierarchicalNode>(
      dev, alloc, *after->nodes, grid.node_capacity());
  auto merged_sdf =
      vr_test::read_back<float>(dev, alloc, *after->tsdf, samples);
  auto merged_weight =
      vr_test::read_back<float>(dev, alloc, *after->weight, samples);
  auto merged_color =
      vr_test::read_back<std::uint32_t>(dev, alloc, *after->color, samples);
  CHECK(merged_nodes.ok() && merged_sdf.ok() && merged_weight.ok() &&
        merged_color.ok());
  const auto& restricted = merged_nodes.value()[merge_parent];
  CHECK(restricted.children == 0 && restricted.level == 2);
  const auto restricted_ptr = static_cast<std::size_t>(restricted.ptr);
  CHECK(merged_weight.value()[restricted_ptr] == 0.0f);
  CHECK(merged_sdf.value()[restricted_ptr] == 0.0f);
  CHECK(merged_color.value()[restricted_ptr] == 0u);
  for (std::uint32_t local = 1; local < 512; ++local) {
    const float expected = (static_cast<float>(restricted.coord.x * 8) +
                            static_cast<float>(local & 7u) + 0.5f) *
                           0.125f * 0.01f;
    CHECK(std::abs(merged_sdf.value()[restricted_ptr + local] - expected) <
          1e-7f);
    CHECK(merged_weight.value()[restricted_ptr + local] == 2.0f);
    CHECK(merged_color.value()[restricted_ptr + local] == 0xffbc00bcu);
  }
  for (std::uint32_t j = 0; j < 8; ++j)
    CHECK(merged_nodes.value()[first_child + j].ptr == -1);
  // The pool was exhausted. The immediately following split reuses exactly
  // the returned group, proving both ownership and capacity were restored.
  std::fill(desired.begin(), desired.end(),
            std::numeric_limits<std::uint32_t>::max());
  desired[merge_parent] = 0;
  CHECK(vr_test::write_back(dev, alloc, requests.value(), desired));
  split = grid.split(requests.value(), 1);
  CHECK(split.ok() && split->split == 1 && split->exhausted == 0);
  after = grid.prepare_leaves();
  CHECK(after.ok() && after->leaf_count == 17);
  merged_nodes = vr_test::read_back<vol::HierarchicalNode>(
      dev, alloc, *after->nodes, grid.node_capacity());
  CHECK(merged_nodes.ok());
  CHECK(merged_nodes.value()[merge_parent].children == first_child + 1);
  CHECK(!grid.merge(requests.value(), 1, 0));

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

  // Two eligible sibling groups compete for one bounded merge event; the
  // deferred candidate remains intact and completes on the following call.
  allocated = destination.allocate_roots(roots, 2);
  CHECK(allocated.ok() && allocated.value() == 0);
  std::fill(desired.begin(), desired.end(), 0u);
  CHECK(vr_test::write_back(dev, alloc, requests.value(), desired));
  split = destination.split(requests.value(), 2);
  CHECK(split.ok() && split->split == 2);
  std::fill(desired.begin(), desired.end(), 2u);
  CHECK(vr_test::write_back(dev, alloc, requests.value(), desired));
  merge = destination.merge(requests.value(), 1, 1);
  CHECK(merge.ok() && merge->merged == 1 && merge->deferred == 1);
  merge = destination.merge(requests.value(), 1, 1);
  CHECK(merge.ok() && merge->merged == 1 && merge->deferred == 0);
  after = destination.prepare_leaves();
  CHECK(after.ok() && after->leaf_count == 2);
  CHECK(destination.leaf_counts()[2] == 2);

  // A root containing one refined child cannot merge in the same call as
  // that child. Eligibility must be a snapshot before any mutation dispatch.
  CHECK(destination.clear());
  allocated = destination.allocate_roots(roots, 1);
  CHECK(allocated.ok() && allocated.value() == 0);
  std::fill(desired.begin(), desired.end(), 0u);
  CHECK(vr_test::write_back(dev, alloc, requests.value(), desired));
  split = destination.split(requests.value(), 1);
  CHECK(split.ok() && split->split == 1);
  split = destination.split(requests.value(), 1);
  CHECK(split.ok() && split->split == 1);
  after = destination.prepare_leaves();
  CHECK(after.ok() && after->leaf_count == 15);
  CHECK(destination.leaf_counts()[0] == 8 && destination.leaf_counts()[1] == 7);
  std::fill(desired.begin(), desired.end(), 2u);
  CHECK(vr_test::write_back(dev, alloc, requests.value(), desired));
  merge = destination.merge(requests.value(), 10, 1);
  CHECK(merge.ok() && merge->merged == 1);
  after = destination.prepare_leaves();
  CHECK(after.ok() && after->leaf_count == 8);
  CHECK(destination.leaf_counts()[1] == 8);
  merge = destination.merge(requests.value(), 10, 1);
  CHECK(merge.ok() && merge->merged == 1);
  after = destination.prepare_leaves();
  CHECK(after.ok() && after->leaf_count == 1);
  CHECK(destination.leaf_counts()[2] == 1);
  std::fill(desired.begin(), desired.end(), 0u);
  CHECK(vr_test::write_back(dev, alloc, requests.value(), desired));
  split = destination.split(requests.value(), 1);
  CHECK(split.ok() && split->split == 1);
  split = destination.split(requests.value(), 1);
  CHECK(split.ok() && split->split == 1);  // both returned groups are reusable

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
  CHECK(check_prolongation_support(dev, alloc) == 0);
  CHECK(validation_errors == 0);
  std::printf("recon hierarchical grid test passed\n");
  return 0;
}
