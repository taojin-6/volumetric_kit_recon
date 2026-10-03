// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <limits>
#include <utility>
#include <vector>

#include "buffer_readback.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/log.hpp"
#include "volumetric_kit/recon/tsdf/hierarchical_tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/hierarchical_grid.hpp"

namespace vr = volumetric_kit::recon;
namespace vol = vr::volume;
namespace tsdf = vr::tsdf;

#define CHECK(condition)                                           \
  do {                                                             \
    if (!(condition)) {                                            \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, \
                   #condition);                                    \
      return 1;                                                    \
    }                                                              \
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
  vr::InstanceConfig ic;
  ic.enable_validation = true;
  auto instance = vr::Instance::create(ic);
  if (!instance) return 0;
  auto gpu = instance.value().select_physical_device();
  if (!gpu) return 0;
  auto device_result = vr::Device::create(instance.value(), gpu.value(), {});
  CHECK(device_result.ok());
  auto device = std::move(device_result).value();
  auto allocator_result =
      vr::Allocator::create(instance.value().handle(), device);
  CHECK(allocator_result.ok());
  auto allocator = std::move(allocator_result).value();

  CHECK(!tsdf::HierarchicalTsdfIntegrator::create(device,
                                                  allocator_result.value())
             .ok());
  CHECK(!tsdf::HierarchicalTsdfIntegrator::create(device_result.value(),
                                                  allocator)
             .ok());

  vol::HierarchicalGridConfig config;
  config.finest = vol::VoxelGridParams{0.02f, 8, 512, 0.08f, 4, 32, 128, 128};
  config.level_count = 2;
  config.child_block_capacity = 16;
  auto grid_result = vol::HierarchicalGrid::create(device, allocator, config);
  CHECK(grid_result.ok());
  auto grid = std::move(grid_result).value();
  vol::BlockIndex root{vr::Vec3i(0, 0, 3), 0};
  CHECK(grid.allocate_roots(&root, 1).value() == 0);
  auto view_result = grid.prepare_leaves();
  CHECK(view_result.ok());
  auto view = view_result.value();
  CHECK(view.leaf_count == 1);

  auto integrator_result =
      tsdf::HierarchicalTsdfIntegrator::create(device, allocator);
  CHECK(integrator_result.ok());
  auto integrator = std::move(integrator_result).value();
  tsdf::HierarchicalTsdfIntegrator empty;
  CHECK(!empty.valid());
  vr::DepthCameraParams camera{32,   32, 16, 16,          0.1f,
                               2.0f, 32, 32, vr::Mat4f(1)};
  std::vector<float> depth(32 * 32, 1.01f);
  tsdf::FrameInput frame{{vr::StorageInput(depth.data()), camera}, nullptr};
  const std::vector<tsdf::FrameInput> frames{frame};
  CHECK(integrator.integrate(view, frames).ok());
  auto nodes = vr_test::read_back<vol::HierarchicalNode>(
      device, allocator, *view.nodes, view.node_capacity);
  auto leaves = vr_test::read_back<std::uint32_t>(
      device, allocator, *view.leaf_indices, view.leaf_count);
  auto distances = vr_test::read_back<float>(device, allocator, *view.tsdf,
                                             view.node_capacity * 512u);
  auto weights = vr_test::read_back<float>(device, allocator, *view.weight,
                                           view.node_capacity * 512u);
  CHECK(nodes.ok() && leaves.ok() && distances.ok() && weights.ok());
  const auto node = nodes.value()[leaves.value()[0]];
  // Independent analytic plane values distinguish cell centers from old nodes.
  for (int z = 0; z < 4; ++z) {
    const float world_z = (24.0f + float(z) + 0.5f) * 0.04f;
    const auto index = std::size_t(node.ptr) + std::size_t(z * 64);
    const float sdf = 1.01f - world_z;
    if (sdf < -0.08f) {
      CHECK(weights.value()[index] == 0);
      continue;
    }
    CHECK(std::abs(distances.value()[index] - sdf) < 2e-6f);
    const float expected_weight = (1.0f / (world_z * world_z)) *
                                  (sdf >= 0 ? 1.0f : (0.08f + sdf) / 0.08f);
    CHECK(std::abs(weights.value()[index] - expected_weight) < 2e-5f);
  }

  // Flat evidence requests the coarsest level. A foreground/background edge
  // requests refinement even though no fine fused field exists yet.
  auto requests = integrator.classify(view, frames);
  CHECK(requests.ok());
  auto desired = vr_test::read_back<std::uint32_t>(
      device, allocator, *requests.value(), view.node_capacity);
  CHECK(desired.ok() && desired.value()[leaves.value()[0]] == 1u);
  // Perspective depth on a slanted plane varies sharply with x but remains
  // geometrically planar: z - 0.5*x = 1.01. It must not force refinement.
  for (int y = 0; y < 32; ++y) {
    for (int x = 0; x < 32; ++x) {
      depth[std::size_t(y * 32 + x)] =
          1.01f / (1.0f - 0.5f * (float(x) + 0.5f - 16.0f) / 32.0f);
    }
  }
  requests = integrator.classify(view, frames);
  CHECK(requests.ok());
  desired = vr_test::read_back<std::uint32_t>(
      device, allocator, *requests.value(), view.node_capacity);
  CHECK(desired.ok() && desired.value()[leaves.value()[0]] == 1u);
  // A narrow valid silhouette surrounded by absent/nonfinite depth still
  // requests detail; no complete tangent patch exists on this foreground.
  for (float missing : {0.0f, std::numeric_limits<float>::quiet_NaN(),
                        std::numeric_limits<float>::infinity()}) {
    std::fill(depth.begin(), depth.end(), missing);
    for (int y = 8; y < 24; ++y)
      for (int x = 16; x < 18; ++x) depth[std::size_t(y * 32 + x)] = 1.01f;
    requests = integrator.classify(view, frames);
    CHECK(requests.ok());
    desired = vr_test::read_back<std::uint32_t>(
        device, allocator, *requests.value(), view.node_capacity);
    CHECK(desired.ok() && desired.value()[leaves.value()[0]] == 0u);
  }
  // No live frames must clear old evidence rather than failing descriptor
  // allocation or leaving the previous silhouette's refinement request.
  auto zero_frame = frame;
  zero_frame.camera.width = zero_frame.camera.height = 0;
  for (const auto& no_live : {std::vector<tsdf::FrameInput>{},
                              std::vector<tsdf::FrameInput>{zero_frame}}) {
    requests = integrator.classify(view, no_live);
    CHECK(requests.ok());
    desired = vr_test::read_back<std::uint32_t>(
        device, allocator, *requests.value(), view.node_capacity);
    CHECK(desired.ok());
    CHECK(std::all_of(desired.value().begin(), desired.value().end(),
                      [](std::uint32_t value) { return value == UINT32_MAX; }));
  }
  std::fill(depth.begin(), depth.end(), 1.01f);
  for (int y = 0; y < 32; ++y) {
    for (int x = 16; x < 32; ++x) depth[std::size_t(y * 32 + x)] = 1.15f;
  }
  requests = integrator.classify(view, frames);
  CHECK(requests.ok());
  desired = vr_test::read_back<std::uint32_t>(
      device, allocator, *requests.value(), view.node_capacity);
  CHECK(desired.ok() && desired.value()[leaves.value()[0]] == 0u);
  auto split = grid.split(*requests.value(), 1);
  CHECK(split.ok() && split.value().split == 1);
  CHECK(!view.is_current());
  CHECK(!integrator.integrate(view, frames).ok());
  view_result = grid.prepare_leaves();
  CHECK(view_result.ok() && view_result.value().leaf_count == 8);
  view = view_result.value();
  std::fill(depth.begin(), depth.end(), 1.01f);
  // New frames can correct the inherited low-confidence initialization.
  for (int i = 0; i < 30; ++i) CHECK(integrator.integrate(view, frames).ok());
  nodes = vr_test::read_back<vol::HierarchicalNode>(
      device, allocator, *view.nodes, view.node_capacity);
  leaves = vr_test::read_back<std::uint32_t>(
      device, allocator, *view.leaf_indices, view.leaf_count);
  distances = vr_test::read_back<float>(device, allocator, *view.tsdf,
                                        view.node_capacity * 512u);
  weights = vr_test::read_back<float>(device, allocator, *view.weight,
                                      view.node_capacity * 512u);
  CHECK(nodes.ok() && leaves.ok() && distances.ok() && weights.ok());
  std::size_t checked = 0;
  for (auto leaf : leaves.value()) {
    const auto& child = nodes.value()[leaf];
    CHECK(child.level == 0);
    for (int z = 0; z < 8; ++z) {
      const float world_z = (float(child.coord.z * 8 + z) + 0.5f) * 0.02f;
      const auto index = std::size_t(child.ptr) + std::size_t(z * 64);
      const float sdf = 1.01f - world_z;
      if (sdf >= -0.02f && sdf <= 0.02f && weights.value()[index] > 0) {
        CHECK(std::abs(distances.value()[index] - sdf) < 0.0002f);
        ++checked;
      }
    }
  }
  CHECK(checked > 0);

  // Invalid raw depths leave the field unchanged, including NaN/Inf bilinear
  // taps even if their interpolation coefficient would be zero.
  const auto before_invalid = distances.value();
  for (float invalid : {std::numeric_limits<float>::quiet_NaN(),
                        std::numeric_limits<float>::infinity()}) {
    std::fill(depth.begin(), depth.end(), invalid);
    CHECK(integrator.integrate(view, frames).ok());
    distances = vr_test::read_back<float>(device, allocator, *view.tsdf,
                                          view.node_capacity * 512u);
    CHECK(distances.ok() && distances.value() == before_invalid);
  }

  // Receded surfaces clear under Dynamic; no stale fine surface remains.
  std::fill(depth.begin(), depth.end(), 1.8f);
  CHECK(integrator.integrate(view, frames, 5, tsdf::IntegrationMode::Dynamic)
            .ok());
  weights = vr_test::read_back<float>(device, allocator, *view.weight,
                                      view.node_capacity * 512u);
  CHECK(weights.ok());
  for (auto leaf : leaves.value()) {
    const auto& child = nodes.value()[leaf];
    for (int i = 0; i < 512; ++i)
      CHECK(weights.value()[std::size_t(child.ptr) + i] == 0);
  }

  CHECK(!integrator
             .integrate(view, frames, std::numeric_limits<float>::infinity())
             .ok());
  tsdf::HierarchicalRefinementParams bad;
  bad.pixel_stride = 0;
  CHECK(!integrator.classify(view, frames, bad).ok());
  auto moved = std::move(integrator);
  CHECK(moved.valid() && !integrator.valid());
  auto* alias = &moved;
  moved = std::move(*alias);
  CHECK(moved.valid());
  auto replacement =
      tsdf::HierarchicalTsdfIntegrator::create(device, allocator);
  CHECK(replacement.ok());
  moved = std::move(replacement).value();
  CHECK(moved.valid() && !replacement.value().valid());
  CHECK(validation_errors == 0);
  std::puts(
      "hierarchical TSDF: analytic fusion, incoming refinement, stale views, "
      "and lifecycle passed");
  return 0;
}
