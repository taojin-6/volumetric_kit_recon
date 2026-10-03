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
#include "volumetric_kit/recon/core/compute_util.hpp"
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

  // Surface hits cover four front children. Optional support includes the
  // empty back children, but only for a smooth currently observed sibling
  // group. This exercises actual classifier -> hysteresis -> merge behavior.
  std::fill(depth.begin(), depth.end(), 1.01f);
  CHECK(integrator.integrate(view, frames).ok());
  requests = integrator.classify(view, frames);
  CHECK(requests.ok());
  desired = vr_test::read_back<std::uint32_t>(
      device, allocator, *requests.value(), view.node_capacity);
  CHECK(desired.ok());
  std::size_t surface_children = 0;
  for (auto leaf : leaves.value())
    if (desired.value()[leaf] == 1u) ++surface_children;
  CHECK(surface_children == 4);
  tsdf::HierarchicalRefinementParams merge_params;
  merge_params.support_coarsening = true;
  requests = integrator.classify(view, frames, merge_params);
  CHECK(requests.ok());
  desired = vr_test::read_back<std::uint32_t>(
      device, allocator, *requests.value(), view.node_capacity);
  CHECK(desired.ok());
  for (auto leaf : leaves.value()) CHECK(desired.value()[leaf] == 1u);
  auto merged = grid.merge(*requests.value(), 1, 2);
  CHECK(merged.ok() && merged->merged == 0 && merged->pending == 1);
  view_result = grid.prepare_leaves();
  CHECK(view_result.ok());
  view = view_result.value();

  // A historical weighted sample behind the current truncation band blocks
  // coarsening, even though another sibling still has valid surface evidence.
  std::uint32_t back_leaf = UINT32_MAX;
  for (auto leaf : leaves.value()) {
    if (nodes.value()[leaf].coord.z == 7) back_leaf = leaf;
  }
  CHECK(back_leaf != UINT32_MAX);
  const auto back_ptr = nodes.value()[back_leaf].ptr;
  for (float historical_weight : {1.0f, 0.0f}) {
    vr::CommandBatch update(device, allocator);
    CHECK(update
              .upload(*view.weight, VkDeviceSize(back_ptr) * sizeof(float),
                      &historical_weight, sizeof(float))
              .ok());
    CHECK(update.submit().ok());
    requests = integrator.classify(view, frames, merge_params);
    CHECK(requests.ok());
    desired = vr_test::read_back<std::uint32_t>(
        device, allocator, *requests.value(), view.node_capacity);
    CHECK(desired.ok());
    CHECK(desired.value()[back_leaf] ==
          (historical_weight > 0 ? UINT32_MAX : 1u));
    if (historical_weight > 0) {
      merged = grid.merge(*requests.value(), 1, 2);
      CHECK(merged.ok() && merged->merged == 0 && merged->pending == 0);
      view_result = grid.prepare_leaves();
      CHECK(view_result.ok());
      view = view_result.value();
    }
  }
  // Missing interior pixels and a camera with no surface hit in this sibling
  // group cannot create coarsening evidence. Fine surface votes still veto.
  for (bool no_surface : {false, true}) {
    std::fill(depth.begin(), depth.end(), no_surface ? 0.7f : 1.01f);
    if (!no_surface)
      for (int y = 18; y < 22; ++y)
        for (int x = 18; x < 22; ++x) depth[std::size_t(y * 32 + x)] = 0.0f;
    requests = integrator.classify(view, frames, merge_params);
    CHECK(requests.ok());
    merged = grid.merge(*requests.value(), 1, 2);
    CHECK(merged.ok() && merged->merged == 0 && merged->pending == 0);
    view_result = grid.prepare_leaves();
    CHECK(view_result.ok());
    view = view_result.value();
  }
  std::fill(depth.begin(), depth.end(), 1.01f);
  for (int pass = 0; pass < 2; ++pass) {
    requests = integrator.classify(view, frames, merge_params);
    CHECK(requests.ok());
    merged = grid.merge(*requests.value(), 1, 2);
    CHECK(merged.ok() && merged->merged == (pass == 1 ? 1u : 0u));
    view_result = grid.prepare_leaves();
    CHECK(view_result.ok());
    view = view_result.value();
  }
  view_result = grid.prepare_leaves();
  CHECK(view_result.ok() && view_result->leaf_count == 1u);
  view = view_result.value();

  // The same perfectly planar world-Z surface needs fine spacing when viewed
  // nearly tangentially through a narrow projective band. A wider common band
  // permits coarse spacing; no per-level truncation is introduced.
  const float grazing_cos = 0.15f;
  const float grazing_sin = std::sqrt(1.0f - grazing_cos * grazing_cos);
  auto grazing_camera = camera;
  grazing_camera.fx = grazing_camera.fy = 256.0f;
  grazing_camera.max_depth = 3.0f;
  grazing_camera.cam_to_world[0][0] = grazing_cos;
  grazing_camera.cam_to_world[0][2] = -grazing_sin;
  grazing_camera.cam_to_world[2][0] = grazing_sin;
  grazing_camera.cam_to_world[2][2] = grazing_cos;
  grazing_camera.cam_to_world[3].x = 0.08f - grazing_sin;
  grazing_camera.cam_to_world[3].y = 0.08f;
  grazing_camera.cam_to_world[3].z = 1.01f - grazing_cos;
  std::vector<float> grazing_depth(32 * 32);
  for (int y = 0; y < 32; ++y)
    for (int x = 0; x < 32; ++x) {
      float ray_x = (float(x) + 0.5f - 16.0f) / 256.0f;
      grazing_depth[std::size_t(y * 32 + x)] =
          grazing_cos / (grazing_cos - grazing_sin * ray_x);
    }
  const tsdf::FrameInput grazing_frame{
      {vr::StorageInput(grazing_depth.data()), grazing_camera}, nullptr};
  for (float band : {0.04f, 0.20f}) {
    auto grazing_config = config;
    grazing_config.finest.voxel_size = 0.005f;
    grazing_config.finest.trunc_dist = band;
    grazing_config.level_count = 3;
    auto grazing_grid_result =
        vol::HierarchicalGrid::create(device, allocator, grazing_config);
    CHECK(grazing_grid_result.ok());
    auto grazing_grid = std::move(grazing_grid_result).value();
    vol::BlockIndex grazing_root{vr::Vec3i(0, 0, 6), 0};
    CHECK(grazing_grid.allocate_roots(&grazing_root, 1).value() == 0);
    auto grazing_view = grazing_grid.prepare_leaves();
    CHECK(grazing_view.ok());
    auto grazing_requests =
        integrator.classify(grazing_view.value(), {grazing_frame});
    CHECK(grazing_requests.ok());
    auto grazing_leaves = vr_test::read_back<std::uint32_t>(
        device, allocator, *grazing_view->leaf_indices,
        grazing_view->leaf_count);
    auto grazing_desired = vr_test::read_back<std::uint32_t>(
        device, allocator, *grazing_requests.value(),
        grazing_view->node_capacity);
    CHECK(grazing_leaves.ok() && grazing_desired.ok());
    CHECK(grazing_desired.value()[grazing_leaves.value()[0]] ==
          (band < 0.1f ? 0u : 2u));
  }

  // Batched distinct camera/depth inputs must exactly match sequential calls.
  // The batch uses device depth for camera two, the sequential path host depth.
  auto batch_grid_result =
      vol::HierarchicalGrid::create(device, allocator, config);
  auto serial_grid_result =
      vol::HierarchicalGrid::create(device, allocator, config);
  CHECK(batch_grid_result.ok() && serial_grid_result.ok());
  auto batch_grid = std::move(batch_grid_result).value();
  auto serial_grid = std::move(serial_grid_result).value();
  CHECK(batch_grid.allocate_roots(&root, 1).value() == 0);
  CHECK(serial_grid.allocate_roots(&root, 1).value() == 0);
  auto batch_view = batch_grid.prepare_leaves();
  auto serial_view = serial_grid.prepare_leaves();
  CHECK(batch_view.ok() && serial_view.ok());
  std::vector<float> second_depth(32 * 32, 1.07f);
  auto second = frame;
  second.camera.cam_to_world[3].x = 0.1f;
  second.camera.cam_to_world[3].z = 0.03f;
  second.depth = vr::StorageInput(second_depth.data());
  auto device_depth =
      vr::device_storage_buffer(allocator, second_depth.size() * sizeof(float));
  CHECK(device_depth.ok());
  vr::CommandBatch upload_depth(device, allocator);
  CHECK(upload_depth
            .upload(device_depth.value(), 0, second_depth.data(),
                    second_depth.size() * sizeof(float))
            .ok());
  CHECK(upload_depth.submit().ok());
  auto second_device = second;
  second_device.depth = vr::StorageInput(device_depth.value());
  CHECK(integrator.integrate(batch_view.value(), {frame, second_device}).ok());
  CHECK(integrator.integrate(serial_view.value(), {frame}).ok());
  CHECK(integrator.integrate(serial_view.value(), {second}).ok());
  for (const auto pair :
       {std::make_pair(batch_view->tsdf, serial_view->tsdf),
        std::make_pair(batch_view->weight, serial_view->weight)}) {
    auto batched = vr_test::read_back<float>(device, allocator, *pair.first,
                                             batch_view->node_capacity * 512u);
    auto serial = vr_test::read_back<float>(device, allocator, *pair.second,
                                            serial_view->node_capacity * 512u);
    CHECK(batched.ok() && serial.ok() && batched.value() == serial.value());
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
