// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// A refused borrowed gate records nothing and does not age or change the grid.
// Valid gates at a nonzero offset still close and open fusion after refusal.

#include <algorithm>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include "volumetric_kit/core/testing/gpu_test.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/grid/testing/device_blocks.hpp"
#include "volumetric_kit/grid/testing/grid_readback.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"

namespace vr = volumetric_kit::recon;
namespace sensor = volumetric_kit::sensor;
namespace vkc = volumetric_kit::core;
namespace vol = volumetric_kit::grid::volume;
namespace tsdf = vr::tsdf;

namespace {
int gpu_main(vkc::test::GpuContext& gpu) {
  const vol::VoxelGridParams shape{0.01f, 8, 512, 0.03f, 8, 16, 128, 16};
  const vol::AttributeSpec attributes[] = {
      {vol::kTsdfAttribute, sizeof(float)},
      {vol::kWeightAttribute, sizeof(float)}};
  auto grid = vol::VoxelBlockGrid::create(gpu.device, gpu.allocator, shape,
                                          attributes, 2);
  auto integrator = tsdf::TsdfIntegrator::create(gpu.device, gpu.allocator);
  CHECK(grid.ok() && integrator.ok());
  const vol::BlockIndex block{vr::Vec3i(0, 0, 6), 0};
  auto placed = vgrid_test::allocate_blocks(gpu, grid->map(), &block, 1);
  CHECK(placed.ok() && *placed == 0);
  sensor::DepthCameraParams camera{};
  camera.fx = camera.fy = 32.0f;
  camera.cx = camera.cy = 16.0f;
  camera.width = camera.height = 32;
  camera.min_depth = 0.1f;
  camera.max_depth = 1.0f;
  camera.cam_to_world = vr::Mat4f(1.0f);
  const std::vector<float> depth(32 * 32, 0.54f);
  const std::vector<tsdf::FrameInput> frames = {
      {vkc::StorageInput(depth.data()), camera, nullptr}};
  const std::uint32_t words[] = {0, 1, 0};
  auto storage = vkc::test::upload_device_buffer(gpu.device, gpu.allocator,
                                                 words, sizeof(words));
  vkc::BufferDesc desc;
  desc.size = sizeof(words);
  desc.usage =
      VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  auto transfer = gpu.allocator.create_buffer(desc);
  desc.size = 7;
  desc.usage |= VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  auto truncated = gpu.allocator.create_buffer(desc);
  CHECK(storage.ok() && transfer.ok() && truncated.ok());
  vkc::Buffer empty;
  vkc::Buffer unknown(
      storage->handle(), storage->size(), storage->usage(),
      storage->sharing_mode(), nullptr, [] {}, std::nullopt);
  auto memory = storage->memory_info();
  CHECK(memory.has_value());
  memory->properties &= ~VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;
  vkc::Buffer nonlocal(
      storage->handle(), storage->size(), storage->usage(),
      storage->sharing_mode(), nullptr, [] {}, memory);
  vkc::Buffer oversized(
      storage->handle(), vkc::max_storage_buffer_range(gpu.device) + 4,
      storage->usage(), storage->sharing_mode(), nullptr, [] {},
      storage->memory_info());
  const vol::DeviceCount invalid[] = {
      {&*storage, std::numeric_limits<VkDeviceSize>::max() - 3, 1},
      {nullptr, 0, 1},
      {&empty, 0, 1},
      {&*transfer, 0, 1},
      {&*truncated, 4, 1},
      {&*storage, 1, 1},
      {&*storage, sizeof(words), 1},
      {&unknown, 0, 1},
      {&nonlocal, 0, 1},
      {&oversized, 0, 1}};
  const auto before = vgrid_test::read_attribute<float>(
      gpu.device, gpu.allocator, *grid, vol::kWeightAttribute);
  CHECK(before.ok());
  for (const auto& gate : invalid) {
    const auto tick = grid->map().tick();
    const auto counters = gpu.device.counters();
    vkc::CommandBatch batch(gpu.device, gpu.allocator);
    const auto status = integrator->record_integrate(
        batch, *grid, frames, 5.0f, tsdf::IntegrationMode::Classic, &gate);
    CHECK(status.domain() == vkc::Status::Code::InvalidArgument);
    CHECK(grid->map().tick() == tick);
    CHECK(batch.submit().ok());
    CHECK(gpu.device.counters().dispatches == counters.dispatches);
    auto after = vgrid_test::read_attribute<float>(
        gpu.device, gpu.allocator, *grid, vol::kWeightAttribute);
    CHECK(after.ok() && *after == *before);
  }
  for (const auto offset : {8u, 4u}) {
    const vol::DeviceCount gate{&*storage, offset, 1};
    vkc::CommandBatch batch(gpu.device, gpu.allocator);
    CHECK(integrator
              ->record_integrate(batch, *grid, frames, 5.0f,
                                 tsdf::IntegrationMode::Classic, &gate)
              .ok());
    CHECK(batch.submit().ok());
    auto after = vgrid_test::read_attribute<float>(
        gpu.device, gpu.allocator, *grid, vol::kWeightAttribute);
    CHECK(after.ok());
    if (offset == 8) {
      CHECK(*after == *before);
    } else {
      CHECK(std::any_of(after->begin(), after->end(),
                        [](float weight) { return weight > 0.0f; }));
    }
  }
  return 0;
}
}  // namespace

int main() {
  return vkc::test::run_on_gpu(vr::device_requirements(), gpu_main);
}
