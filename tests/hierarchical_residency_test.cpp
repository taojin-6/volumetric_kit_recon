// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string_view>

#include "volumetric_kit/recon/core/compute_util.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/log.hpp"
#include "volumetric_kit/recon/core/vk_result.hpp"
#include "volumetric_kit/recon/mesh/hierarchical_marching_cubes.hpp"
#include "volumetric_kit/recon/tsdf/hierarchical_tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/hierarchical_grid.hpp"

namespace vr = volumetric_kit::recon;
namespace vol = vr::volume;
namespace tsdf = vr::tsdf;
namespace mesh = vr::mesh;

#define CHECK(c)                                                        \
  do {                                                                  \
    if (!(c)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); \
      return 1;                                                         \
    }                                                                   \
  } while (0)

namespace {
constexpr VkDeviceSize kBytes = 65536;
constexpr VkBufferUsageFlags kUsage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                      VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
                                      VK_BUFFER_USAGE_TRANSFER_DST_BIT;

// Allocate from each actual compatible type, rather than assuming a mapped
// helper is non-local on discrete GPUs or forging provenance on UMA.
vr::Result<std::optional<vr::Buffer>> raw_buffer(
    const vr::Device& device, std::uint32_t index,
    const VkPhysicalDeviceMemoryProperties& properties) {
  const auto& type = properties.memoryTypes[index];
  if ((type.propertyFlags & (VK_MEMORY_PROPERTY_PROTECTED_BIT |
                             VK_MEMORY_PROPERTY_LAZILY_ALLOCATED_BIT |
                             VK_MEMORY_PROPERTY_DEVICE_COHERENT_BIT_AMD)) != 0)
    return std::optional<vr::Buffer>{};
  VkBufferCreateInfo desc{};
  desc.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  desc.size = kBytes;
  desc.usage = kUsage;
  desc.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VkBuffer handle = VK_NULL_HANDLE;
  VR_VK_TRY(vkCreateBuffer(device.handle(), &desc, nullptr, &handle));
  VkMemoryRequirements needs{};
  vkGetBufferMemoryRequirements(device.handle(), handle, &needs);
  if ((needs.memoryTypeBits & (1u << index)) == 0) {
    vkDestroyBuffer(device.handle(), handle, nullptr);
    return std::optional<vr::Buffer>{};
  }
  VkMemoryAllocateInfo alloc{};
  alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  alloc.allocationSize = needs.size;
  alloc.memoryTypeIndex = index;
  VkDeviceMemory memory = VK_NULL_HANDLE;
  VkResult result = vkAllocateMemory(device.handle(), &alloc, nullptr, &memory);
  if (result == VK_SUCCESS)
    result = vkBindBufferMemory(device.handle(), handle, memory, 0);
  void* mapped = nullptr;
  if (result == VK_SUCCESS &&
      (type.propertyFlags & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT) != 0)
    result = vkMapMemory(device.handle(), memory, 0, VK_WHOLE_SIZE, 0, &mapped);
  if (result != VK_SUCCESS) {
    vkDestroyBuffer(device.handle(), handle, nullptr);
    if (memory != VK_NULL_HANDLE)
      vkFreeMemory(device.handle(), memory, nullptr);
    return vr::vk_error(result, "raw hierarchy test buffer");
  }
  return std::optional<vr::Buffer>(vr::Buffer(
      handle, kBytes, kUsage, VK_SHARING_MODE_EXCLUSIVE, mapped,
      [vk = device.handle(), handle, memory, mapped]() {
        if (mapped != nullptr) vkUnmapMemory(vk, memory);
        vkDestroyBuffer(vk, handle, nullptr);
        vkFreeMemory(vk, memory, nullptr);
      },
      vr::BufferMemoryInfo{type.propertyFlags, index, type.heapIndex}));
}

int check_inputs(vol::HierarchicalGrid& grid,
                 tsdf::HierarchicalTsdfIntegrator& integrator,
                 mesh::HierarchicalMarchingCubes& extractor,
                 const vr::Buffer& candidate, bool local) {
  auto prepared = grid.prepare_leaves();
  CHECK(prepared && prepared->leaf_count == 0);
  const auto field = prepared.value();
  const auto accepted = [local](const vr::Status& status) {
    return local ? status.ok()
                 : status.domain() == vr::Status::Code::InvalidArgument;
  };
  using Member = const vr::Buffer* vol::HierarchicalFieldView::*;
  for (const Member member :
       {&vol::HierarchicalFieldView::root_hash,
        &vol::HierarchicalFieldView::nodes,
        &vol::HierarchicalFieldView::leaf_indices,
        &vol::HierarchicalFieldView::tsdf, &vol::HierarchicalFieldView::weight,
        &vol::HierarchicalFieldView::color}) {
    CHECK((field.*member)->is_device_local());
    auto borrowed = field;
    borrowed.*member = &candidate;
    CHECK(accepted(borrowed.validate()));
    CHECK(accepted(integrator.integrate(borrowed, {})));
    CHECK(accepted(integrator.classify(borrowed, {}).status()));
    CHECK(accepted(extractor.extract_device(borrowed).status()));
  }
  // Zero budgets must still validate the borrowed request buffer, without
  // changing topology or relying on initialized desired levels.
  CHECK(accepted(grid.split(candidate, 0).status()));
  CHECK(accepted(grid.merge(candidate, 0).status()));
  CHECK(accepted(grid.update_topology(candidate, 0, 0).status()));

  const vr::DepthCameraParams camera{1,    1, 0, 0,           0.1f,
                                     2.0f, 1, 1, vr::Mat4f(1)};
  tsdf::FrameInput depth{{vr::StorageInput(candidate), camera}, nullptr};
  CHECK(accepted(integrator.integrate(field, {depth})));
  CHECK(accepted(integrator.classify(field, {depth}).status()));
  // Color and depth have independent borrowed-input validation, including
  // with an empty field whose kernels will not consume the frame.
  const float host_depth = 0.0f;
  tsdf::ColorFrame color{};
  color.buffer = &candidate;
  color.cam = {1, 1, 0, 0, 1, 1, vr::Mat4f(1)};
  tsdf::FrameInput rgbd{{vr::StorageInput(&host_depth), camera}, &color};
  CHECK(accepted(integrator.integrate(field, {rgbd})));
  CHECK(accepted(grid.allocate_from_depth({{depth.depth, camera}}).status()));
  return 0;
}

int run_tests() {
  vr::InstanceConfig config;
  config.enable_validation = true;
  auto instance = vr::Instance::create(config);
  if (!instance) return 77;
  auto physical = instance->select_physical_device();
  if (!physical) return 77;
  auto device = vr::Device::create(instance.value(), physical.value(), {});
  CHECK(device);
  auto allocator = vr::Allocator::create(instance->handle(), device.value());
  CHECK(allocator);
  vol::HierarchicalGridConfig grid_config;
  grid_config.finest.voxel_size = 0.01f;
  grid_config.finest.trunc_dist = 0.04f;
  grid_config.finest.num_buckets = 1;
  grid_config.finest.bucket_size = 8;
  grid_config.finest.num_blocks = 8;
  grid_config.child_block_capacity = 8;
  grid_config.color = true;
  auto grid = vol::HierarchicalGrid::create(device.value(), allocator.value(),
                                            grid_config);
  auto integrator = tsdf::HierarchicalTsdfIntegrator::create(device.value(),
                                                             allocator.value());
  auto extractor = mesh::HierarchicalMarchingCubes::create(device.value(),
                                                           allocator.value());
  CHECK(grid && integrator && extractor);
  auto owned = vr::device_storage_buffer(allocator.value(), kBytes);
  CHECK(owned && owned->is_device_local());
  // The backing remains alive in owned, but an adoption without provenance
  // must be rejected even when its real allocation happens to be local.
  const vr::Buffer unknown(owned->handle(), kBytes, kUsage,
                           VK_SHARING_MODE_EXCLUSIVE, nullptr, {});
  CHECK(check_inputs(grid.value(), integrator.value(), extractor.value(),
                     unknown, false) == 0);

  VkPhysicalDeviceMemoryProperties properties{};
  vkGetPhysicalDeviceMemoryProperties(physical.value(), &properties);
  bool saw_local = false, saw_nonlocal = false, saw_host_local = false;
  for (std::uint32_t i = 0; i < properties.memoryTypeCount; ++i) {
    auto allocated = raw_buffer(device.value(), i, properties);
    CHECK(allocated);
    if (!allocated.value()) continue;
    const auto& buffer = *allocated.value();
    const auto flags = properties.memoryTypes[i].propertyFlags;
    const bool local = (flags & VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) != 0;
    CHECK(buffer.memory_info()->properties == flags);
    CHECK(buffer.memory_info()->type_index == i);
    CHECK(buffer.memory_info()->heap_index ==
          properties.memoryTypes[i].heapIndex);
    CHECK(buffer.is_device_local() == local);
    {
      vr::CommandBatch zero(device.value(), allocator.value());
      CHECK(zero.zero(buffer, 0, kBytes));
      CHECK(zero.submit());
    }
    CHECK(check_inputs(grid.value(), integrator.value(), extractor.value(),
                       buffer, local) == 0);
    saw_local |= local;
    saw_nonlocal |= !local;
    saw_host_local |= local && buffer.mapped() != nullptr;
  }
  CHECK(saw_local);
  std::printf("hierarchical residency passed: nonlocal=%d mapped-local=%d\n",
              saw_nonlocal, saw_host_local);
  return 0;
}
}  // namespace

int main() {
  std::atomic<unsigned> errors{0};
  vr::set_log_handler([&](vr::LogLevel level, std::string_view message) {
    if (level == vr::LogLevel::Error) ++errors;
    if (level == vr::LogLevel::Error || level == vr::LogLevel::Warning)
      std::fprintf(stderr, "%.*s\n", int(message.size()), message.data());
  });
  const int result = run_tests();
  vr::set_log_handler({});
  return errors == 0 ? result : 1;
}
