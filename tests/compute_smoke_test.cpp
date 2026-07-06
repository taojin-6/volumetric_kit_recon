// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Compute smoke: the end-to-end proof of recon's Vulkan compute path. It builds
// the whole chain -- Instance -> Device -> Allocator -> Buffer -> Descriptor ->
// ComputePipeline -> record/dispatch/wait -> readback -- and asserts the shader
// wrote what it should. This is the "validate MoltenVK compute before building
// on it" de-risk gate the CLAUDE.md roadmap calls for: it exercises the real
// driver, not a mock.
//
// Like the availability smokes, it exits 0 (skip) when the environment has no
// Vulkan driver or compute device -- that is environmental, not a code defect.
// Every step past device creation is a hard assertion.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <vector>

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/buffer.hpp"
#include "volumetric_kit/recon/core/compute_pipeline.hpp"
#include "volumetric_kit/recon/core/descriptor.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/shader.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"

namespace vr = volumetric_kit::recon;

namespace {

// Read a compiled SPIR-V module (a stream of 32-bit words) into memory. Returns
// empty on any read error or a size that is not a whole number of words.
std::vector<std::uint32_t> load_spirv(const char* path) {
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file) {
    return {};
  }
  const std::streamsize size = file.tellg();
  if (size <= 0 || size % 4 != 0) {
    return {};
  }
  file.seekg(0);
  std::vector<std::uint32_t> words(static_cast<std::size_t>(size) / 4);
  file.read(reinterpret_cast<char*>(words.data()), size);
  if (!file) {
    return {};
  }
  return words;
}

}  // namespace

int main() {
  // --- Instance + device. Absent driver/device is a skip, not a failure.
  vr::Result<vr::Instance> instance = vr::Instance::create({});
  if (!instance) {
    std::fprintf(stderr, "no Vulkan instance (%s); skipping\n",
                 instance.status().message().c_str());
    return 0;
  }
  vr::Result<VkPhysicalDevice> gpu = instance.value().select_physical_device();
  if (!gpu) {
    std::fprintf(stderr, "no compute-capable device (%s); skipping\n",
                 gpu.status().message().c_str());
    return 0;
  }
  vr::Result<vr::Device> device =
      vr::Device::create(instance.value().handle(), gpu.value(), {});
  if (!device) {
    std::fprintf(stderr, "device create failed: %s\n",
                 device.status().message().c_str());
    return 1;
  }

  vr::Result<vr::Allocator> allocator =
      vr::Allocator::create(instance.value().handle(), device.value());
  if (!allocator) {
    std::fprintf(stderr, "allocator create failed: %s\n",
                 allocator.status().message().c_str());
    return 1;
  }

  // --- A host-visible storage buffer of N uints for the shader to fill. The
  // fusion tiers stage into device-local memory; a host-mapped SSBO is the
  // simplest end-to-end proof.
  constexpr std::uint32_t kCount = 1024;
  vr::BufferDesc buffer_desc;
  buffer_desc.size = kCount * sizeof(std::uint32_t);
  buffer_desc.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_desc.memory = vr::MemoryUsage::HostVisible;
  buffer_desc.mapped = true;
  vr::Result<vr::Buffer> buffer = allocator.value().create_buffer(buffer_desc);
  if (!buffer) {
    std::fprintf(stderr, "buffer create failed: %s\n",
                 buffer.status().message().c_str());
    return 1;
  }

  // --- Shader module from the SPIR-V the build compiled (path via
  // VR_FILL_SPV).
  const std::vector<std::uint32_t> code = load_spirv(VR_FILL_SPV);
  if (code.empty()) {
    std::fprintf(stderr, "could not read SPIR-V at %s\n", VR_FILL_SPV);
    return 1;
  }
  vr::Result<vr::ShaderModule> shader =
      vr::ShaderModule::create(device.value().handle(), code.data(),
                               code.size() * sizeof(std::uint32_t));
  if (!shader) {
    std::fprintf(stderr, "shader create failed: %s\n",
                 shader.status().message().c_str());
    return 1;
  }

  // --- Descriptor set layout: one storage buffer at binding 0, compute stage.
  VkDescriptorSetLayoutBinding binding{};
  binding.binding = 0;
  binding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  binding.descriptorCount = 1;
  binding.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
  vr::Result<vr::DescriptorSetLayout> layout =
      vr::DescriptorSetLayout::create(device.value().handle(), &binding, 1);
  if (!layout) {
    std::fprintf(stderr, "layout create failed: %s\n",
                 layout.status().message().c_str());
    return 1;
  }

  VkDescriptorPoolSize pool_size{};
  pool_size.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  pool_size.descriptorCount = 1;
  vr::Result<vr::DescriptorPool> pool =
      vr::DescriptorPool::create(device.value().handle(), &pool_size, 1, 1);
  if (!pool) {
    std::fprintf(stderr, "pool create failed: %s\n",
                 pool.status().message().c_str());
    return 1;
  }
  vr::Result<vr::DescriptorSet> set =
      pool.value().allocate(layout.value().handle());
  if (!set) {
    std::fprintf(stderr, "set allocate failed: %s\n",
                 set.status().message().c_str());
    return 1;
  }
  set.value().write_storage_buffer(0, buffer.value().handle(), 0,
                                   VK_WHOLE_SIZE);

  // --- Compute pipeline: explicit set layout + a uint push constant (count).
  VkPushConstantRange push{};
  push.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
  push.offset = 0;
  push.size = sizeof(std::uint32_t);
  VkDescriptorSetLayout set_layout = layout.value().handle();
  vr::ComputePipelineDesc pipeline_desc;
  pipeline_desc.shader = &shader.value();
  pipeline_desc.set_layouts = &set_layout;
  pipeline_desc.set_layout_count = 1;
  pipeline_desc.push_ranges = &push;
  pipeline_desc.push_range_count = 1;
  vr::Result<vr::ComputePipeline> pipeline =
      vr::ComputePipeline::create(device.value().handle(), pipeline_desc);
  if (!pipeline) {
    std::fprintf(stderr, "pipeline create failed: %s\n",
                 pipeline.status().message().c_str());
    return 1;
  }

  // --- Record + dispatch + wait (one-shot). The trailing barrier makes the
  // shader's writes available to the host read below.
  const VkDescriptorSet descriptor_set = set.value().handle();
  const std::uint32_t count = kCount;
  const vr::Status submitted =
      device.value().submit_single_time([&](VkCommandBuffer cmd) {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                          pipeline.value().handle());
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                pipeline.value().layout(), 0, 1,
                                &descriptor_set, 0, nullptr);
        vkCmdPushConstants(cmd, pipeline.value().layout(),
                           VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(count),
                           &count);
        const std::uint32_t groups = (count + 63u) / 64u;
        vkCmdDispatch(cmd, groups, 1, 1);

        VkMemoryBarrier barrier{};
        barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
        barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                             VK_PIPELINE_STAGE_HOST_BIT, 0, 1, &barrier, 0,
                             nullptr, 0, nullptr);
      });
  if (!submitted) {
    std::fprintf(stderr, "dispatch failed: %s\n", submitted.message().c_str());
    return 1;
  }

  // --- Verify the shader wrote values[i] == i.
  const auto* out = static_cast<const std::uint32_t*>(buffer.value().mapped());
  if (out == nullptr) {
    std::fprintf(stderr, "buffer was not host-mapped\n");
    return 1;
  }
  for (std::uint32_t i = 0; i < kCount; ++i) {
    if (out[i] != i) {
      std::fprintf(stderr, "mismatch at index %u: got %u, want %u\n", i, out[i],
                   i);
      return 1;
    }
  }

  std::printf(
      "recon compute smoke passed: dispatched %u threads through the compute "
      "pipeline; buffer readback matched\n",
      kCount);
  return 0;
}
