// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The pin that keeps ONE transfer curve from becoming two.
//
// core/color_space.hpp and core/shaders/color_common.glsl implement the same
// exact piecewise sRGB function on either side of the host/device boundary --
// they must, because the tsdf integrator decodes with the GLSL one while the
// examples encode with the host one, and the atlas is decoded by fixed-function
// _SRGB sampling that follows the same standard. Two implementations of one
// curve is precisely the silent drift the 2026-08-02 color-space decision
// exists to prevent, so this runs the device curve over all 256 codes on the
// real driver (MoltenVK on Apple, the NVIDIA ICD on the Linux CI box) and
// compares it against the host's, in the style of the existing on-device ABI
// round-trips.
//
// Two properties, and they fail differently:
//   * the decoded LINEAR values agree within float tolerance -- catches a
//     substituted pow(x, 2.2), a dropped linear toe, or a mistyped constant;
//   * the 8-bit ROUND TRIP is exact for every code -- catches a rounding-mode
//     difference between packUnorm4x8 and the host's +0.5, which the tolerance
//     above would happily absorb while quietly costing a code per fusion.
//
// Exits 0 (skip) where no device is present.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <vector>

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/buffer.hpp"
#include "volumetric_kit/recon/core/color_space.hpp"
#include "volumetric_kit/recon/core/compute_pipeline.hpp"
#include "volumetric_kit/recon/core/descriptor.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/core/shader.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"

namespace vr = volumetric_kit::recon;

namespace {

constexpr std::uint32_t kCodes = 256;

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

  auto make_buffer = [&](std::size_t bytes) {
    vr::BufferDesc desc;
    desc.size = bytes;
    desc.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
    desc.memory = vr::MemoryUsage::HostVisible;
    desc.mapped = true;
    return allocator.value().create_buffer(desc);
  };
  vr::Result<vr::Buffer> linear_buf = make_buffer(kCodes * sizeof(float));
  vr::Result<vr::Buffer> round_buf =
      make_buffer(kCodes * sizeof(std::uint32_t));
  if (!linear_buf || !round_buf) {
    std::fprintf(stderr, "buffer create failed\n");
    return 1;
  }

  const std::vector<std::uint32_t> code = load_spirv(VR_COLOR_PARITY_SPV);
  if (code.empty()) {
    std::fprintf(stderr, "could not read SPIR-V at %s\n", VR_COLOR_PARITY_SPV);
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

  VkDescriptorSetLayoutBinding bindings[2]{};
  for (std::uint32_t i = 0; i < 2; ++i) {
    bindings[i].binding = i;
    bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    bindings[i].descriptorCount = 1;
    bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
  }
  vr::Result<vr::DescriptorSetLayout> layout =
      vr::DescriptorSetLayout::create(device.value().handle(), bindings, 2);
  if (!layout) {
    std::fprintf(stderr, "layout create failed: %s\n",
                 layout.status().message().c_str());
    return 1;
  }
  VkDescriptorPoolSize pool_size{};
  pool_size.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  pool_size.descriptorCount = 2;
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
  set.value().write_storage_buffer(0, linear_buf.value().handle(), 0,
                                   VK_WHOLE_SIZE);
  set.value().write_storage_buffer(1, round_buf.value().handle(), 0,
                                   VK_WHOLE_SIZE);

  VkDescriptorSetLayout set_layout = layout.value().handle();
  vr::ComputePipelineDesc pipeline_desc;
  pipeline_desc.shader = &shader.value();
  pipeline_desc.set_layouts = &set_layout;
  pipeline_desc.set_layout_count = 1;
  vr::Result<vr::ComputePipeline> pipeline =
      vr::ComputePipeline::create(device.value().handle(), pipeline_desc);
  if (!pipeline) {
    std::fprintf(stderr, "pipeline create failed: %s\n",
                 pipeline.status().message().c_str());
    return 1;
  }

  const VkDescriptorSet descriptor_set = set.value().handle();
  const vr::Status submitted =
      device.value().submit_single_time([&](VkCommandBuffer cmd) {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                          pipeline.value().handle());
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                pipeline.value().layout(), 0, 1,
                                &descriptor_set, 0, nullptr);
        vkCmdDispatch(cmd, 1, 1, 1);  // one 256-wide workgroup: one per code

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

  const auto* gpu_linear =
      static_cast<const float*>(linear_buf.value().mapped());
  const auto* gpu_round =
      static_cast<const std::uint32_t*>(round_buf.value().mapped());
  if (gpu_linear == nullptr || gpu_round == nullptr) {
    std::fprintf(stderr, "buffers were not host-mapped\n");
    return 1;
  }

  // 1e-5 absolute is far tighter than the ~0.0036 gap between the exact curve
  // and pow(x, 2.2) at its worst, so a substituted approximation fails loudly
  // here rather than showing up later as a seam in the renderer between
  // textured and vertex-colored triangles.
  float worst = 0.0f;
  for (std::uint32_t c = 0; c < kCodes; ++c) {
    const auto packed = static_cast<std::uint32_t>(c) |
                        (static_cast<std::uint32_t>(c) << 8) |
                        (static_cast<std::uint32_t>(c) << 16) | 0xFF000000u;
    const vr::Vec3f host_linear = vr::unpack_srgb_to_linear(packed);
    const float diff = std::fabs(gpu_linear[c] - host_linear.x);
    if (diff > worst) {
      worst = diff;
    }
    if (!(diff < 1e-5f)) {
      std::fprintf(stderr,
                   "linear mismatch at code %u: gpu %.9f, host %.9f (diff "
                   "%.9f)\n",
                   c, static_cast<double>(gpu_linear[c]),
                   static_cast<double>(host_linear.x),
                   static_cast<double>(diff));
      return 1;
    }
    // The round trip must be EXACT on device, code for code -- the host test
    // asserts the same property host-side, so together they say the 8-bit voxel
    // attribute survives a fusion pass on either implementation.
    const std::uint32_t host_round = vr::pack_linear_to_srgb(host_linear);
    if (gpu_round[c] != host_round || (gpu_round[c] & 0xFFu) != c) {
      std::fprintf(stderr,
                   "round-trip mismatch at code %u: gpu 0x%08x, host 0x%08x\n",
                   c, gpu_round[c], host_round);
      return 1;
    }
  }

  std::printf(
      "recon core color-space GPU parity passed: the GLSL curve matched the "
      "host curve over all 256 codes (worst linear delta %.2e) and every "
      "8-bit round trip was exact on-device\n",
      static_cast<double>(worst));
  return 0;
}
