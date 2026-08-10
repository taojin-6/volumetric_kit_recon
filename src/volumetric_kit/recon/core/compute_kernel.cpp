// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/core/compute_kernel.hpp"

#include "volumetric_kit/recon/core/gpu_timer.hpp"

#include <vector>

#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/shader.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"

namespace volumetric_kit::recon {

Status KernelSetBuilder::add(ComputeKernel& out, const unsigned char* spv,
                             std::size_t spv_size, std::uint32_t bindings,
                             const VkPushConstantRange* push) {
  // The layout: `bindings` compute-stage storage buffers at 0..bindings-1 (the
  // caller's set-0 declarations match by index).
  std::vector<VkDescriptorSetLayoutBinding> b(bindings);
  for (std::uint32_t i = 0; i < bindings; ++i) {
    b[i].binding = i;
    b[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    b[i].descriptorCount = 1;
    b[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
  }
  VR_ASSIGN(out.layout,
            DescriptorSetLayout::create(device_, b.data(), bindings));

  // The pipeline from the embedded SPIR-V; the shader module is transient (the
  // pipeline does not retain it).
  VR_ASSIGN(
      ShaderModule module,
      ShaderModule::create(device_, reinterpret_cast<const std::uint32_t*>(spv),
                           spv_size));
  const VkDescriptorSetLayout layout_handle = out.layout.handle();
  ComputePipelineDesc desc;
  desc.shader = &module;
  desc.set_layouts = &layout_handle;
  desc.set_layout_count = 1;
  desc.push_ranges = push;
  desc.push_range_count = push != nullptr ? 1u : 0u;
  VR_ASSIGN(out.pipeline, ComputePipeline::create(device_, desc));

  kernels_.push_back(&out);
  descriptor_total_ += bindings;
  return {};
}

Result<DescriptorPool> KernelSetBuilder::build() {
  // One set per kernel, `descriptor_total_` storage-buffer descriptors overall.
  VkDescriptorPoolSize size{};
  size.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  size.descriptorCount = descriptor_total_;
  VR_ASSIGN(
      DescriptorPool pool,
      DescriptorPool::create(device_, &size, 1,
                             static_cast<std::uint32_t>(kernels_.size())));
  // Allocate every set before committing any into the caller's kernels, so a
  // mid-loop failure destroys the local pool (freeing the sets already made)
  // with the kernels untouched -- rather than leaving earlier kernels holding a
  // set whose pool is gone while valid() reports true.
  std::vector<DescriptorSet> sets;
  sets.reserve(kernels_.size());
  for (ComputeKernel* kernel : kernels_) {
    VR_ASSIGN(DescriptorSet set, pool.allocate(kernel->layout.handle()));
    sets.push_back(set);
  }
  for (std::size_t i = 0; i < kernels_.size(); ++i) {
    kernels_[i]->set = sets[i];
  }
  return pool;
}

Status dispatch(Device& device, const ComputeKernel& kernel, const void* push,
                std::uint32_t push_size, std::uint32_t groups,
                std::uint32_t max_groups, GpuTimer* timer, const char* label) {
  // A 1-D dispatch flattens the whole input onto groupCountX, but Vulkan only
  // guarantees maxComputeWorkGroupCount[0] >= 65535 -- an oversized input would
  // be invalid usage on a min-spec (mobile) driver. Reject it as a clean error
  // rather than risk a device-lost.
  if (groups > max_groups) {
    return Status::invalid_argument(
        "dispatch: workgroup count exceeds the device's "
        "maxComputeWorkGroupCount[0] -- input too large for a 1-D dispatch");
  }
  // A non-zero push_size with no data would read past a null pointer in
  // vkCmdPushConstants; reject it up front (mirrors ComputePipeline::create's
  // null-push-range check).
  if (push_size > 0 && push == nullptr) {
    return Status::invalid_argument(
        "dispatch: push is null with a non-zero push_size");
  }
  // The destination scope, widened for a renderer reading these writes as
  // geometry -- but only as far as the recording queue family allows.
  //
  // A mesh the renderer draws directly is read at VERTEX_INPUT as vertex
  // attributes and indices, and at DRAW_INDIRECT as a command; none of which
  // COMPUTE|HOST covers, so those writes were never made visible to the stages
  // that read them. That omission does not fail loudly -- the draw gets
  // whatever happens to be in memory, which on a GPU that completed the
  // dispatch anyway is usually the right answer, right up until it is not.
  //
  // Widened unconditionally in the *stage* sense (no per-dispatch knob, which
  // would have to be threaded through every kernel this helper exists to keep
  // uniform, to save an execution dependency the driver already had to satisfy
  // for the host and compute cases) but NOT unconditionally in the *capability*
  // sense: Vulkan permits a barrier to name only stages the recording command
  // buffer's queue family supports, and VK_PIPELINE_STAGE_VERTEX_INPUT_BIT
  // requires VK_QUEUE_GRAPHICS_BIT. recon requires only compute of the family
  // it is handed, so it can legitimately sit on a compute-only one -- a
  // discrete GPU's async-compute family, which is exactly what the shared
  // bootstrap's two-families plan picks there. Naming VERTEX_INPUT on such a
  // queue is invalid usage in *every* dispatch in every tier, and invisible on
  // Apple, where every MoltenVK family is graphics+compute. DRAW_INDIRECT needs
  // only graphics *or* compute, so it is always available here; the two access
  // bits that belong to VERTEX_INPUT travel with it.
  const bool family_has_graphics =
      (device.compute_family_flags() & VK_QUEUE_GRAPHICS_BIT) != 0;
  VkPipelineStageFlags dst_stages = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT |
                                    VK_PIPELINE_STAGE_HOST_BIT |
                                    VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT;
  VkAccessFlags dst_access =
      VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT |
      VK_ACCESS_HOST_READ_BIT | VK_ACCESS_INDIRECT_COMMAND_READ_BIT;
  if (family_has_graphics) {
    dst_stages |= VK_PIPELINE_STAGE_VERTEX_INPUT_BIT;
    dst_access |=
        VK_ACCESS_VERTEX_ATTRIBUTE_READ_BIT | VK_ACCESS_INDEX_READ_BIT;
  }
  // A cross-queue handoff needs a semaphore regardless, and a semaphore's
  // signal/wait already carries availability and visibility for every prior
  // write -- so on a compute-only family the renderer is reachable only that
  // way, and nothing is lost by omitting the stages Vulkan forbids naming here.
  const auto record = [&](VkCommandBuffer cmd) {
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                      kernel.pipeline.handle());
    const VkDescriptorSet set = kernel.set.handle();
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                            kernel.pipeline.layout(), 0, 1, &set, 0, nullptr);
    if (push_size > 0) {
      vkCmdPushConstants(cmd, kernel.pipeline.layout(),
                         VK_SHADER_STAGE_COMPUTE_BIT, 0, push_size, push);
    }
    vkCmdDispatch(cmd, groups, 1, 1);
    // Make this kernel's SSBO writes available and visible to (a) the next
    // dispatch's shader reads/writes, (b) a host read of the mapped results,
    // and (c) a renderer consuming the buffer as geometry -- (c) as far as this
    // queue family permits; see the scope built above.
    VkMemoryBarrier barrier{};
    barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
    barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
    barrier.dstAccessMask = dst_access;
    vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, dst_stages,
                         0, 1, &barrier, 0, nullptr, 0, nullptr);
  };
  // The untimed overload rather than the timed one with a null timer: they are
  // equivalent, but naming the plain one keeps a profiler-free build's call
  // graph free of the timer entirely.
  if (timer == nullptr) {
    return device.submit_single_time(record);
  }
  return device.submit_single_time(record, timer, label);
}

}  // namespace volumetric_kit::recon
