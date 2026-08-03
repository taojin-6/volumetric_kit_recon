// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/core/compute_kernel.hpp"

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
                std::uint32_t max_groups) {
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
  return device.submit_single_time([&](VkCommandBuffer cmd) {
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
    // and (c) a renderer consuming the buffer as geometry.
    //
    // (c) is what interop seam B needs and what this used to omit. A mesh the
    // renderer draws directly is read at VERTEX_INPUT as vertex attributes,
    // indices and an indirect command -- none of which COMPUTE|HOST covers, so
    // the writes were never made visible to the stage that reads them. That
    // omission does not fail loudly: the draw gets whatever happens to be in
    // memory, which on a GPU that completed the dispatch anyway is usually the
    // right answer, right up until it is not.
    //
    // Named unconditionally rather than behind a flag on the dispatch. The
    // stages are a *destination* mask, so listing one nothing reads costs an
    // execution dependency the driver already had to satisfy for the host and
    // compute cases -- and the alternative, a per-dispatch knob, would have to
    // be threaded through every kernel this helper exists to keep uniform, to
    // save nothing measurable.
    VkMemoryBarrier barrier{};
    barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
    barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
    barrier.dstAccessMask =
        VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT |
        VK_ACCESS_HOST_READ_BIT | VK_ACCESS_VERTEX_ATTRIBUTE_READ_BIT |
        VK_ACCESS_INDEX_READ_BIT | VK_ACCESS_INDIRECT_COMMAND_READ_BIT;
    vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                         VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT |
                             VK_PIPELINE_STAGE_HOST_BIT |
                             VK_PIPELINE_STAGE_VERTEX_INPUT_BIT |
                             VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT,
                         0, 1, &barrier, 0, nullptr, 0, nullptr);
  });
}

}  // namespace volumetric_kit::recon
