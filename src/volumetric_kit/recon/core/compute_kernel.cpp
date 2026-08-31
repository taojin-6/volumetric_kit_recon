// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/core/compute_kernel.hpp"

#include "volumetric_kit/recon/core/gpu_timer.hpp"

#include <string>
#include <utility>
#include <vector>

#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/shader.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"

namespace volumetric_kit::recon {

namespace {

// Prefix a build failure with the kernel it belongs to, preserving its domain
// and backend detail. A tier registers up to eight kernels in one create(), and
// the underlying Status names only the Vulkan call ("ComputePipeline::create:
// ..."), so without this a binding-count disagreement says nothing about
// *which* shader disagreed. Rebuilt through the public factories because the
// domain constructor is private -- deliberately, so a domain cannot be paired
// with a detail it did not come from.
Status named_failure(const char* name, const Status& why) {
  std::string what = (name != nullptr ? name : "<unnamed kernel>");
  what += ": ";
  what += why.message();
  switch (why.domain()) {
    case Status::Code::InvalidArgument:
      return Status::invalid_argument(std::move(what));
    case Status::Code::NotFound:
      return Status::not_found(std::move(what));
    case Status::Code::Unsupported:
      return Status::unsupported(std::move(what));
    case Status::Code::OutOfMemory:
      return Status::out_of_memory(std::move(what));
    case Status::Code::IoError:
      return Status::io_error(std::move(what));
    case Status::Code::Backend:
      return Status::backend_error(why.detail(), std::move(what));
    case Status::Code::Ok:
      break;
  }
  // Unreachable: only reached with an OK status, which no caller below passes.
  return why;
}

// VR_ASSIGN, but attributing the failure to the kernel being built. Every step
// of add() goes through it, so no build failure can reach a tier unnamed.
template <typename T>
Status assign_named(T& out, Result<T>&& from, const char* name) {
  if (!from) return named_failure(name, from.status());
  out = std::move(from).value();
  return {};
}

}  // namespace

Status KernelSetBuilder::add(ComputeKernel& out, const char* name,
                             const unsigned char* spv, std::size_t spv_size,
                             std::uint32_t bindings,
                             const VkPushConstantRange* push) {
  // Set before the first early return, so a kernel that fails to build is
  // named in the diagnostic that reports the failure (named_failure below).
  out.name = name;
  // The layout: `bindings` compute-stage storage buffers at 0..bindings-1 (the
  // caller's set-0 declarations match by index).
  std::vector<VkDescriptorSetLayoutBinding> b(bindings);
  for (std::uint32_t i = 0; i < bindings; ++i) {
    b[i].binding = i;
    b[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    b[i].descriptorCount = 1;
    b[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
  }
  VR_TRY(assign_named(
      out.layout,
      DescriptorSetLayout::create(device_->handle(), b.data(), bindings),
      name));

  // The pipeline from the embedded SPIR-V; the shader module is transient (the
  // pipeline does not retain it).
  ShaderModule module;
  VR_TRY(assign_named(
      module,
      ShaderModule::create(device_->handle(),
                           reinterpret_cast<const std::uint32_t*>(spv),
                           spv_size),
      name));
  const VkDescriptorSetLayout layout_handle = out.layout.handle();
  ComputePipelineDesc desc;
  desc.shader = &module;
  desc.set_layouts = &layout_handle;
  desc.set_layout_count = 1;
  desc.push_ranges = push;
  desc.push_range_count = push != nullptr ? 1u : 0u;
  VR_TRY(assign_named(out.pipeline,
                      ComputePipeline::create(device_->handle(), desc), name));

  // Name the objects a capture indexes by, not just the region the dispatch
  // records: Nsight groups a capture by VkPipeline and MoltenVK maps a named
  // pipeline onto its MTLComputePipelineState label, so an unnamed pipeline
  // leaves shader cost impossible to attribute back to a kernel -- which is the
  // question the labels exist to answer. Named here because this is where the
  // three handles are made and where the name is in scope.
  device_->set_object_name(VK_OBJECT_TYPE_PIPELINE,
                           debug_object_handle(out.pipeline.handle()), name);
  device_->set_object_name(VK_OBJECT_TYPE_PIPELINE_LAYOUT,
                           debug_object_handle(out.pipeline.layout()), name);
  device_->set_object_name(VK_OBJECT_TYPE_DESCRIPTOR_SET_LAYOUT,
                           debug_object_handle(out.layout.handle()), name);

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
      DescriptorPool::create(device_->handle(), &size, 1,
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
                std::uint32_t max_groups, GpuStageScope* stage) {
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
  // One call, timed or not: a null timer makes the timed overload of
  // submit_single_time byte-for-byte the untimed one (it delegates to exactly
  // this call), so branching on `stage` here would only name the same thing
  // twice.
  return device.submit_single_time(
      [&](VkCommandBuffer cmd) {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                          kernel.pipeline.handle());
        const VkDescriptorSet set = kernel.set.handle();
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                kernel.pipeline.layout(), 0, 1, &set, 0,
                                nullptr);
        if (push_size > 0) {
          vkCmdPushConstants(cmd, kernel.pipeline.layout(),
                             VK_SHADER_STAGE_COMPUTE_BIT, 0, push_size, push);
        }
        vkCmdDispatch(cmd, groups, 1, 1);
        // Make this kernel's SSBO writes available and visible to (a) the next
        // dispatch's shader reads/writes, (b) a host read of the mapped
        // results, and (c) a renderer consuming the buffer as geometry -- (c)
        // as far as this queue family permits; see the scope built above.
        VkMemoryBarrier barrier{};
        barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
        barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        barrier.dstAccessMask = dst_access;
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                             dst_stages, 0, 1, &barrier, 0, nullptr, 0,
                             nullptr);
      },
      stage != nullptr ? stage->timer() : nullptr,
      stage != nullptr ? stage->name() : nullptr,
      // The region a GPU profiler attributes this dispatch to. Passed rather
      // than recorded inside the lambda so submit_single_time can place it
      // outside the timestamp pair -- see that overload's `debug_label`.
      kernel.name);
}

}  // namespace volumetric_kit::recon
