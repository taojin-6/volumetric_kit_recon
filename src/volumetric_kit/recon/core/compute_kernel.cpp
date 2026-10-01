// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/core/compute_kernel.hpp"

#include <string>
#include <utility>
#include <vector>

#include "volumetric_kit/recon/core/command_batch.hpp"
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
  if (push != nullptr && push->offset != 0) {
    return named_failure(name, Status::invalid_argument(
                                   "the push range must start at offset 0"));
  }
  out.push_bytes = push != nullptr ? push->size : 0;
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

Result<KernelSets> allocate_kernel_sets(const Device& device,
                                        const ComputeKernel& kernel,
                                        std::uint32_t bindings,
                                        std::uint32_t count) {
  if (!kernel.valid() || bindings == 0 || count == 0) {
    return Status::invalid_argument(
        "allocate_kernel_sets: needs a built kernel, and bindings and count "
        "above 0");
  }
  VkDescriptorPoolSize size{};
  size.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  size.descriptorCount = bindings * count;
  KernelSets out;
  VR_ASSIGN(out.pool, DescriptorPool::create(device.handle(), &size, 1, count));
  out.sets.reserve(count);
  for (std::uint32_t i = 0; i < count; ++i) {
    VR_ASSIGN(DescriptorSet set, out.pool.allocate(kernel.layout.handle()));
    out.sets.push_back(set);
  }
  return out;
}

Status dispatch(Device& device, const ComputeKernel& kernel, const void* push,
                std::uint32_t push_size, std::uint32_t groups,
                std::uint32_t max_groups, GpuStageScope* stage) {
  // A batch of one, so the checks, the span, the label and the barrier have
  // one definition.
  CommandBatch batch(device);
  VR_TRY(batch.dispatch(kernel, push, push_size, groups, max_groups, stage));
  return batch.submit();
}

}  // namespace volumetric_kit::recon
