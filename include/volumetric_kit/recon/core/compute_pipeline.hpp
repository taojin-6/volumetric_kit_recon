// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file compute_pipeline.hpp
/// @brief A compute `VkPipeline` and the `VkPipelineLayout` it is built on.

#include <cstdint>

#include "volumetric_kit/recon/core/export.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/core/unique_handle.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"

namespace volumetric_kit::recon {

class ShaderModule;

/// @brief Parameters for @ref ComputePipeline::create.
///
/// The pipeline layout is built from the descriptor-set layouts and
/// push-constant ranges the caller supplies here, rather than reflected from
/// the SPIR-V -- so the compute tier takes no reflection dependency. The
/// bindings declared here must match the shader's `layout(set=, binding=)`
/// declarations (a mismatch is undefined behaviour at dispatch, not a create
/// error).
struct ComputePipelineDesc {
  /// The compute stage (must be non-null and valid).
  const ShaderModule* shader = nullptr;
  /// Descriptor-set layouts, one per set index (may be null iff count is 0).
  const VkDescriptorSetLayout* set_layouts = nullptr;
  std::uint32_t set_layout_count = 0;
  /// Push-constant ranges (may be null iff count is 0).
  const VkPushConstantRange* push_ranges = nullptr;
  std::uint32_t push_range_count = 0;
  /// The shader entry-point name.
  const char* entry_point = "main";
};

/// @brief Owns a compute `VkPipeline` and its `VkPipelineLayout`.
class VR_CORE_API ComputePipeline {
 public:
  /// @brief Create a compute pipeline per @p desc.
  /// @param device  The device to create it on.
  /// @param desc    The compute stage, descriptor-set layouts, and push ranges.
  /// @return The pipeline, or a non-OK @ref Status:
  ///         @ref Status::Code::InvalidArgument for a null/invalid shader, null
  ///         entry point, a null array paired with a non-zero count, or a null
  ///         device; @ref Status::Code::Backend if pipeline-layout or pipeline
  ///         creation fails.
  static Result<ComputePipeline> create(VkDevice device,
                                        const ComputePipelineDesc& desc);

  ComputePipeline() noexcept = default;
  ComputePipeline(ComputePipeline&&) noexcept = default;
  ComputePipeline& operator=(ComputePipeline&&) noexcept = default;
  ComputePipeline(const ComputePipeline&) = delete;
  ComputePipeline& operator=(const ComputePipeline&) = delete;

  /// @return The pipeline handle (`VK_NULL_HANDLE` when empty).
  VkPipeline handle() const noexcept { return pipeline_.get(); }
  /// @return The pipeline layout (needed to bind descriptor sets / push).
  VkPipelineLayout layout() const noexcept { return layout_.get(); }
  /// @return `true` if this owns a pipeline.
  bool valid() const noexcept { return pipeline_.valid(); }

 private:
  // Declared layout-before-pipeline so member destruction (reverse order) frees
  // the pipeline first, then the layout it was built on.
  UniqueHandle<VkPipelineLayout, vkDestroyPipelineLayout> layout_;
  UniqueHandle<VkPipeline, vkDestroyPipeline> pipeline_;
};

}  // namespace volumetric_kit::recon
