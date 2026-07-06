// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file core/compute_kernel.hpp
/// @brief A compute kernel's bundled Vulkan resources plus a shared-pool
///        builder, so a tier declares each kernel once instead of maintaining
///        parallel layout / pipeline / descriptor-set members by hand.

#include <cstddef>
#include <cstdint>
#include <vector>

#include "volumetric_kit/recon/core/compute_pipeline.hpp"
#include "volumetric_kit/recon/core/descriptor.hpp"
#include "volumetric_kit/recon/core/export.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"

namespace volumetric_kit::recon {

class Device;

/// @brief One compute kernel's owned resources: its descriptor-set layout, the
///        pipeline built from its SPIR-V, and one descriptor set allocated from
///        the shared pool.
///
/// A resource bundle that replaces the three parallel `*_layout_` /
/// `*_pipeline_` / `*_set_` members a compute tier used to carry per kernel --
/// one member per kernel instead of three. @ref KernelSetBuilder populates it;
/// @ref dispatch runs it. Move-only follows from the members (rule of zero).
struct ComputeKernel {
  DescriptorSetLayout layout;  ///< Descriptor-set layout (N storage buffers).
  ComputePipeline pipeline;    ///< Pipeline built from the kernel's SPIR-V.
  DescriptorSet set;           ///< One set allocated from the shared pool.

  /// @return `true` once built (its pipeline is live; `false` when moved-from).
  bool valid() const noexcept { return pipeline.valid(); }
};

/// @brief Builds a group of @ref ComputeKernel that share one descriptor pool.
///
/// Register each kernel with @ref add (its SPIR-V, storage-buffer binding
/// count, and optional push-constant range), then @ref build sizes the pool to
/// the exact descriptor total, creates it, and allocates every kernel's set.
/// This resolves the pool chicken-and-egg -- sizing the pool needs every
/// kernel's binding count, but each set must come from the pool -- so a tier
/// never hand-sums `descriptorCount` / `maxSets`. Every binding is a
/// compute-stage storage buffer (the compute tiers' shape).
class VR_CORE_API KernelSetBuilder {
 public:
  /// @param device  The device the kernels are built on.
  explicit KernelSetBuilder(VkDevice device) noexcept : device_(device) {}

  KernelSetBuilder(const KernelSetBuilder&) = delete;
  KernelSetBuilder& operator=(const KernelSetBuilder&) = delete;

  /// @brief Register a kernel: build @p out's layout (@p bindings storage
  ///        buffers at 0..bindings-1) and its pipeline from the embedded SPIR-V
  ///        now; @p out's set is allocated later by @ref build.
  /// @param out       Receives the built layout + pipeline (and later the set);
  ///                  must outlive both the builder and the returned pool.
  /// @param spv       The SPIR-V byte array (4-byte aligned).
  /// @param spv_size  Its size in bytes.
  /// @param bindings  Number of storage-buffer bindings the shader declares.
  /// @param push      Optional push-constant range (`nullptr` = none).
  /// @return An OK @ref Status, or a non-OK one if the layout or the pipeline
  ///         fails to build.
  Status add(ComputeKernel& out, const unsigned char* spv, std::size_t spv_size,
             std::uint32_t bindings, const VkPushConstantRange* push = nullptr);

  /// @brief Create the shared pool (sized to every registered kernel) and
  ///        allocate each kernel's set from it.
  /// @return The pool (the caller owns it; it must outlive the kernels' sets),
  ///         or a non-OK @ref Status if the pool or a set allocation fails.
  Result<DescriptorPool> build();

 private:
  VkDevice device_;
  std::vector<ComputeKernel*> kernels_;
  std::uint32_t descriptor_total_ = 0;
};

/// @brief Record + submit a one-shot 1-D dispatch of @p kernel over @p groups
///        workgroups, followed by a barrier making its writes visible to the
///        next dispatch and to a host read.
///
/// Binds @p kernel's pipeline + set, pushes @p push_size bytes from @p push
/// (skipped when @p push_size is 0), dispatches, and emits a
/// COMPUTE->COMPUTE(+HOST) memory barrier -- each kernel runs as its own
/// fence-waited submission, and a fence orders execution but not memory, so
/// this barrier (not the fence) carries cross-dispatch visibility. Rejects
/// @p groups > @p max_groups (the device's `maxComputeWorkGroupCount[0]`) as a
/// clean error rather than risk invalid usage on a min-spec driver.
/// @return An OK @ref Status, or a non-OK one if @p groups exceeds
///         @p max_groups or the submission fails.
VR_CORE_API Status dispatch(Device& device, const ComputeKernel& kernel,
                            const void* push, std::uint32_t push_size,
                            std::uint32_t groups, std::uint32_t max_groups);

}  // namespace volumetric_kit::recon
