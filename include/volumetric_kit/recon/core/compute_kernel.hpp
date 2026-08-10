// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file core/compute_kernel.hpp
/// @brief A compute kernel's bundled Vulkan resources plus a shared-pool
///        builder, so a tier declares each kernel once instead of maintaining
///        parallel layout / pipeline / descriptor-set members by hand.

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "volumetric_kit/recon/core/compute_pipeline.hpp"
#include "volumetric_kit/recon/core/descriptor.hpp"
#include "volumetric_kit/recon/core/export.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"

namespace volumetric_kit::recon {

// Forward-declared, not included: gpu_timer.hpp reaches Device and this header
// is included by every compute tier, so a full include would pull the timer's
// query-pool machinery into all of them for a defaulted-null parameter.
class GpuTimer;

class Device;

/// @brief One compute kernel's owned resources: its descriptor-set layout, the
///        pipeline built from its SPIR-V, and one descriptor set allocated from
///        the shared pool.
///
/// A resource bundle that replaces the three parallel `*_layout_` /
/// `*_pipeline_` / `*_set_` members a compute tier used to carry per kernel --
/// one member per kernel instead of three. @ref KernelSetBuilder populates it;
/// @ref dispatch runs it.
struct ComputeKernel {
  DescriptorSetLayout layout;  ///< Descriptor-set layout (N storage buffers).
  ComputePipeline pipeline;    ///< Pipeline built from the kernel's SPIR-V.
  DescriptorSet set;           ///< One set allocated from the shared pool.

  ComputeKernel() noexcept = default;
  ~ComputeKernel() = default;
  // The moves are hand-written (not defaulted) only to reset the copyable,
  // non-owning `set` on the source: `layout` and `pipeline` self-reset on move,
  // but a defaulted move would *copy* `set`, leaving a moved-from kernel with a
  // stale handle while valid() is false. Nulling it keeps a moved-from kernel
  // fully empty and its accessors consistent with valid().
  ComputeKernel(ComputeKernel&& other) noexcept
      : layout(std::move(other.layout)),
        pipeline(std::move(other.pipeline)),
        set(other.set) {
    other.set = {};
  }
  ComputeKernel& operator=(ComputeKernel&& other) noexcept {
    if (this != &other) {
      layout = std::move(other.layout);
      pipeline = std::move(other.pipeline);
      set = other.set;
      other.set = {};
    }
    return *this;
  }
  ComputeKernel(const ComputeKernel&) = delete;
  ComputeKernel& operator=(const ComputeKernel&) = delete;

  /// @return `true` once fully built -- its pipeline is live AND its set is
  ///         allocated. `false` before @ref KernelSetBuilder::build has
  ///         allocated the set, or when moved-from.
  bool valid() const noexcept { return pipeline.valid() && set.valid(); }
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
  ///
  /// @warning The builder retains a pointer to @p out until @ref build writes
  ///          its set, so every registered @p out must stay at a fixed address
  ///          from @ref add through @ref build -- do not move, reallocate, or
  ///          destroy the kernels (or a container holding them) in between, and
  ///          each must outlive both the builder and the returned pool. Storing
  ///          the kernels as stable members (or in a pre-reserved container) is
  ///          the intended usage.
  /// @param out       Receives the built layout + pipeline (and later the set).
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
/// clean error rather than risk invalid usage on a min-spec driver, and rejects
/// a null @p push with a non-zero @p push_size (mirroring
/// @ref ComputePipeline::create's push-range validation).
/// @return An OK @ref Status, or a non-OK one if @p groups exceeds
///         @p max_groups, @p push is null with @p push_size > 0, or the
///         submission fails.
///
/// @param timer  Optional @ref GpuTimer collecting a device span around this
///               dispatch, through @ref Device::submit_single_time's timed
///               overload. `nullptr` is exactly the untimed path.
/// @param label  Span label; borrowed on @ref StageRow::name's terms, so it
///               must outlive every read of the metrics the timer is published
///               into (a string literal). Ignored when @p timer is null.
///
/// Threading the timer *here* rather than at each tier's own submit is what
/// makes device timing uniform across every kernel in the repo: each tier
/// already routes through this helper for the workgroup guard and the barrier,
/// so a span costs it two arguments rather than its own submit path. The span
/// covers the recorded work alone -- bind, push, dispatch, barrier -- and
/// excludes the command-buffer allocate, the submit, and the fence wait around
/// it, which is precisely the difference a wall-clock stage row cannot show.
VR_CORE_API Status dispatch(Device& device, const ComputeKernel& kernel,
                            const void* push, std::uint32_t push_size,
                            std::uint32_t groups, std::uint32_t max_groups,
                            GpuTimer* timer = nullptr,
                            const char* label = nullptr);

}  // namespace volumetric_kit::recon
